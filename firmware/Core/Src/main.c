#include "main.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define VBUS             12.0f
#define VOLTAGE_LIMIT    7.0f
#define POLE_PAIRS       7.0f
#define ALIGN_VOLTAGE_M1 3.5f
#define ALIGN_VOLTAGE_M2 4.5f
#define ALIGN_TIME_MS    2500u

#define CMD_DEADZONE     5
#define TIMEOUT_MS       2000u
#define RAMP_ACCEL       0.05f
#define RAMP_BRAKE       0.05f
#define TARGET_RATE      (0.0002f / POLE_PAIRS)
#define TARGET_LIMIT     1.57f

#define LED_PORT         GPIOC
#define LED_PIN          GPIO_PIN_13
#define DRV_EN_PORT      GPIOB
#define DRV_EN1_PIN      GPIO_PIN_12
#define DRV_EN2_PIN      GPIO_PIN_13
#define ENC_CS_PORT      GPIOB
#define ENC1_CS_PIN      GPIO_PIN_0
#define ENC2_CS_PIN      GPIO_PIN_1

#define PI_F             3.14159265358979f
#define TWO_PI_F         6.28318530717959f
#define SQRT3_2_F        0.86602540378444f

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_err;
  float out_limit;
} Pid;

typedef struct {
  TIM_HandleTypeDef *htim;
  uint16_t cs_pin;
  float offset;
  float target;
  float speed;
  Pid pid;
} Axis;

SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

static uint8_t rx_byte = 0;
static volatile int8_t cmd_x = 0;
static volatile int8_t cmd_y = 0;
static volatile uint32_t last_pkt_time = 0;

static Axis pan  = { &htim1, ENC1_CS_PIN, 0.0f, 0.0f, 0.0f, { 12.0f, 0.0f, 0.0f, 0.0f, 0.0f, VOLTAGE_LIMIT } };
static Axis tilt = { &htim2, ENC2_CS_PIN, 0.0f, 0.0f, 0.0f, { 12.0f, 0.0f, 0.0f, 0.0f, 0.0f, VOLTAGE_LIMIT } };

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

static float constrain_f(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float wrap_pi(float a)
{
  while (a > PI_F) a -= TWO_PI_F;
  while (a < -PI_F) a += TWO_PI_F;
  return a;
}

static float read_as5048a(uint16_t cs_pin)
{
  uint16_t tx = 0xFFFF;
  uint16_t rx = 0;
  HAL_GPIO_WritePin(ENC_CS_PORT, cs_pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx, (uint8_t *)&rx, 1, 10);
  HAL_GPIO_WritePin(ENC_CS_PORT, cs_pin, GPIO_PIN_SET);
  return (float)(rx & 0x3FFF) / 16384.0f * TWO_PI_F;
}

static void set_phase_voltage(TIM_HandleTypeDef *htim, float uq, float ud, float angle_el)
{
  uq = constrain_f(uq, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);

  float ca = cosf(angle_el);
  float sa = sinf(angle_el);
  float u_alpha = ca * ud - sa * uq;
  float u_beta  = sa * ud + ca * uq;

  float ua = u_alpha;
  float ub = -0.5f * u_alpha + SQRT3_2_F * u_beta;
  float uc = -0.5f * u_alpha - SQRT3_2_F * u_beta;

  float center = VBUS / 2.0f - (fmaxf(ua, fmaxf(ub, uc)) + fminf(ua, fminf(ub, uc))) / 2.0f;
  ua += center;
  ub += center;
  uc += center;

  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)(constrain_f(ua / VBUS, 0.0f, 1.0f) * arr));
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)(constrain_f(ub / VBUS, 0.0f, 1.0f) * arr));
  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint32_t)(constrain_f(uc / VBUS, 0.0f, 1.0f) * arr));
}

static float pid_update(Pid *p, float err)
{
  p->integral = constrain_f(p->integral + err, -p->out_limit, p->out_limit);
  float out = p->kp * err + p->ki * p->integral + p->kd * (err - p->prev_err);
  p->prev_err = err;
  return constrain_f(out, -p->out_limit, p->out_limit);
}

static void ramp(float *speed, float target)
{
  int braking = (target == 0.0f) || (target * (*speed) < 0.0f);
  float step = braking ? RAMP_BRAKE : RAMP_ACCEL;
  float diff = target - *speed;
  if (diff > step) *speed += step;
  else if (diff < -step) *speed -= step;
  else *speed = target;
}

static void align(Axis *a, float voltage)
{
  set_phase_voltage(a->htim, voltage, 0.0f, 0.0f);
}

static void axis_step(Axis *a, int8_t cmd)
{
  float c = (abs(cmd) < CMD_DEADZONE) ? 0.0f : (float)cmd;
  ramp(&a->speed, c);
  a->target = constrain_f(a->target + a->speed * TARGET_RATE, -TARGET_LIMIT, TARGET_LIMIT);

  float angle = wrap_pi(read_as5048a(a->cs_pin) - a->offset);
  float uq = pid_update(&a->pid, wrap_pi(a->target - angle));
  set_phase_voltage(a->htim, uq, 0.0f, angle * POLE_PAIRS + PI_F / 2.0f);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  static uint8_t state = 0;
  static int8_t x = 0;
  static int8_t y = 0;

  if (huart->Instance != USART1) return;

  switch (state) {
    case 0: if (rx_byte == 0xFF) state = 1; break;
    case 1: x = (int8_t)rx_byte; state = 2; break;
    case 2: y = (int8_t)rx_byte; state = 3; break;
    case 3:
      if (rx_byte == 0xFE) {
        cmd_x = x;
        cmd_y = y;
        last_pkt_time = HAL_GetTick();
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
      }
      state = 0;
      break;
  }
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN1_PIN | DRV_EN2_PIN, GPIO_PIN_SET);
  HAL_Delay(100);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  __HAL_TIM_MOE_ENABLE(&htim1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  align(&pan, ALIGN_VOLTAGE_M1);
  align(&tilt, ALIGN_VOLTAGE_M2);
  HAL_Delay(ALIGN_TIME_MS);
  pan.offset = read_as5048a(pan.cs_pin);
  tilt.offset = read_as5048a(tilt.cs_pin);
  align(&pan, 0.0f);
  align(&tilt, 0.0f);
  HAL_Delay(500);

  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  last_pkt_time = HAL_GetTick();

  while (1) {
    if (HAL_GetTick() - last_pkt_time > TIMEOUT_MS) {
      cmd_x = 0;
      cmd_y = 0;
    }
    axis_step(&pan, cmd_x);
    axis_step(&tilt, cmd_y);
    HAL_Delay(1);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState = RCC_HSI_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  osc.PLL.PLLM = 8;
  osc.PLL.PLLN = 100;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN1_PIN | DRV_EN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ENC_CS_PORT, ENC1_CS_PIN | ENC2_CS_PIN, GPIO_PIN_SET);

  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin = LED_PIN;
  HAL_GPIO_Init(LED_PORT, &g);
  g.Pin = DRV_EN1_PIN | DRV_EN2_PIN;
  HAL_GPIO_Init(DRV_EN_PORT, &g);
  g.Pin = ENC1_CS_PIN | ENC2_CS_PIN;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ENC_CS_PORT, &g);
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void pwm_init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance)
{
  TIM_MasterConfigTypeDef master = {0};
  TIM_OC_InitTypeDef oc = {0};

  htim->Instance = instance;
  htim->Init.Prescaler = 0;
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = 3999;
  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim->Init.RepetitionCounter = 0;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(htim) != HAL_OK) Error_Handler();

  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim, &master) != HAL_OK) Error_Handler();

  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(htim, &oc, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(htim, &oc, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(htim, &oc, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
}

static void MX_TIM1_Init(void)
{
  TIM_BreakDeadTimeConfigTypeDef bdt = {0};

  pwm_init(&htim1, TIM1);
  bdt.OffStateRunMode = TIM_OSSR_DISABLE;
  bdt.OffStateIDLEMode = TIM_OSSI_DISABLE;
  bdt.LockLevel = TIM_LOCKLEVEL_OFF;
  bdt.DeadTime = 0;
  bdt.BreakState = TIM_BREAK_DISABLE;
  bdt.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  bdt.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &bdt) != HAL_OK) Error_Handler();
  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void)
{
  pwm_init(&htim2, TIM2);
  HAL_TIM_MspPostInit(&htim2);
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {
  }
}
