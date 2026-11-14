#include "main.h"
#include "foc.h"
#include <stdint.h>

#define ALIGN_VOLTAGE_PAN  3.5f
#define ALIGN_VOLTAGE_TILT 4.5f
#define ALIGN_TIME_MS      2500u
#define TIMEOUT_MS         2000u

#define PID_KP             12.0f
#define PID_KI             0.0f
#define PID_KD             0.1f

#define LED_PORT           GPIOC
#define LED_PIN            GPIO_PIN_13
#define DRV_EN_PORT        GPIOB
#define DRV_EN1_PIN        GPIO_PIN_12
#define DRV_EN2_PIN        GPIO_PIN_13
#define ENC_CS_PORT        GPIOB
#define ENC1_CS_PIN        GPIO_PIN_0
#define ENC2_CS_PIN        GPIO_PIN_1

#define TWO_PI_F           6.28318530717959f

typedef struct {
  TIM_HandleTypeDef *htim;
  uint16_t cs_pin;
  Axis axis;
} Motor;

SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

static uint8_t rx_byte = 0;
static volatile int8_t cmd_x = 0;
static volatile int8_t cmd_y = 0;
static volatile uint32_t last_pkt_time = 0;

static Motor pan  = { .htim = &htim1, .cs_pin = ENC1_CS_PIN };
static Motor tilt = { .htim = &htim2, .cs_pin = ENC2_CS_PIN };

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

static float read_as5048a(uint16_t cs_pin)
{
  uint16_t tx = 0xFFFF;
  uint16_t rx = 0;
  HAL_GPIO_WritePin(ENC_CS_PORT, cs_pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&tx, (uint8_t *)&rx, 1, 10);
  HAL_GPIO_WritePin(ENC_CS_PORT, cs_pin, GPIO_PIN_SET);
  return (float)(rx & 0x3FFF) / 16384.0f * TWO_PI_F;
}

static void apply(Motor *m, float uq, float angle_el)
{
  float duty[3];
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(m->htim);

  foc_phase_duties(uq, 0.0f, angle_el, duty);
  __HAL_TIM_SET_COMPARE(m->htim, TIM_CHANNEL_1, (uint32_t)(duty[0] * arr));
  __HAL_TIM_SET_COMPARE(m->htim, TIM_CHANNEL_2, (uint32_t)(duty[1] * arr));
  __HAL_TIM_SET_COMPARE(m->htim, TIM_CHANNEL_3, (uint32_t)(duty[2] * arr));
}

static void motor_step(Motor *m, int8_t cmd)
{
  FocCommand c = axis_update(&m->axis, cmd, read_as5048a(m->cs_pin));
  apply(m, c.uq, c.angle_el);
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

  axis_init(&pan.axis, PID_KP, PID_KI, PID_KD);
  axis_init(&tilt.axis, PID_KP, PID_KI, PID_KD);

  apply(&pan, ALIGN_VOLTAGE_PAN, 0.0f);
  apply(&tilt, ALIGN_VOLTAGE_TILT, 0.0f);
  HAL_Delay(ALIGN_TIME_MS);
  axis_set_zero(&pan.axis, read_as5048a(pan.cs_pin));
  axis_set_zero(&tilt.axis, read_as5048a(tilt.cs_pin));
  apply(&pan, 0.0f, 0.0f);
  apply(&tilt, 0.0f, 0.0f);
  HAL_Delay(500);

  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  last_pkt_time = HAL_GetTick();

  while (1) {
    if (HAL_GetTick() - last_pkt_time > TIMEOUT_MS) {
      cmd_x = 0;
      cmd_y = 0;
    }
    motor_step(&pan, cmd_x);
    motor_step(&tilt, cmd_y);
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
