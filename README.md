# Tracking Gimbal with FOC Firmware

A two-axis camera gimbal that tracks a person using YOLOv8 on a Raspberry Pi and keeps the face centered in the frame. Two gearless BLDC motors are driven by STM32 firmware with encoder feedback, PID position control, and field-oriented control.

September to December 2025

Software: C (STM32 HAL), Python, YOLOv8, GStreamer  
Hardware: STM32F411, two GM3506 BLDC motors, two DRV8313 drivers, two AS5048A magnetic encoders, Raspberry Pi with camera

## System

```mermaid
flowchart LR
    CAM["Camera on the tilt arm"]
    subgraph PI["Raspberry Pi, Python"]
        GS["GStreamer capture<br/>640 x 480"]
        YO["YOLOv8 tracking<br/>person class, track IDs"]
        TS["Target selection<br/>one track ID"]
        OF["Offset to command<br/>dead zone, PD, low-pass"]
    end
    subgraph MCU["STM32F411, C"]
        RX["USART interrupt<br/>packet parser"]
        TG["Target angle<br/>per axis"]
        PID["PID position loop"]
        FOC["FOC voltage output<br/>SVPWM, 25 kHz"]
        ENC["AS5048A read<br/>SPI, 14-bit"]
    end
    DRV["DRV8313 x 2"]
    MOT["GM3506 x 2"]

    CAM --> GS --> YO --> TS --> OF
    OF -->|"USART 115200<br/>0xFF, x, y, 0xFE"| RX
    RX --> TG --> PID --> FOC -->|"PWM x 6"| DRV --> MOT
    MOT -->|rotor angle| ENC --> PID
    ENC -->|electrical angle| FOC
    MOT -.->|pan and tilt move the camera| CAM
```

The Raspberry Pi only determines the face position. The STM32 closes the motor control loop using encoder feedback, so delayed or dropped frames from the Raspberry Pi do not leave the motors without feedback.

## Control loop

```mermaid
flowchart LR
    C["x and y commands<br/>signed bytes"] --> R["dead zone, ramp"]
    R --> I["integrate into target angle<br/>limit ±90°"]
    I --> E(("+ / −"))
    S["rotor angle θ<br/>encoder − zero"] --> E
    E --> P["PID<br/>output Uq, limit 7 V"]
    P --> IP["inverse Park<br/>Ud = 0, θe = 7θ + 90°"]
    IP --> IC["inverse Clarke<br/>and centering"]
    IC --> T["TIM1 / TIM2<br/>compare registers"]
```

The loop runs every 1 ms for both axes.

## Startup

```mermaid
flowchart TD
    A["Enable drivers, start PWM"] --> B["Hold both motors at a fixed electrical angle<br/>3.5 V pan, 4.5 V tilt, 2.5 s"]
    B --> C["Store each encoder reading as zero"]
    C --> D["Release, wait 0.5 s"]
    D --> E["Start USART reception"]
    E --> F["1 ms control loop"]
```

## Packet

| Byte | Value |
|---|---|
| 0 | `0xFF` start |
| 1 | x command, signed |
| 2 | y command, signed |
| 3 | `0xFE` end |

Values within a dead zone of ±5 are ignored. If no packet arrives for 2 s, the command is set to zero and the gimbal holds its position.

## Result

![Offset of the target from the image center while the person sits, stands up and moves](docs/tracking_offset.png)

Measured from the demo video over 287 frames. When the person stands up, the face reaches an offset of 178 px in the 480 px image. It returns to within 20 px of center 1.5 s later.

## What I built

- System architecture with vision on the Raspberry Pi and motor control on the STM32, connected by USART
- STM32 firmware in C with SPI encoder input, PID position control, and FOC with SVPWM
- Integration of the vision and motor systems and the 3D-printed frame

## Repository

```text
firmware/Core/Src/main.c   control loop, encoder, FOC, USART
firmware/Core/Inc/main.h
firmware/gimbal.ioc        STM32CubeMX configuration
docs/tracking_offset.png
```

## Build

Open `firmware/gimbal.ioc` in STM32CubeIDE, generate the code, and replace the generated `main.c` and `main.h` with the files from this repository.

| Signal | Pins |
|---|---|
| Pan motor PWM (TIM1) | PA8, PA9, PA10 |
| Tilt motor PWM (TIM2) | PA0, PA1, PA2 |
| Encoders (SPI1) | PA5 SCK, PA6 MISO, PA7 MOSI, PB0 and PB1 chip select |
| Driver enable | PB12, PB13 |
| USART1 from the Raspberry Pi | PB6 TX, PB7 RX |