# Carrier PCB

A 2-layer carrier board that the BlackPill (STM32F411) plugs into. It holds the
motor drivers, power, the IMU header, the ESP32-C3 BLE bridge and the
connectors. Full parts list: [`bom.csv`](bom.csv).

This file is the spec the KiCad project (`hardware/kicad/`) is drawn from:
every part, net and layout rule the schematic and board must follow.

## What is on it

| Block | Part | Why this part |
|---|---|---|
| MCU | WeAct BlackPill STM32F411CEU6 (socketed) | Cortex-M4F with FPU at 100 MHz, cheap, easy to swap if I kill one |
| IMU | MPU-6050 on a GY-521 module (socketed) | Well documented, 1 kHz internal rate, data-ready pin for interrupt-driven reads |
| Motor drivers | 2x TI DRV8874 | 6 A peak, built-in current sense (IPROPI), overcurrent/thermal shutdown with an nFAULT pin, simple PH/EN interface |
| BLE | ESP32-C3-MINI-1 | Full BLE 5 stack, programmable so I control the GATT layout, about $2 |
| 5 V rail | TI TPS563201 buck, 3 A | 12.6 V in, efficient, small. Feeds the BlackPill 5 V pin |
| 3.3 V for ESP32 | AP2112K-3.3 LDO | Keeps radio current spikes off the MCU/IMU 3.3 V rail |
| Battery | 3S LiPo 1300 mAh (11.1 V nominal, 12.6 V full) | Enough voltage for 12 V motors, about 20 min of balancing |
| Motors | JGB37-520 12 V 330 rpm with Hall encoders | 30:1 gearbox, 1320 counts per output rev at 4x decoding |

## Power tree

```
3S LiPo (9.9-12.6 V) -> XT30 -> 5 A PTC -> P-FET reverse protection -> switch -> VBAT
VBAT -> DRV8874 x2 (VM)          motor power, own copper pour
VBAT -> TPS563201 -> 5V          BlackPill 5V pin (its LDO makes the MCU/IMU 3V3)
5V   -> AP2112K-3.3 -> 3V3_RF    ESP32-C3 only
VBAT -> 100k/22k -> PA4          battery monitor (11 V reads as 1.98 V)
```

## Pin map

Same table as `firmware/app/inc/board.h`:

| Signal | MCU pin | Peripheral |
|---|---|---|
| IMU SCL / SDA | PB8 / PB9 | I2C1 at 400 kHz, RX by DMA1 Stream0 |
| IMU INT | PB5 | EXTI5 rising edge |
| Left / right motor EN (PWM) | PA6 / PA7 | TIM3 CH1 / CH2 at 20 kHz |
| Left / right motor PH (direction) | PB0 / PB1 | GPIO |
| Driver nSLEEP (both) | PB12 | GPIO, low = outputs off |
| Driver nFAULT (wired-OR) | PB13 | GPIO in, 10k pull-up |
| Left encoder A / B | PA15 / PB3 | TIM2 encoder mode |
| Right encoder A / B | PB6 / PB7 | TIM4 encoder mode |
| ESP32-C3 link | PA9 TX / PA10 RX | USART1 at 460800, DMA both ways |
| Debug console | PA2 TX / PA3 RX | USART2 at 115200 |
| Battery sense | PA4 | ADC1 IN4 |
| Motor current sense | PA0 / PA1 | ADC1 IN0 / IN1 (routed for rev 2 firmware) |
| SWD | PA13 / PA14 / NRST | ST-Link |

ESP32-C3 side: GPIO4 = RX (from PA9), GPIO5 = TX (to PA10). GPIO20/21 are not
used for the link because the ROM bootloader prints on them at reset.

DRV8874 strapping: PMODE tied low (PH/EN mode), IMODE set for cycle-by-cycle
current limit, IPROPI through 1.5 k to ground (about 2.4 V at 3.5 A, 450 uA per A).

## Layout rules

1. **Split the board into a power half and a logic half.** Battery connector,
   PTC, FET, drivers, bulk cap and motor connectors all sit along one edge.
   MCU, IMU and ESP32 sit on the other. Motor current never has a reason to
   cross the logic side.
2. **One ground, joined at one place.** Solid ground pour on the bottom layer.
   The motor return currents run in a wide top-layer pour back to the battery
   minus terminal and meet the logic ground only at that terminal (star point).
   No split planes with signals crossing a gap.
3. **Keep the motor current loop small.** VM bulk cap (220 uF) sits between
   the two drivers. Each driver gets its own 10 uF + 100 nF right at the VM pin,
   with vias straight to ground. The thermal pad gets a via array for heat.
4. **Decoupling at every IC.** 100 nF as close as possible to every supply pin,
   plus a bulk cap per rail. The IMU header gets its own 100 nF next to it.
5. **IMU placement.** On the logic side, as close to the wheel axle line as the
   layout allows (less linear acceleration from body rotation reaches the
   accelerometer), away from the inductor and the drivers, mounted with the
   chip X axis pointing forward.
6. **Buck converter.** Input cap, switch node and inductor in a tight loop;
   switch node copper kept small and away from I2C and the IMU.
7. **I2C.** Short traces, 2.2 k pull-ups at the MCU end, no routing under the
   buck inductor.
8. **ESP32-C3 antenna.** Antenna hangs off the board edge with the keep-out
   from the datasheet, no copper or ground under it.
9. **Trace widths.** 1 oz copper. Motor and VBAT traces at least 1.5 mm
   (about 4 A with a 20 C rise). Signals 0.25 mm.

## Bring-up checklist

1. No battery, power from USB on the BlackPill. Check 3V3 rails, flash firmware,
   check the debug console prints and the IMU answers WHO_AM_I.
2. Bench supply at 11 V with a 1 A current limit, wheels off the ground.
   Check 5 V rail, ESP32 advertises, dashboard connects.
3. Spin each motor from the dashboard (drive keys while held), confirm the
   direction and encoder sign. Flip `MOTOR_*_INVERT` / `ENC_*_INVERT` in
   `firmware/app/inc/motor.h` if needed.
4. Tilt the robot by hand and confirm the angle sign: leaning forward must read
   positive. Fix the axis macros in `imu_mpu6050.c` if not.
5. Battery, robot held upright, arm, then let go gently.

## What I would change in rev 2

- **IMU on the board.** The MPU-6050 is end of life. Put a TDK ICM-42688-P on the
  PCB itself, near the axle line, read over SPI. Lower noise, a hardware
  anti-alias filter and no flying module.
- **Current loop.** IPROPI is already routed to the ADC. Next step is a third,
  innermost torque (current) loop at 5-10 kHz.
- **MCU on the board.** Drop the BlackPill socket, put the STM32 down directly
  with USB-C for flashing. Smaller and no connector wobble.
- **4-layer stackup.** Signal / ground / power / signal makes the grounding
  story much simpler.
- **Per-cell battery monitoring** and a proper low-voltage cutoff in hardware.
- **Test points** on every rail and on I2C, UART and PWM.
