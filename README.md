# thumper-nucleo
Wild Thumper motor driver project.

STM32F446RE Nucleo Board + VNH5019 Motor Driver + Nucleo Motion MEMS

## [STM32F446RE Nucleo Board](https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html)
Main brain of the system designed to:
*  Run PID loop control of PWM output to Motor Driver
*  Read the encoder outputs of the motors & generate velocity measurements
*  Read the motion MEMS to get accelerometer, gryroscope, magnometer, and temperature measurements

## [VNH5019 Motor Driver](https://www.pololu.com/product/2507)
H bridge control board designed to handle the stall current of the motors.

Arduino style headers are the main way to connect to the STM32 Nucleo board.

![Arduino header pinout](https://a.pololu-files.com/picture/0J3753.280.jpg?5877a2053bf73d6dbb43d10e7d268ce5)

Current sense voltage output proportional to motor current (approx. 140 mV/A; only active while H-bridge is driving)

## [Nucleo Motion MEMS](https://www.st.com/en/ecosystems/x-nucleo-iks01a3.html)
Sensors connected over I²C to STM32 Nucleo through Arduino and Nucleo headers.

Sensor Information:
*  LSM6DSO: MEMS 3D accelerometer (±2/±4/±8/±16 g) + 3D gyroscope (±125/±250/±500/±1000/±2000 dps).
*  LIS2MDL: MEMS 3D magnetometer (±50 gauss).
*  LIS2DW12: MEMS 3D accelerometer (±2/±4/±8/±16 g).
*  LPS22HH: MEMS pressure sensor, 260-1260 hPa absolute digital output barometer.
*  HTS221: capacitive digital relative humidity and temperature.
*  STTS751: Temperature sensor (–40 °C to +125 °C).
*  DIL 24-pin socket available for additional MEMS adapters and other sensors.
*  I²C sensor hub features on LSM6DSO available.

## [Wild Thumper Chassis](https://www.pololu.com/product/1563)
The Wild Thumper Chassis is a differential-drive chassis so turning is accomplished by driving the motors of each side at different speeds.

Chassis is designed to be operated with 3 motors + wheels of one side connected to singular output of motor driver.

There is a built in suspension to help keep the wheels pushed to the ground when driving over rough terain.

An encoder is attached to the middle wheel of each side to get an idea of how fast each group of motors is running.

## [Motors + Encoders](https://www.pololu.com/product/1575)
The motors are intended for a maximum nominal operating voltage of 7.2 V (2 V minimum), and each has a stall current of 6.6 A and a no-load current of 420 mA at 7.2 V.

Since the motors will briefly draw the full stall current when abruptly starting from rest (and nearly twice the stall current when abruptly going from full speed in one 
direction to full speed in the other), we recommend a motor driver capable of supplying the 20 A combined per-channel stall current of these motors at 7.2 V.

Each version is also optionally available with an integrated 48 CPR quadrature encoder.

Exact gear ratio: (22/12)×(20/12)×(22/10)×(22/10)×(22/10)×(23/10)≈74.83:1

## [micro-ROS](https://micro.ros.org/)
