# thumper-nucleo
Wild Thumper motor driver project.

STM32F446RE Nucleo Board + VNH5019 Motor Driver + Nucleo Motion MEMS

## [STM32F446RE Nucleo Board](https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html)
Main brain of the system designed to:
*  Use micro-ROS with FreeRTOS to build ROS compliant node
*  FreeRTOS will also be used to generate main control flow of node: sense -> plan -> act
*  Run PID loop control of PWM output to Motor Driver
*  Sense current through the VNH5019 Motor Driver for PID loop control and protection of the motors
*  Read the encoder outputs of the motors & generate velocity measurements
*  Read the motion MEMS to get accelerometer, gryroscope, magnometer, and temperature measurements

## [VNH5019 Motor Driver](https://www.pololu.com/product/2507)
H bridge control board designed to handle the stall current for the bank of three motors per H bridge.

Arduino style headers are the main way to connect to the STM32 Nucleo board.

![Arduino header pinout](https://a.pololu-files.com/picture/0J3753.280.jpg?5877a2053bf73d6dbb43d10e7d268ce5)

Current sense voltage output proportional to motor current (approx. 140 mV/A; only active while H-bridge is driving)

## [Nucleo Motion MEMS](https://www.st.com/en/ecosystems/x-nucleo-iks01a3.html)
IMU Sensors connected over I²C to STM32 Nucleo through Arduino and Nucleo headers.

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

There is a built in suspension to help keep the wheels pushed to the ground when driving over rough terain to maintain traction with the wheels.

An encoder is attached to the middle wheel of each side to get an idea of how fast each group of motors is running. The middle wheel pair are selected because the Wild Thumper Chassis always pushes both of those down using the suspension versus the other pairs of wheels which will react to one another, i.e. if one wheel is pushed up then it will force the opposing wheel down. This design ensures that the motos with the encoders are always trying to make contact with the ground.

Initial control will be velocity based meaning that the Wild Thumper will recieve a target velocity for each bank of motors that it will try to maintain through the PID loop. Position control will be the next step and will be relative positioning using both encoders to determine the distance traveled and relative position from the starting point.

## [Motors + Encoders](https://www.pololu.com/product/1575)
The motors are intended for a maximum nominal operating voltage of 7.2 V (2 V minimum), and each has a stall current of 6.6 A and a no-load current of 420 mA at 7.2 V.

Since the motors will briefly draw the full stall current when abruptly starting from rest (and nearly twice the stall current when abruptly going from full speed in one 
direction to full speed in the other), we recommend a motor driver capable of supplying the 20 A combined per-channel stall current of these motors at 7.2 V.

Each version is also optionally available with an integrated 48 CPR quadrature encoder.

Exact gear ratio: (22/12)×(20/12)×(22/10)×(22/10)×(22/10)×(23/10)≈74.83:1

The encoders will first be used to calculate the velocity of the motors to be used for the PID control loop as feedback of the target velocity. Later implementation will then use the encoder counts for relative positioning.

## [micro-ROS](https://micro.ros.org/)
Micro-ROS offers seven key features:
*  Microcontroller-optimized client API supporting all major ROS concepts
*  Seamless integration with ROS 2
*  Extremely resource-constrained but flexible middleware
*  Multi-ROS support with generic build system
*  Permissive license
*  Vibrant community and ecosystem
*  Long-term maintainability and interoperability

Supported Hardware listed on their site:
*  ST NUCLEO-F446ZE
*  ST NUCLEO-F746ZG
*  ST NUCLEO-H743ZI

The ST NUCLEO-F446RE is in the same family as the ST NUCLEO-F446ZE so there is a good chance that it will work the same but further investigation is needed here.

micro-ROS utlizies an RTOS and for this project FreeRTOS will be used.

Vulcanexus is an all-in-one ROS 2 tool set for easy and customized robotics development.

[STM32CubeMX micro-ROS integration](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils)
