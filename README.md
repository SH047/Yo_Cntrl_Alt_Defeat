# Obstacle-Avoiding Robot

A rear-wheel-drive autonomous robot with front-wheel steering, three ultrasonic distance sensors, and Raspberry Pi computer vision. The Raspberry Pi uses a camera and OpenCV to detect obstacles/colours and sends high-level commands such as `dodgeRight()` and `dodgeLeft()` to an ESP32 over serial/UART. The ESP32 handles ultrasonic sensing, steering, and motor control.

## 1. System Architecture

```text
                 ┌──────────────────────┐
                 │     RASPBERRY PI     │
                 │                      │
                 │ Camera + OpenCV      │
                 │ Obstacle/colour      │
                 │ decision making      │
                 └──────────┬───────────┘
                            │
                       Serial / UART
                            │
                            ▼
                 ┌──────────────────────┐
                 │        ESP32         │
                 │                      │
                 │ Ultrasonic sensing   │
                 │ Servo steering       │
                 │ Motor control        │
                 └───────┬───────┬──────┘
                         │       │
              ┌──────────┘       └───────────┐
              ▼                              ▼
       ┌─────────────┐                ┌─────────────┐
       │ Servo motor │                │ Motor driver│
       │ Front steer │                │             │
       └─────────────┘                └──────┬──────┘
                                            │
                                            ▼
                                       Rear BO motor
                                       + rear wheels
```

The Raspberry Pi performs high-level visual processing. The ESP32 is responsible for real-time interaction with the physical hardware.

## 2. Main Hardware

- Raspberry Pi
- Raspberry Pi camera module
- ESP32 development board
- 3 × HC-SR04 ultrasonic sensors
- Servo motor for front-wheel steering
- BO motor
- Compatible single-channel motor driver with IN1, IN2 and ENA/PWM inputs
- 9 V battery/supply
- 9 V → 5 V buck converter
- Front steering mechanism and chassis
- Rear wheels connected to the BO motor
- Resistors for ultrasonic ECHO voltage dividers: 1 kΩ and 2 kΩ for each sensor
- Jumper wires and suitable power wiring

## 3. ESP32 Pin Connections

| Component | Connection | ESP32 pin |
|---|---|---|
| Front ultrasonic TRIG | TRIG | GPIO 13 |
| Front ultrasonic ECHO | ECHO | GPIO 34 |
| Left ultrasonic TRIG | TRIG | GPIO 14 |
| Left ultrasonic ECHO | ECHO | GPIO 35 |
| Right ultrasonic TRIG | TRIG | GPIO 25 |
| Right ultrasonic ECHO | ECHO | GPIO 32 |
| Motor driver IN1 | Motor control | GPIO 18 |
| Motor driver IN2 | Motor control | GPIO 19 |
| Motor driver ENA | PWM/speed | GPIO 15 |
| Servo signal | PWM | Use a free suitable GPIO |
| Common ground | GND | ESP32 GND |

GPIO 34 and GPIO 35 are input-only pins, which is appropriate for the ultrasonic ECHO signals.

## 4. Ultrasonic Sensor Wiring

Each HC-SR04 is connected as follows:

```text
HC-SR04 VCC  → 5 V
HC-SR04 GND  → GND
HC-SR04 TRIG → assigned ESP32 GPIO
HC-SR04 ECHO → voltage divider → assigned ESP32 GPIO
```

The HC-SR04 ECHO output can be approximately 5 V, while ESP32 GPIO is designed for 3.3 V logic. Do **not** connect a 5 V ECHO signal directly to an ESP32 input.

Use one voltage divider for every ECHO line:

```text
HC-SR04 ECHO
     │
    1 kΩ
     │
     ├────────────→ ESP32 ECHO GPIO
     │
    2 kΩ
     │
    GND
```

This produces approximately 3.3 V from a 5 V ECHO signal.

Make three identical dividers:

- Front ECHO → GPIO 34
- Left ECHO → GPIO 35
- Right ECHO → GPIO 32

The sensors should be physically positioned so that one faces forward and the other two face left and right.

```text
                    FRONT
                      ↑
                [FRONT SENSOR]

          [LEFT]                 [RIGHT]
          SENSOR                  SENSOR

                 ┌─────────┐
                 │  ROBOT  │
                 └────┬────┘
                      │
                   BO MOTOR
```

## 5. Power System

The 9 V supply powers the motor circuit and is also reduced to 5 V for the servo and ultrasonic sensors.

```text
                 9 V BATTERY
                ┌──────┴──────┐
                │             │
                ▼             ▼
          Motor driver    Buck converter
          motor supply       9 V → 5 V
                              │
                    ┌─────────┴─────────┐
                    ▼                   ▼
                 Servo VCC        Ultrasonic VCC
```

The ESP32 may be supplied through its appropriate 5 V/VIN input if the board and regulator specifications permit it. Check the exact ESP32 development board before connecting power.

All grounds must be common:

```text
Battery GND
   ├── Motor driver GND
   ├── Buck converter GND
   ├── ESP32 GND
   ├── Servo GND
   └── Ultrasonic GND
```

A common ground is essential because the ESP32 control signals need the same voltage reference as the devices receiving those signals.

## 6. Servo Wiring

The servo controls the front steering mechanism:

```text
Buck +5 V   → Servo VCC
Buck GND    → Servo GND
ESP32 GPIO  → Servo SIGNAL
```

Do not power the servo from an ESP32 GPIO. Steering against mechanical resistance can cause the servo to draw significant current, so the 5 V buck converter should supply the servo.

## 7. Motor Driver Wiring

The ESP32 controls the rear BO motor through the motor driver:

```text
ESP32 GPIO 18 ─────→ Motor driver IN1
ESP32 GPIO 19 ─────→ Motor driver IN2
ESP32 GPIO 15 ─────→ Motor driver ENA/PWM
ESP32 GND ──────────→ Motor driver GND

9 V battery ────────→ Motor driver motor-power input

Motor driver OUT1 ──→ BO motor
Motor driver OUT2 ──→ BO motor
```

The exact power and output terminals depend on the motor-driver module. Verify its pin labels and voltage/current ratings before connecting the motor.

Example ESP32 definitions:

```cpp
#define MOTOR_PIN_1 18
#define MOTOR_PIN_2 19
#define ENA 15
```

## 8. Raspberry Pi and Camera

Connect the camera module to the Raspberry Pi using the appropriate camera connector and configure the Raspberry Pi camera software for the installed operating system.

The software flow is:

```text
Camera
   ↓
Image capture
   ↓
OpenCV processing
   ↓
Obstacle/colour detection
   ↓
High-level decision
   ↓
"dodgeRight()" / "dodgeLeft()"
   ↓
UART/Serial
   ↓
ESP32
```

OpenCV can be used for image preprocessing, colour detection, contour/object detection, and other required vision operations. The exact OpenCV algorithm depends on the type of obstacle or colour that must be detected.

## 9. Raspberry Pi–ESP32 Communication

A serial/UART connection is used to transfer commands from the Raspberry Pi to the ESP32.

A simple command protocol can be used, for example:

```text
RIGHT
LEFT
FORWARD
STOP
```

The Raspberry Pi sends the command after processing the camera image. The ESP32 receives the command and calls the corresponding control routine.

If physical UART pins are used, connect TX of the transmitting device to RX of the receiving device and RX to TX, with a common ground. Confirm the voltage levels and UART configuration of the specific Raspberry Pi and ESP32 setup before wiring.

## 10. Obstacle-Avoidance Logic

The three ultrasonic sensors provide spatial information around the robot.

For example:

```text
Left   = 15 cm
Front  = 20 cm
Right  = 80 cm
```

The right side has substantially more free space, so the robot can select a right-hand avoidance manoeuvre.

Another example:

```text
Left   = 80 cm
Front  = 20 cm
Right  = 15 cm
```

The left side has more clearance, so the robot can select a left-hand manoeuvre.

If an obstacle is directly ahead:

```text
Left   = 60 cm
Front  = 15 cm
Right  = 55 cm
```

the obstacle is primarily in the forward path, and the left/right distances can be compared to select the clearer direction.

A single ultrasonic sensor would only provide information about the distance in one direction. Three sensors allow the ESP32 to compare available clearance on both sides and make a more useful steering decision.

## 11. Control Responsibilities

The Raspberry Pi and ESP32 have separate responsibilities.

**Raspberry Pi**
- Captures camera images
- Runs OpenCV
- Detects obstacles/colours
- Makes high-level vision decisions
- Sends avoidance commands

**ESP32**
- Reads the three ultrasonic sensors
- Determines local obstacle clearance
- Controls the motor driver
- Controls the steering servo
- Executes commands received from the Raspberry Pi

This separation keeps computationally intensive vision processing on the Raspberry Pi while keeping time-sensitive hardware control on the ESP32.

## 12. Why This Architecture Works

The robot combines camera-based vision with direct distance measurement. The camera provides visual information that ultrasonic sensors cannot provide, while ultrasonic sensors provide direct proximity information that does not depend on image interpretation.

Front-wheel steering separates steering from propulsion:

```text
Servo → front-wheel steering
BO motor → rear-wheel propulsion
```

This makes the physical control system straightforward: the ESP32 changes the servo angle to steer while the motor driver controls the rear BO motor.

Before powering the complete system, verify polarity, common ground, regulator output voltage, motor-driver ratings, servo current requirements, and the 3.3 V limitation of ESP32 GPIO inputs. Test the motor, servo, and each ultrasonic sensor separately before running the complete obstacle-avoidance program.
