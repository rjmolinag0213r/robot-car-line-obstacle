# 🚗 Smart Arduino Robot Car: Line Tracking + Obstacle Avoidance

This project features an **autonomous Arduino-powered robot car** capable of both **line tracking** and **obstacle avoidance** using infrared sensors, ultrasonic distance measurement, and a servo-mounted scanner.

## 🧠 Features

- 🔄 **Dual-Mode Navigation**:
  - **Line Tracking**: Follows a black line using 5 IR sensors.
  - **Obstacle Avoidance**: Detects and avoids obstacles using an ultrasonic sensor mounted on a servo for directional scanning.

- ⚙️ **Motor Control**:
  - Smooth movement using an L298N motor driver.
  - Speed and directional adjustments via PWM and GPIO pins.

- 👁️ **Sensor Integration**:
  - **5-Channel IR Sensors** for precise line detection.
  - **Ultrasonic Sensor (HC-SR04)** for obstacle distance measurement.
  - **Servo Motor** to rotate the ultrasonic sensor and scan for free paths.

## 🔧 Hardware Requirements

- Arduino Uno or compatible board  
- L298N Motor Driver  
- 2 DC Motors + Wheels  
- 5 IR Line Sensors (Analog)  
- HC-SR04 Ultrasonic Sensor  
- Servo Motor (SG90 or similar)  
- Chassis, Battery Pack, and Cables  

## 📁 File Structure

- `final_version_track_obstacle.ino` – Main Arduino sketch for robot logic including motor control, sensor reading, line following, and obstacle avoidance.

## ⚡ How It Works

1. The car begins in **line tracking mode**, reading 5 analog IR sensors to stay on course.
2. When an obstacle is detected in front:
   - The car stops and the **servo scans** left and right.
   - It evaluates which direction is free and steers around the obstacle.
3. After navigating around the obstacle, it resumes **line tracking**.

## ▶️ Getting Started

1. Wire up your hardware according to the defined pin assignments in the code.
2. Upload `final_version_track_obstacle.ino` to your Arduino using the Arduino IDE.
3. Place the car on a track with a black line and let it navigate!

## 📌 Pin Configuration (in code)

| Component         | Pin         |
|------------------|-------------|
| IR Sensors       | A0–A4       |
| Left Motor       | D7, D8, D6  |
| Right Motor      | D11, D12, D3|
| Ultrasonic       | Trig: D10, Echo: D2 |
| Servo            | D9 (assumed, defined in `Servo.attach`) |

## 🧠 Customization Ideas

- Add Bluetooth/WiFi for remote control.
- Integrate PID control for better line tracking.
- Include OLED/LCD screen for status display.

## 🛠️ License

This project is open-source and available under the [MIT License](LICENSE).
