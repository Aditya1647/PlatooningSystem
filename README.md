# Enhanced Platooning for Energy Efficiency and Safety 

## Overview
Platooning is an innovative approach to vehicle management that leverages cooperative automation to enhance **energy efficiency** and **safety**. This project focuses on developing an advanced platooning system using real-time communication, simulation, and sensor-based decision-making.

### Key Features
- **Need for Platooning:** Addresses critical challenges in energy consumption and road safety.
- **Simulation with CARLA:** Testing and validating platooning performance under varied traffic and environmental conditions.
- **Bluetooth-Controlled Lead Vehicle:** The front car is controlled via Bluetooth for baseline behavior coordination.
- **ESP-to-ESP Communication:** Rear vehicles communicate using ESP-NOW for efficient operation.
- **Real-Time Sensor Data Analysis:** IMU and Ultrasonic sensors ensure optimal decision-making and adaptability.

---

## Video Demo
[![Watch the video](https://img.youtube.com/vi/emA3QzaC8Kk/0.jpg)](https://www.youtube.com/watch?v=emA3QzaC8Kk)

Click on the image above to watch the demo of our **Enhanced Platooning System** in action!

---

## Motivation
### 1. Increasing Demand for Energy Efficiency
- Reduces **fuel consumption** and **CO₂ emissions**.
- Helps **fleet operators** lower operational costs.

### 2. Enhancing Road Safety
- **Real-Time Communication** prevents collisions.
- **Predictive Hazard Detection** with Ultrasonic and IMUs.
- **Minimized Human Dependency** improves reliability.

### 3. Technological Advancements
- **Sensor Fusion** for accurate environmental perception.
- **Machine Learning & AI** for real-time dynamic platooning.

---

## System Model and Workflow
### Lead Vehicle (Master)
#### Hardware Components
- **Microcontroller:** ESP32
- **Sensors:** IMU, Ultrasonic sensor
- **Actuation:** 4-motor drivetrain with motor controller
- **Communication:**
  - Bluetooth (remote control)
  - ESP-NOW (vehicle-to-vehicle communication)

#### Control Mechanism
1. **Remote Control**: Controlled via an Android Bluetooth app.
2. **Sensor Data Transmission**: Sends acceleration and orientation data to the follower.

### Follower Vehicle (Slave)
#### Control Logic
1. **Data Reception and Processing:** Receives lead vehicle data.
2. **Distance and Obstacle Management:** Uses an ultrasonic sensor.
3. **Throttle and Steering Computation:** 
   - **Deterministic Cruise Control** (Rule-based).
   - **Machine Learning Model** (Not implemented on hardware due to ESP32 limitations).

### Simulation Environment
- **CARLA Simulator** for testing and validating platooning algorithms.
- **Algorithms Tested:**
  1. Cooperative Adaptive Cruise Control (CACC)
  2. Linear Quadratic (LQ) Control
  3. Machine Learning-based Control Strategies
- **Communication Protocols:**
  - **Bluetooth:** Remote control for lead vehicle.
  - **ESP-NOW:** Low-latency inter-vehicle communication.

---

## Results
- **15% Reduction in Fuel Consumption** due to optimized inter-vehicle spacing.
- **Improved Safety** via real-time sensor integration and predictive hazard detection.
- **Cost-Efficient Solution** using ESP32 modules.

### Drag Coefficient Analysis
- Close spacing (1-5m) significantly reduces drag.
- Beyond **15m spacing**, drag reduction plateaus.
- Trucks benefit the most due to larger aerodynamic advantages.

### Simulation Insights
- **Safe Spacing Increases with Speed:** 4.52m at 20 km/h to 8.41m at 50 km/h.
- **Trailing Vehicles Benefit More** from drag reduction.
- **Platoon Efficiency Stabilizes** at 15-20m but remains better than non-platooning.

---

## Challenges and Solutions
### 1. Obtaining x,y,z Coordinates and Yaw
- **Issue:** Difficulty in obtaining accurate positional data.
- **Solution:** Used IMU sensor differentiation and fine-tuned sensor alignment.

### 2. Hardware Limitations Preventing ML Deployment
- **Issue:** ESP32 lacks real-time ML computation capability.
- **Solution:** Future work involves **lightweight ML models** using **TensorFlow Lite Micro**.

### 3. High Computational Overhead from CARLA Simulation
- **Issue:** Running the simulator and ML model simultaneously caused performance issues.
- **Solution:** Pre-training using a **deterministic model** before full simulation.

---

## Future Improvements
1. **Deploy ML Model on ESP32**
   - Use **quantization and pruning** to reduce model size.
   - Explore **TensorFlow Lite Micro** for real-time ML deployment.
2. **Dynamic Platooning Behavior**
   - Adjust platoon configuration based on **real-time road conditions**.
   - Expand to **multi-vehicle interactions**.
3. **Advanced Sensors Integration**
   - Incorporate **LiDAR & stereo cameras** for enhanced perception.
   - Implement **V2X communication** for infrastructure interaction.
4. **Enhanced Safety Features**
   - Use **deep learning for predictive collision avoidance**.
   - Develop robust **fail-safe mechanisms**.

---

## How to Set Up the Project
### 1. Install CARLA Simulator
1. Download from [CARLA Downloads](https://tiny.carla.org/carla-0-9-15-windows).
2. Extract files and run `CarlaUE4.exe` (Windows) or `./CarlaUE4.sh` (Ubuntu).

### 2. Set Up Python Environment
- Install Anaconda and create a virtual environment:
  ```bash
  conda create -n carla-sim python=3.7.16
  conda activate carla-sim
  ```
- Install dependencies:
  ```bash
  pip install carla pygame numpy jupyter opencv-python
  ```

### 3. Running the Platooning Simulation
- **Deterministic Model:**
  ```bash
  python simple_follower.py
  ```
- **ML Model Training:**
  ```bash
  jupyter notebook ML_model_training.ipynb
  ```
- **ML Model Execution in CARLA:**
  ```bash
  python ml_code.py
  ```

### 4. Setting Up ESP32 Communication
- Install **ESP32 libraries** in Arduino IDE.
- Upload `master_code.ino` to **Lead Vehicle ESP32**.
- Upload `slave_code.ino` to **Follower Vehicle ESP32**.

---

## Conclusion
This project successfully demonstrates **energy-efficient and safe platooning** using a combination of **simulation, sensor-based decision-making, and real-time communication**. Future work includes **real-time ML deployment**, **dynamic platooning behavior**, and **advanced sensor integration**.
