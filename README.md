# Enhanced Platooning for Energy Efficiency and Safety

## Overview
This project focuses on developing an advanced platooning system that uses machine learning to enhance energy efficiency and road safety. It combines real-time communication, a high-fidelity simulation environment (CARLA), and sensor-based decision-making to optimize vehicle coordination.

For a detailed problem description, objectives, and requirements, please see the [PROBLEM_STATEMENT.md](PROBLEM_STATEMENT.md).

### Key Features
- **Energy Efficiency:** Reduces fuel consumption and CO₂ emissions by minimizing aerodynamic drag.
- **Enhanced Safety:** Prevents collisions through real-time communication and predictive hazard detection.
- **Simulation and Hardware:** Validated in the CARLA simulator and implemented on ESP32-based hardware.
- **Multiple Control Strategies:** Implements both a deterministic (rule-based) and an ML-based control model.

---

## Video Demo
[![Watch the video](https://img.youtube.com/vi/emA3QzaC8Kk/0.jpg)](https://www.youtube.com/watch?v=emA3QzaC8Kk)

Click on the image above to watch the demo of our **Enhanced Platooning System** in action!

---

## File Structure
```
.
├── images
│   ├── motor_connection.jpg
│   └── sensor_connection.jpg
├── model
│   ├── ML_model_training.ipynb
│   └── platooning_model.tflite
├── src
│   ├── arduino
│   │   ├── master_code.ino
│   │   └── slave_code.ino
│   └── python
│       ├── ml_code.py
│       └── simple_follower.py
├── PROBLEM_STATEMENT.md
└── README.md
```

## File Descriptions

### `src/`
This directory contains the source code for the project.

- **`arduino/`**: Contains the Arduino sketches for the ESP32 microcontrollers.
    - **`master_code.ino`**: The code for the lead vehicle (master). It reads sensor data and sends it to the follower vehicle.
    - **`slave_code.ino`**: The code for the follower vehicle (slave). It receives data from the master and controls the vehicle's movement.

- **`python/`**: Contains the Python scripts for the CARLA simulation.
    - **`simple_follower.py`**: A simple implementation of a follower vehicle using a deterministic model.
    - **`ml_code.py`**: The code for running the machine learning model in the CARLA simulation.

### `model/`
This directory contains the machine learning model and the training notebook.

- **`ML_model_training.ipynb`**: A Jupyter notebook that trains the platooning model.
- **`platooning_model.tflite`**: The trained TensorFlow Lite model for platooning.

### `images/`
This directory contains images related to the hardware setup.

- **`motor_connection.jpg`**: A diagram showing the motor connections.
- **`sensor_connection.jpg`**: A diagram showing the sensor connections.

---

## System Architecture

### 1. Hardware Implementation (ESP32)

The physical implementation consists of two vehicles: a lead vehicle (master) and a follower vehicle (slave).

#### Lead Vehicle (Master)
- **Microcontroller:** ESP32
- **Control:** Manually operated via a Bluetooth serial app.
- **Communication:** Sets up a Wi-Fi access point and broadcasts its sensor data via an HTTP server.
- **Sensors:** An MPU-6050 IMU measures acceleration and angular velocity.
- **Code:** `src/arduino/master_code.ino`

#### Follower Vehicle (Slave)
- **Microcontroller:** ESP32
- **Control Logic:** A proportional controller adjusts speed and steering to follow the leader.
- **Communication:** Connects to the master's Wi-Fi network to receive IMU data.
- **Sensors:**
    - An MPU-6050 IMU measures its own orientation.
    - An ultrasonic sensor measures the distance to the lead vehicle.
- **Code:** `src/arduino/slave_code.ino`

### 2. Simulation (CARLA)

The platooning algorithms are tested and validated in the CARLA simulator.

#### Deterministic Follower
- **Description:** A simple rule-based follower using a Proportional-Derivative (PD) controller for longitudinal control (throttle/brake) and a Proportional (P) controller for lateral control (steering).
- **Goal:** Maintain a fixed distance from the leader while matching its trajectory.
- **Script:** `src/python/simple_follower.py`

#### Machine Learning Follower
- **Description:** Uses a trained TensorFlow Lite model to predict vehicle controls.
- **Model:** The model (`model/platooning_model.tflite`) takes sensor data as input and outputs throttle, brake, and steer values.
- **Inputs:** Lead/follower speed, lead/follower yaw, current distance, target distance, and speed error.
- **Script:** `src/python/ml_code.py`
- **Training:** The model is trained in the `model/ML_model_training.ipynb` notebook.

---

## Motivation
### 1. Increasing Demand for Energy Efficiency
- Reduces **fuel consumption** and **CO₂ emissions**.
- Helps **fleet operators** lower operational costs .

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
  pip install carla pygame numpy jupyter opencv-python tflite-runtime
  ```

### 3. Running the Platooning Simulation
- **Deterministic Model:**
  ```bash
  python src/python/simple_follower.py
  ```
- **ML Model Training:**
  ```bash
  jupyter notebook model/ML_model_training.ipynb
  ```
- **ML Model Execution in CARLA:**
  ```bash
  python src/python/ml_code.py
  ```

### 4. Setting Up ESP32 Communication
- Install **ESP32 libraries** in Arduino IDE (including `Adafruit_MPU6050`, `ESPAsyncWebServer`, and `ArduinoJson`).
- Upload `src/arduino/master_code.ino` to **Lead Vehicle ESP32**.
- Upload `src/arduino/slave_code.ino` to **Follower Vehicle ESP32**.

---

## Conclusion
This project successfully demonstrates **energy-efficient and safe platooning** using a combination of **simulation, sensor-based decision-making, and real-time communication**. Future work includes **real-time ML deployment**, **dynamic platooning behavior**, and **advanced sensor integration**. 