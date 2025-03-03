# Efficient Platooning with CARLA Simulator

Group - 39
Yugam Bhatt
Vaibhav Gupta
Aditya Gupta
Shashwat Saini
Nitin Kumar

Welcome to the Efficient Platooning project! This README will guide you through the process of setting up the CARLA simulator, the Python environment, and associated tools for the project.

---

## Prerequisites

- A computer with **Windows** or **Ubuntu** (Ubuntu is recommended for stability).
- A graphics card for smoother simulation performance (optional but helpful).

---

## Step 1: Installing CARLA Simulator

1. **Download the Simulator**:
   - Visit [CARLA Downloads](https://tiny.carla.org/carla-0-9-15-windows).
   - Download the latest release for your operating system.
   - Optionally, download additional maps.

2. **Extract the Files**:
   - Place the downloaded ZIP file in a directory (e.g., `C:\CARLA` for Windows).
   - Extract the ZIP file. After extraction, the main directory should contain the simulator files.

3. **Run the Simulator**:
   - Navigate to the extracted directory.
   - Launch the simulator by running the executable (`CarlaUE4.exe` on Windows).
   - Use **WASD** keys to navigate in the simulator and hold the left mouse button to adjust the camera view.

---

## Step 2: Setting Up Python Environment

### 1. Install Anaconda

- Download and install [Anaconda](https://www.anaconda.com/).
- Follow installation instructions available on YouTube.

### 2. Create a Virtual Environment

1. Open **Anaconda Prompt**.
2. Navigate to the CARLA Python API folder:
   ```bash
   cd C:\CARLA\PythonAPI
   cd examples
Create a virtual environment with Python 3.7.16:

bash

conda create -n carla-sim python=3.7.16
Confirm the installation if prompted.
Activate the environment:

bash

conda activate carla-sim
3. Install Required Libraries
Install the following libraries one by one:

CARLA:
bash

pip install carla
Pygame:
bash

pip install pygame
NumPy:
bash

pip install numpy
Jupyter:
bash

pip install jupyter
OpenCV:
bash

pip install opencv-python
4. Test the Environment
Start the CARLA simulator.
Navigate to the examples folder:
bash

cd C:\CARLA\PythonAPI\examples
Run a sample script:
bash

python generate_traffic.py
Use Ctrl+C to stop the script and clean up the simulation.
Step 3: Setting Up and Running the Deterministic Model
Place the simple_follower.py script in the C:\CARLA\PythonAPI\examples folder.
Start the simulator and run the script:
bash

python simple_follower.py
Step 4: Training the ML Model
Open the ML_model_training.ipynb notebook:
bash

jupyter notebook ML_model_training.ipynb
Train the machine learning model and save it as:

platooning_model.tflite
Step 5: Running the ML Model in CARLA
Place the ml_code.py script in the C:\CARLA\PythonAPI\examples folder.
Start the simulator and run the script:
bash

python ml_code.py
Step 6: Setting Up ESP32 Master and Slave Devices
1. Install Required Libraries
Ensure the following libraries and settings are configured in Arduino IDE:

ESP32 Module for Arduino IDE
Async HTTP Server
Async Client
Partition size set to Huge App
2. Upload the Code
Open master_code.ino and slave_code.ino in Arduino IDE.
Connect ESP32 devices via USB.
Upload master_code.ino to the Master ESP32 and slave_code.ino to the Slave ESP32.