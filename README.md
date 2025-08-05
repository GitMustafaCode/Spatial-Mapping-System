# Spatial-Mapping-System

A cost-effective and efficient embedded spatial mapping system using a VL53L1X Time-of-Flight (ToF) sensor, stepper motor, and microcontroller to generate 3D visualizations of indoor environments.

## **Read this document for a detailed description**
https://github.com/AbaanKhan970/Spatial-Mapping-System/blob/ff64e0b8df3bd05317c80784a1e2e8878c886e59/ProjectDescription.pdf

## **Demonstration**
Setup

![image](https://github.com/user-attachments/assets/3c30caab-2e3f-4614-a1d4-3c1e4d0bfb10)

3D visualisation of hallway scan

![image](https://github.com/user-attachments/assets/55ac5f9a-5b96-4725-a40b-432eb3750625)  ![image](https://github.com/user-attachments/assets/468500ee-c6bd-4840-adb1-258a32191239)

## **Circuit Schematic**
![image](https://github.com/user-attachments/assets/7343a142-c9ad-4d87-a23f-7d1beec329ad)

## **Getting Started**
### **Prerequisites**
- Texas Instruments MSP432E401Y Microcontroller
- VL53L1X Time-of-Flight Sensor
- 28BYJ-48 Stepper Motor with ULN2003 Driver Board
- A Windows/Linux system with Python 3.8+
- Required libraries: `pyserial`, `open3d`, `numpy`

### **Setup**
#### **Hardware Setup**
1. Connect the **VL53L1X ToF sensor** to the microcontroller via I2C (SCL to PB2, SDA to PB3).
2. Connect the **Stepper motor** to the microcontroller through the ULN2003 driver board.
3. Configure the **onboard push button (PJ1)** for initiating scans.
4. Verify **LED status indicators (PN0, PN1)** are correctly wired.

#### **Software Setup**
1. Clone this repository:
   ```bash
   git clone https://github.com/YourUsername/SpatialMapping.git
   cd SpatialMapping
   ```
2. Open **Keil uVision** and load the provided project files.
3. Compile and upload the firmware to the **MSP432E401Y** microcontroller.
4. Install required Python libraries:
   ```bash
   pip install pyserial open3d numpy
   ```
5. Modify the Python script (`2DX_pythonCode_khana454.py`) to match your system's **UART port**.
6. Run the Python script to start capturing spatial data:
   ```bash
   python 2DX_pythonCode_khana454.py
   ```

## **Project Overview**
This project creates a **3D spatial map** by rotating a **ToF sensor** mounted on a **stepper motor**. The system collects distance data at every **11.25° step** and transmits it to a computer for visualization.

### **How It Works**
1. Press **PJ1** to start scanning.
2. The **stepper motor** rotates **360°**, taking measurements every **11.25°**.
3. Data is sent to the **MSP432E401Y** via **I2C**.
4. The microcontroller transmits the data to the **PC via UART**.
5. The Python script processes data into **3D models** using Open3D.

## **Code Structure**
### **Embedded System Components (C Code - Keil uVision)**
#### **VL53L1X_api.c / VL53L1X_api.h**
- Interfaces with the **ToF sensor** using **I2C**.
- Retrieves distance measurements in **millimeters**.
- Sends data to the **microcontroller memory**.

#### **uart.c / uart.h**
- Manages **UART communication** between microcontroller and PC.
- Sends measured distances to the **Python script**.

#### **SysTick.c / SysTick.h**
- Configures **timing delays** for sensor sampling and stepper motor control.

#### **PLL.c / PLL.h**
- Configures the **system clock speed** based on student parameters.

#### **onboardLEDs.c / onboardLEDs.h**
- Controls **status LEDs** for data collection feedback.

### **Python Components (Data Processing & Visualization)**
#### **2DX_pythonCode_khana454.py**
- Reads data from the **MSP432E401Y via UART**.
- Converts raw **distance readings** into **(x, y, z) coordinates**.
- Generates a **3D point cloud** using **Open3D**.

## **Features**
✔️ **Time-of-Flight Sensor** for precise distance measurement  
✔️ **Stepper Motor Integration** for automated scanning  
✔️ **Real-time 3D Visualization** using Open3D  
✔️ **UART Communication** between microcontroller and PC  
✔️ **Modular Code Design** for easy expansion  
