# Design and Implementation of a Sophisticated Battery Management System (BMS)

![Image](https://github.com/user-attachments/assets/b7d20004-bb00-4500-b7d7-fc145cfc3571)

## 🚀 Project Overview

This project, **"Design and Implementation of a Sophisticated Battery Management System (BMS)"**, is the graduation project of eight senior Electrical Power Engineering students from **Ain Shams University**, class of **2025**. 

Under the esteemed mentorship of **Dr. Mohamed Mostafa** and in collaboration with **Vehiclevo**, our objective is to design, build, and validate a fully integrated BMS tailored for **electric vehicles (EVs)** — a key component for ensuring safe, efficient, and reliable battery operation.

The project not only focuses on hardware design and embedded software development but also includes simulation modeling in MATLAB/Simulink, with an emphasis on real-time control, cell balancing, fault detection, and energy optimization.

---

## 🧠 Objective

To develop an intelligent Battery Management System capable of:
- Real-time **voltage**, **current**, and **temperature** monitoring
- Accurate **State of Charge (SOC)** estimation
- **Dynamic cell balancing** to enhance battery longevity
- **Proactive fault detection** for improved safety
- Seamless **integration of MATLAB models** for control validation

---

## 🧩 System Components

### ✅ Hardware

Our hardware implementation consists of two major subsystems:

#### 🔋 Battery Management System (BMS)
- **NXP FRDMM33771BTPLEVB** — a high-precision slave battery monitoring IC for real-time voltage, current, and temperature measurements.
- **KL25Z Microcontroller** — functions as the master controller for handling data acquisition, diagnostics, and protection logic.
- Custom-designed **sensing and protection circuits** — for current measurement, voltage dividers, and overcurrent/overvoltage protection.
- **LCD Interface** — provides a real-time visual display of system parameters.
- **Relays**, **connectors**, and other peripheral hardware for integration and switching.

#### ⚡ Custom Battery Charger
- A dedicated **charging circuit** was designed and implemented on a custom printed circuit board (PCB).
- The system is powered by an external **42V DC power supply**.
- An integrated **Boost Converter** steps the voltage up to **60V** to meet the required charging specifications for the battery pack.

### ✅ Software Architecture

The software is modularly layered and divided into:

- **Application Layer**: Manages high-level functionalities like thermal management, diagnostics, and balancing logic.
- **Service Layer**: Handles data handling, system states, and watchdog mechanisms.
- **HAL (Hardware Abstraction Layer)**: Provides unified access to low-level drivers (ADC, GPIO, I2C, SPI, UART).
- **MCAL (Microcontroller Abstraction Layer)**: Direct interface with hardware peripherals.

> 🔁 The balancing algorithm component was developed in MATLAB/Simulink, and its corresponding subsystem was exported using **automatic code generation**, ensuring seamless integration with the embedded firmware.

### ✅ MATLAB/Simulink Integration

A high-fidelity simulation model of the battery system was created in Simulink:
- Developed using the **Equivalent Circuit Model (ECM)** for lithium-ion cells to mimic real-world electrical behavior.
- Parametric simulations enable accurate estimation of **State of Charge (SOC)** and **State of Health (SOH)**.
- The model is used to **validate control strategies** and system responses under various operating conditions, making it a valuable tool for testing without physical hardware.

---

## 🎯 Key Features and Benefits

- 📊 **Real-Time Monitoring** — Continuous sensing and control of cell parameters
- 🔋 **Accurate SOC Estimation** — For effective energy management
- 🛡️ **Fault Detection** — Early diagnosis to prevent failures
- ♻️ **Cell Balancing** — Maintains voltage uniformity across cells
- ❄️ **Thermal & Fan Management** — Ensures temperature safety
- 🧪 **Scalable & Modular Design** — Easily expandable for future EV pack sizes

---

## 👥 Team Members

- Abdulla Mohamed Ibrahim
- Amr Ahmed Hassan
- Yomna Mohamed Abdelfadeel
- Ganna Ayman Mounir
- Abdelrahman Mohamed Rafaat
- Ziad Yasser Mohamed
- Khaled Abdelmonem Abdelaziz
- Ahmed Mohamed Mohamed

---

## 🤝 Sponsorship & Mentorship

### Sponsored by: [**Vehiclevo**](https://www.linkedin.com/company/vehiclevo/posts/?feedView=all)  
We are incredibly proud to have Vehiclevo as our industry sponsor. Their support enabled us to access state-of-the-art components, tools, and expert mentorship.

### Mentors from Vehiclevo:
- Eng. [**Mohamed Maklad**](https://www.linkedin.com/in/mohamed-maklad-1a4350100/)
- Eng. [**Omar Gamal Eldin**](https://www.linkedin.com/in/omar-gamal-4780b0181/)
- Eng. [**Fady Bassyoni**](https://www.linkedin.com/in/fadybassiouni/)

---

## 🏫 Institution
Ain Shams University

Faculty of Engineering

Department of Electrical Power and Machines

Graduation Year: 2025

Project Supervisor: Dr. [**Mohamed Mostafa**](https://www.linkedin.com/in/mohamed-mostafa-bb56a619a/?utm_source=share&utm_campaign=share_via&utm_content=profile&utm_medium=ios_app)

---

## 🗂️ Repository Structure

```
/Hardware_Documents
  └── Schematics, PCB Layouts, BOM

/Software_Firmware
  └── Source code for KL25Z, drivers, middleware

/Matlab_Model
  └── Simulink files, ECM models, simulation data

/Images
  └── Hardware pictures, architecture diagram, LCD display outputs

README.md
LICENSE
```

---

## 🖼️ Project Visuals

> 📸 *Images of assembled PCBs, sensor boards, LCD interface, and test setup will be added here.*

---

## 📎 How to Use

1. Clone the repo  
   ```bash
   git clone https://github.com/AbdallahMohamedd/gp.git
   ```

2. Open the Simulink model under `/Matlab_Model` to simulate battery behavior and SOC estimation.

3. For firmware development, use the source files under `/Software_Firmware` with **MCUXpresso** toolchain.

---

## 📢 Final Notes

This project represents a culmination of our academic knowledge, engineering passion, and dedication to real-world innovation. We hope it contributes to safer, more efficient electric mobility solutions.

Feel free to reach out, collaborate, or fork our work to take it further 🚗⚡

---

## 📬 Contact

For inquiries or collaboration:
📧 [abdallahmohamed4197@gmail.com]  
📍 Cairo, Egypt  
🌐 [LinkedIn](https://www.linkedin.com/in/abdullah-mohamed2002/)
