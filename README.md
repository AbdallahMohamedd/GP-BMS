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
- **NXP FRDM-MC33771B-TPLEVB** — high-precision slave battery monitoring IC
- **KL25Z Microcontroller** — master controller
- Custom-designed **sensing and protection circuits**
- **LCD Interface** for status monitoring
- Relays, connectors, and peripheral safety hardware

### ✅ Software Architecture
The software is layered into:
- **Application Layer**: High-level logic (balancing, thermal, diagnostics)
- **Service Layer**: Data handling, watchdogs, and status management
- **HAL (Hardware Abstraction Layer)**: Interfaces for slave control, LCD
- **MCAL (Microcontroller Abstraction Layer)**: Communication protocols like I2C, SPI, UART, PWM, GPIO

### ✅ MATLAB/Simulink Integration
We’ve developed detailed battery and control models using Simulink:
- **Equivalent Circuit Model (ECM)** for Li-ion cells
- Parametric simulations for SOC and SOH estimation
- Model-based validation of control strategies

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

### Sponsored by: [**Vehiclevo**](https://www.vehiclevo.com)  
We are incredibly proud to have Vehiclevo as our industry sponsor. Their support enabled us to access state-of-the-art components, tools, and expert mentorship.

### Mentors from Vehiclevo:
- Eng. **Mohamed Maklad**
- Eng. **Omar Gamal Eldin**
- Eng. **Fady Bassyoni**

---

## 🏫 Institution

**Ain Shams University**  
Faculty of Engineering  
Department of Electrical Power and Machines  
Graduation Year: **2025**

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
   git clone https://github.com/your-username/your-bms-repo.git
   ```

2. Open the Simulink model under `/Matlab_Model` to simulate battery behavior and SOC estimation.

3. For firmware development, use the source files under `/Software_Firmware` with **MCUXpresso** or **Keil** toolchain.

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
