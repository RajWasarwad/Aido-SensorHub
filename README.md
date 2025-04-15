# Aido Robot: Sensor Hub and Data Fusion (Simulated)

> ⚠️ **Disclaimer:** This is a simulated project developed for educational and demonstration purposes. It is **not** an actual implementation of the Aido Robot hardware but is intended to showcase the core logic, design architecture, and data fusion principles used in a sensor hub system.

## 📌 Project Overview

This project simulates the working of a **Sensor Hub and Data Fusion System** for the Aido Robot — a multi-sensor, intelligent companion robot. It models how various environmental and proximity sensors can be integrated with intelligent data polling, logging, and air quality assessment.

## 🔧 Technologies Used

- **STM32 Blue Pill (STM32F103C6)** microcontroller
- **Proteus** for simulation
- **STM32CubeIDE** for embedded development
- **C (Bare-metal)** programming
- External `.c/.h` module structure

## 🧠 Key Features

- ✅ **Adaptive Sensor Polling** based on environmental triggers and thresholds  
- ✅ **Sensor Power Management** using GPIO control to reduce energy consumption  
- ✅ **Structured Sensor Representation** via `sensor_t` struct for modular design  
- ✅ **Data Logging** with FIFO buffer mechanism for temporary storage  
- ✅ **Advanced AQI Calculation** using pollutant concentration data  
- ✅ **Sleep/Stand-by Modes** to improve overall system efficiency  
- ✅ **Integration with IR and Ultrasonic Sensors** for obstacle detection  
- ✅ **Modular Codebase** structured for scalability and hardware independence

## 📈 Sensors Simulated

| Sensor      | Type                                        | Functionality                        | Polling  | Logging  |
|-------------|---------------------------------------------|--------------------------------------|----------|----------|
| DHT22       | Temp & Humidity                             | Adaptive polling, comfort zone logic | ✅       | ✅       |
| MQ135       | Gas Sensor (CO, etc)                        | Adaptive on fluctuations             | ✅       | ✅       |
| BME688      | Env. Sensor (Gas, Pressure, Temp, Humidity) | Adaptive                             | ✅       | ✅       |
| Ultrasonic  | Distance                                    | Continuous object detection          | ✅       | ❌       |
| IR Sensor   | Obstacle detection                          | Continuous sensing                   | ✅       | ❌       |

## 📊 Air Quality Index (AQI)

AQI is calculated using individual pollutant indexes based on the following formula:
$$ I=\frac{(Ihigh​−Ilow​)}{(Chigh​−Clow​)}​×(C−Clow​)+Ilow​ $$
𝐼   : AQI value \
𝐶   : Pollutant concentration \
𝐶𝑙𝑜𝑤 , 𝐶ℎ𝑖𝑔ℎ  ​ : Concentration range \
𝐼𝑙𝑜𝑤 , 𝐼ℎ𝑖𝑔ℎ  ​ : AQI range for that concentration

- Based on 24-hour averages for PM2.5
- Based on 8-hour averages for CO, NO2, etc.
- Overall AQI determined by the highest individual AQI

## 📚 Project Structure

    |-- Core/
        |
        ├── src/ 
        │ ├── main.c 
        │ ├── sensor_logger.c
        │ ├── aqi.c
        | |-- DHT22.c
        |
        ├── inc/ 
        │ ├── main.h 
        │ ├── sensor_logger.h
        │ ├── aqi.h
        | |-- DHT22.h
        ├── README.md 
    
    |--- Debug/
        |
        |-- Aido SensorHub.hex 

---

## 📷 Circuit Diagram

> Proteus-based schematic showing STM32 Blue Pill and connected sensors.

![Circuit Diagram](/Imgs/Circuit.png)

---

## 🔁 System Flow Chart

> A high-level flow diagram of sensor polling, data logging, and AQI processing.

![System Flowchart](/Imgs/Flow.png)

---

## ⚙️ Pin Configuration

> GPIO pin mapping of each sensor and peripherals to the STM32 controller.

![Pin Configuration](/Imgs/Pins.png)

## 📂 DHT22 GitHub Repository

Used the external Library for Dht22 from "MrHause":  
👉 **[GitHub Link](https://github.com/MrHause/DHT22_STM32_HAL_LIBRARY)**

---

## 📌 Future Scope

- Real hardware deployment and calibration  
- Integration with machine learning models for predictive analytics  
- Live AQI display and data cloud sync

---

🛠️ Feel free to fork, study, and build upon this simulation.  
📩 Contributions and suggestions are always welcome!
