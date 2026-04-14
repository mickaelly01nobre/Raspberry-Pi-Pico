# Raspberry Pi Pico FreeRTOS Demo (Sensors & Actuators)

## 📌 Overview
This project demonstrates the use of **FreeRTOS** on the **Raspberry Pi Pico (RP2040)**, integrating multiple sensors and actuators to explore real-time operating system features in practice.

The system acts as a **comprehensive RTOS test platform**, showcasing core FreeRTOS concepts such as task scheduling, inter-task communication, synchronization, memory management, and timers — all applied to real sensor data and actuator control.

---

## ⚙️ Features
- 🧠 **FreeRTOS Integration**
  - Multitasking with real-time scheduling
  - Preemptive scheduler

- 🧵 **Task Management**
  - Multiple concurrent tasks
  - Task prioritization
  - Periodic and event-driven execution

- 🔄 **Inter-Task Communication**
  - Queues for data exchange between tasks

- 🔒 **Synchronization Mechanisms**
  - Mutex for shared resource protection
  - Binary and counting semaphores

- ⏱️ **Timers**
  - Software timers for scheduled operations
  - Event-based callbacks

- 🧮 **Memory Management**
  - Dynamic allocation using FreeRTOS heap
  - Efficient memory usage

- 🔌 **Sensors & Actuators Integration**
  - Real-time sensor data acquisition
  - Actuator control based on processed data

---

## 🧠 System Architecture
- **Microcontroller:** Raspberry Pi Pico (RP2040)  
- **RTOS:** FreeRTOS  

- **Peripherals:**
  - Sensors (temperature, distance, motion, etc.)  
  - Actuators (LEDs, buzzers, motors, etc.)  

- **Core Modules:**
  - Task Scheduler  
  - Queues (data exchange)  
  - Mutex & Semaphores  
  - Software Timers  
  - Memory Manager  

---

## ⚡ FreeRTOS Concepts Demonstrated

### 🧵 Tasks
- Concurrent execution of multiple tasks  
- Different priorities and execution frequencies  
- Periodic sensor reading tasks  

### 📬 Queues
- Safe data exchange between tasks  
- Sensor data sent to processing/control tasks  

### 🔒 Mutex
- Protect shared resources (e.g., peripherals, communication interfaces)  
- Avoid race conditions  

### 🚦 Semaphores
- Task synchronization  
- Event signaling between interrupts and tasks  

### ⏱️ Timers
- Software timers for periodic actions  
- Callback-based execution  

### 🧮 Memory Management
- Dynamic memory allocation  
- Heap usage optimization  

### ⚙️ Scheduler
- Preemptive scheduling  
- Task switching based on priority and timing  

---

## 🖥️ System Operation
The system runs multiple tasks simultaneously, such as:

- Sensor acquisition tasks (reading data)  
- Data processing tasks  
- Actuator control tasks  
- Communication between tasks using queues  
- Synchronization using mutexes and semaphores  
- Periodic execution controlled by timers  

All operations are managed by the **FreeRTOS scheduler**, ensuring deterministic and efficient execution.

---

## 📊 Applications
- RTOS learning and teaching platform  
- Embedded systems prototyping  
- Real-time sensor systems  
- IoT development base  

---

## 🚀 Future Improvements
- Integration with wireless communication (Wi-Fi / Bluetooth)  
- Data visualization dashboard  
- Low-power optimization  
- Advanced scheduling strategies  
- Edge AI integration  

---

---

## 🛠️ Technologies & Skills
- Embedded Systems  
- Real-Time Operating Systems (RTOS)  
- FreeRTOS  
- Raspberry Pi Pico (RP2040)  
- Multithreading & Concurrency  
- Inter-Task Communication (Queues)  
- Synchronization (Mutex & Semaphores)  
- Memory Management  
- Sensor & Actuator Integration  

---

## 📷 Preview
(Add images of your setup, serial monitor output, or system behavior here)

---

## 👩‍💻 Author
Developed by Mickaelly Freitas Nobre

---
