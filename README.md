# Raspberry Pi Pico FreeRTOS Demo (Sensores & Atuadores)

## 📌 Visão Geral
Este projeto demonstra o uso do **FreeRTOS** no **Raspberry Pi Pico (RP2040)**, integrando múltiplos sensores e atuadores para explorar, na prática, os recursos de um sistema operacional de tempo real.

O sistema atua como uma **plataforma completa de testes de RTOS**, apresentando conceitos centrais do FreeRTOS como escalonamento de tarefas, comunicação entre tarefas, sincronização, gerenciamento de memória e temporizadores — todos aplicados a dados reais de sensores e controle de atuadores.

---

## ⚙️ Funcionalidades
- 🧠 **Integração com FreeRTOS**
  - Multitarefa com escalonamento em tempo real
  - Escalonador preemptivo

- 🧵 **Gerenciamento de Tarefas**
  - Múltiplas tarefas concorrentes
  - Priorização de tarefas
  - Execução periódica e orientada a eventos

- 🔄 **Comunicação entre Tarefas**
  - Filas (queues) para troca de dados entre tarefas

- 🔒 **Mecanismos de Sincronização**
  - Mutex para proteção de recursos compartilhados
  - Semáforos binários e de contagem

- ⏱️ **Temporizadores**
  - Temporizadores de software para operações agendadas
  - Execução baseada em callbacks

- 🧮 **Gerenciamento de Memória**
  - Alocação dinâmica usando heap do FreeRTOS
  - Uso eficiente de memória

- 🔌 **Integração de Sensores & Atuadores**
  - Aquisição de dados de sensores em tempo real
  - Controle de atuadores baseado nos dados processados

---

## 🧠 Arquitetura do Sistema
- **Microcontrolador:** Raspberry Pi Pico (RP2040)  
- **RTOS:** FreeRTOS  

- **Periféricos:**
  - Sensores (temperatura, distância, movimento, etc.)  
  - Atuadores (LEDs, buzzers, motores, etc.)  

- **Módulos Principais:**
  - Escalonador de Tarefas  
  - Filas (troca de dados)  
  - Mutex & Semáforos  
  - Temporizadores de Software  
  - Gerenciador de Memória  

---

## ⚡ Conceitos de FreeRTOS Demonstrados

### 🧵 Tarefas
- Execução concorrente de múltiplas tarefas  
- Diferentes prioridades e frequências de execução  
- Tarefas periódicas de leitura de sensores  

### 📬 Filas (Queues)
- Troca segura de dados entre tarefas  
- Dados de sensores enviados para tarefas de processamento/controle  

### 🔒 Mutex
- Proteção de recursos compartilhados (ex: periféricos, interfaces de comunicação)  
- Evita condições de corrida (race conditions)  

### 🚦 Semáforos
- Sincronização entre tarefas  
- Sinalização de eventos entre interrupções e tarefas  

### ⏱️ Temporizadores
- Temporizadores de software para ações periódicas  
- Execução baseada em callbacks  

### 🧮 Gerenciamento de Memória
- Alocação dinâmica de memória  
- Otimização do uso da heap  

### ⚙️ Escalonador
- Escalonamento preemptivo  
- Troca de tarefas baseada em prioridade e tempo  

---

## 🖥️ Funcionamento do Sistema
O sistema executa múltiplas tarefas simultaneamente, como:

- Tarefas de aquisição de sensores (leitura de dados)  
- Tarefas de processamento de dados  
- Tarefas de controle de atuadores  
- Comunicação entre tarefas usando filas  
- Sincronização utilizando mutex e semáforos  
- Execução periódica controlada por temporizadores  

Todas as operações são gerenciadas pelo **escalonador do FreeRTOS**, garantindo execução determinística e eficiente.

---

## 📊 Aplicações
- Plataforma de aprendizado e ensino de RTOS  
- Prototipagem de sistemas embarcados  
- Sistemas de sensores em tempo real  
- Base para desenvolvimento de IoT  

---

## 🚀 Melhorias Futuras
- Integração com comunicação sem fio (Wi-Fi / Bluetooth)  
- Dashboard para visualização de dados  
- Otimização de baixo consumo de energia  
- Estratégias avançadas de escalonamento  
- Integração com Edge AI  

---

## 🛠️ Tecnologias & Habilidades
- Sistemas Embarcados  
- Sistemas Operacionais de Tempo Real (RTOS)  
- FreeRTOS  
- Raspberry Pi Pico (RP2040)  
- Multithreading & Concorrência  
- Comunicação entre Tarefas (Filas)  
- Sincronização (Mutex & Semáforos)  
- Gerenciamento de Memória  
- Integração de Sensores & Atuadores  

---

## 📷 Preview
(Adicione imagens do seu setup, saída do monitor serial ou comportamento do sistema aqui)

---

## 👩‍💻 Autor
Desenvolvido por Mickaelly Freitas Nobre

---


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
