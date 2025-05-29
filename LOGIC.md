# NUS Team Calibur Robot Firmware -- Logic Flow

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#overview">Overview</a></li>
    <li><a href="#folder-structure">Folder Structure</a></li>
    <li>
      <a href="#logic-flow">Logic Flow</a>
      <ul>
        <li><a href="#1-startup-and-initialization">1. Startup and Initialization</a></li>
        <li><a href="#2-rtos-task-setup">2. RTOS Task Setup</a></li>
      </ul>
    </li>
    <li>
      <a href="#tasks-and-processes">Tasks and Processes</a>
      <ul>
        <li><a href="#1-imu-processing-task">1. IMU Processing Task</a></li>
        <li><a href="#2-motor-calibration-task">2. Motor Calibration Task</a></li>
        <li><a href="#3-control-input-task">3. Control Input Task</a></li>
        <li><a href="#4-referee-processing-task">4. Referee Processing Task</a></li>
        <li><a href="#5-buzzing-task">5. Buzzing Task</a></li>
        <li><a href="#6-hud-task">6. HUD Task</a></li>
      </ul>
    </li>
  </ol>
</details>


## Overview 
This repository contains the firmware for the **Development Board C**, developed by NUS Calibur Robotics for the RoboMaster competition. it serves as the main control firmware for the **Standrd, Hero and Sentry** robots

>Todo: Add high level overview with diagrams 

## Folder Structure
The structure below only includes the two main components: `Core` and `System`. Other files (e.g. peripheral initialization, HAL drivers, and CubeMX-generated boilerplate) are omitted for clarity

- `Core`: Contains all firmware related to robot control
- `System`: Handles communications with external devices such as other microcontrollers or mini PC

```bash
#todo: add short descriptions of what is in each folder
├───Core                 
│   ├───BSP           
│   │   ├───Inc
│   │   └───Src
│   ├───Inc            
│   ├───robot_config    
│   ├───Src            
│   ├───Startup         
│   └───Tasks
│       ├───Inc
│       └───Src
├───System         
│   ├───BRoCo           
│   │   ├───include 
│   │   │   ├───BRoCo       
│   │   │   ├───Build       
│   │   │   └───Protocol    
│   │   └───src
│   ├───Threads        
│   │   ├───Inc
│   │   └───Src
│   └───utils           
│       ├───Inc
│       └───Src
```

## Logic Flow

### 1. Startup and Initialization
The program starts in `main.c`, where the program initalizes all the necessary hardware and peripherals. All these are defined through the `.ioc` file in STM32CubeMX

After the hardware and peripheral setup, the function `startup_task()` is called to initialize the LED, buzzer and servo GPIO pins

Finally, the FreeRTOS kernel is initialized with `osKernelInitialize()`, and user-defined RTOS components are set up via `MX_FREERTOS_Init()`. Once the scheduler is started with `osKernelStart()`, control is handed over to the FreeRTOS environment

### 2. RTOS Task Setup
The `MX_FREERTOS_Init()` handles all the FreeRTOS setup and tasks before the scheduler is launched. 




## Tasks and Processes

### 1. IMU Processing Task

### 2. Motor Calibration Task 

### 3. Control Input Task 

### 4. Referee Processing Task

### 5. Buzzing Task 

### 6. HUD Task