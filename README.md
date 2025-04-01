# Line Maze Solver (ESP32)
A maze-solving robot using node-based mapping and shortest path optimization.

## Features
- **ESP32** microcontroller
- **Node-based mapping** for efficient maze solving
- **Dry run & final optimized path**
- Uses **TCRT5000 IR sensors** to detect paths

## How It Works
1. The robot explores the maze and records intersections.
2. It creates a tree structure of paths.
3. On the final run, it finds the shortest path to the goal.

## Setup
1. **Install Dependencies**  
   - Install **Arduino IDE**  
   - Install ESP32 board support  
   - Install required libraries (`Wire.h`, `Arduino.h`, etc.)
2. **Upload Code**  
   - Open `perfectoperfect.ino` in Arduino IDE  
   - Select the correct **ESP32 board** and upload  
3. **Hardware Connections**  
   - Connect **IR sensors, motors, and drivers** as per the [wiring diagram](images/wiring_diagram.png)  
4. **Run the Robot!**  
   - Start the robot and let it explore! 🚀


## Components Used
- **ESP32**
- **TCRT5000 IR Sensor** (x5)
- **DRV8833 Motor Driver**
- **N20 Encoded Motors**

## Repository Structure

