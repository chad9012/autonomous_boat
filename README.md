# Autonomous Boat Project – IIT Guwahati

### Internship Documentation – Chandan Singh Chauhan

This project focuses on building and extending an **Autonomous Surface Vehicle (ASV)** platform using a combination of **Pixhawk 2.4.8**, **Raspberry Pi 4 (8GB)**, and other open-source tools for autonomous navigation, seabed mapping, and surface robotics research.

This repository serves as a **reference and guide for future students**, so they can easily continue or expand on this work.

---

## Repositories

| Repo                             | Purpose                                              |
| -------------------------------- | ---------------------------------------------------- |
| [`autonomous_boat_pi_ubuntu`](#) | Code and setup for **Raspberry Pi 4 (Ubuntu 24.04)** |
| [`autonomous_boat_pc`](#)        | Code and setup for **Ground Control Station (PC)**   |

---

## Hardware Setup Summary

### Components Used

| Component                                   | Details                                                                      |
| ------------------------------------------- | ---------------------------------------------------------------------------- |
| **Pixhawk 2.4.8**                           | Flight Controller (ArduRover Firmware)                                       |
| **Raspberry Pi 4 (8GB RAM)**                | Onboard computer (Ubuntu 24.04)                                              |
| **ESC: Blue Robotics Basic ESC (2 pcs)**    | Controls thruster motors                                                     |
| **Thrusters: Blue Robotics T200 (2 pcs)**   | Propulsion system                                                            |
| **Power Distribution Board: Matek Systems** | Distributes battery power                                                    |
| **Power Module**                            | Powers Pixhawk and ESCs; also steps down voltage to 5V for ArduCam PTZ motor |
| **Battery: Orange LiPo 8000 mAh**           | Main power source                                                            |
| **Buck Converter (5-25V to USB 5V)**        | Powers Raspberry Pi via USB                                                  |
| **Camera: ArduCam PTZ with 219 MX Sensor**  | For visual data collection                                                   |
| **Telemetry Module (Air & Ground)**         | Radio communication with PC                                                  |
| **GPS: u-blox M8N**                         | Provides global position                                                     |
| **Sonar: Blue Robotics Ping 2**             | Depth measurement                                                            |
| **Servos: MG995 (2 pcs)**                   | PTZ pan-tilt mechanism                                                       |

---

## Project Goals

The overall project aims to:

* **Build a modular autonomous boat platform**
* **Enable remote/manual control using RC and telemetry**
* **Integrate computer vision, mapping, and navigation using onboard computing (Raspberry Pi)**
* **Enable future expansion to autonomous missions, data collection, and mapping**

---

## What This Internship Covered

During this internship, I focused on **setting up the complete hardware and software infrastructure** for an autonomous surface vehicle, including:

1. **Basic boat assembly and Pixhawk setup**
2. **RC manual control and telemetry link**
3. **Integration of onboard computing (Raspberry Pi 4 with Ubuntu 24.04)**
4. **ROS2 + MAVROS setup for advanced control (To be detailed later)**
5. **Sensor integration: GPS, Sonar, Camera (To be detailed later)**

This README currently documents **Step 1: Basic Boat Setup**. Further steps can be easily added below this section as the project progresses.

---

# **Step 1: Basic Boat Setup – Manual & RC Control**

### Hardware Connection (Basic Setup)

This step covers setting up the **bare minimum working boat system** to get manual control and telemetry working. The Raspberry Pi is **NOT used yet** in this phase.

#### Components Required for Step 1:

* Pixhawk 2.4.8
* Matek Power Distribution Board
* Blue Robotics Basic ESCs (2 pcs)
* Blue Robotics T200 Thrusters (2 pcs)
* Power Module (for powering Pixhawk and ESCs)
* Orange 8000 mAh LiPo Battery
* RC Transmitter & Receiver
* u-blox M8N GPS
* Telemetry Radio (Air & Ground modules)

---

### Wiring Guide

**Battery Power Distribution:**

| Connection                                    | Purpose                                                     |
| --------------------------------------------- | ----------------------------------------------------------- |
| **Battery → Power Module**                    | Feeds power to Pixhawk (5V regulated)                       |
| **Battery → Power Distribution Board → ESCs** | Powers ESCs and Thrusters directly                          |
| **Power Module AUX Output (5V)**              | Powers PTZ motor (via buck converter or directly if needed) |

---

**Pixhawk Connections:**

| Port              | Component                                            |
| ----------------- | ---------------------------------------------------- |
| **Main Out 1, 2** | ESC signals for Left/Right Thrusters                 |
| **RC IN**         | RC Receiver (PWM/SBUS depending on receiver type)    |
| **GPS Port**      | u-blox M8N GPS                                       |
| **Telem1**        | Telemetry Air Module (for QGroundControl connection) |

---

### Software Setup

#### 1. Install **ArduRover Firmware** on Pixhawk

Use **QGroundControl** to:

* Flash **ArduRover firmware**
* Calibrate **ESCs**, **RC Transmitter**, and **Compass**
* Set **vehicle type** to "Boat / Ground Rover"

##### Reference:

* [ArduRover Setup Docs](paste link)
* [QGroundControl Download](paste link)

---

### Manual Control

Once setup is complete, you can:

* **Control the boat manually** using your **RC transmitter**
* **Monitor GPS position, battery voltage, and sensor data** in **QGroundControl** via telemetry radio

---



## Useful Links (To Be Added)

| Resource                    | Link         |
| --------------------------- | ------------ |
| ArduPilot Documentation     | <paste link> |
| Blue Robotics T200 Thruster | <paste link> |
| Blue Robotics Basic ESC     | <paste link> |
| Ping Sonar Docs             | <paste link> |
| ArduCam PTZ Module          | <paste link> |
| Matek PDB                   | <paste link> |
| MG995 Servo                 | <paste link> |

---

# **Step 2: PC and Raspberry Pi Setup**

## Objective

This part of the project focuses on setting up the **software environment** on both the **Ground Control PC** and the **Raspberry Pi 4** to enable:

* ROS 2 communication
* Remote access to the Raspberry Pi via GUI
* Ground Station development and visualization

---

## **2.1 PC Setup (Dual Boot with Ubuntu 24.04)**

### Why Dual Boot?

To develop ROS 2 applications and control the boat from the **PC (Ground Station)**, you need **Ubuntu 24.04 (Noble)** because it supports the latest **ROS 2 Jazzy** distribution.

---

### Steps to Install Dual Boot Ubuntu 24.04 on Laptop

1. **Download Ubuntu 24.04 ISO**

   [Ubuntu 24.04 Download](https://releases.ubuntu.com/24.04/)

2. **Create Bootable USB**

   Use **Rufus** or **Balena Etcher** on Windows to create a bootable USB.

   * [Rufus Download](https://rufus.ie/)
   * [Balena Etcher Download](https://www.balena.io/etcher/)

3. **Partition Your Disk**

   * Use **Windows Disk Management** to shrink your existing partition and free up space (at least 50 GB recommended).

4. **Install Ubuntu 24.04**

   * Boot from the USB and select **Install Ubuntu**.
   * Choose **Install Ubuntu alongside Windows** (dual boot).
   * Follow the prompts.

5. **Post-Installation**

   * Update your system:

     ```bash
     sudo apt update && sudo apt upgrade
     ```

---

### Install ROS 2 Jazzy on PC

1. **Set locale**

   ```bash
   sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   ```

2. **Add ROS 2 Jazzy Repo**

   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
   sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list'
   ```

3. **Install ROS 2 Jazzy Desktop**

   ```bash
   sudo apt update
   sudo apt install ros-jazzy-desktop
   ```

4. **Source ROS 2**

   ```bash
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

##### Reference:

[ROS 2 Jazzy Installation Guide](paste ROS2 jazzy link)

---

## **2.2 Raspberry Pi 4 Setup (Ubuntu 24.04 + Lubuntu GUI + ROS 2 Base)**

### Hardware

* **Raspberry Pi 4 (8 GB RAM)**
* **Micro SD Card (at least 32 GB recommended)**

---

### Install Ubuntu 24.04 Server on Pi (Headless Setup)

1. **Download Pi Image**

   [Ubuntu 24.04 Server for Raspberry Pi](https://cdimage.ubuntu.com/releases/24.04/release/)

2. **Flash Image to SD Card**

   Use **Raspberry Pi Imager** or **Balena Etcher**.

   * [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
   * [Etcher Download](https://www.balena.io/etcher/)

3. **Enable SSH (Headless Setup)**

   * After flashing, place an empty file named `ssh` (no extension) in the `/boot` partition of the SD card.

4. **Boot the Raspberry Pi**

   * Connect to your local network via Ethernet or Wi-Fi.
   * Find Pi’s IP using `nmap` or check your router.

5. **SSH into Raspberry Pi**

   ```bash
   ssh ubuntu@<pi-ip-address>
   ```

   Default password: `ubuntu` (you'll be prompted to change it at first login)

---

### Install Lubuntu Desktop (Lightweight GUI)

Since **Ubuntu Server** has no GUI, you can install **Lubuntu Desktop** for lightweight graphical interface:

```bash
sudo apt update && sudo apt install lubuntu-desktop
```

---

### Install NoMachine (Remote Desktop Access)

To view the GUI from your **PC**, install **NoMachine** on both the **Raspberry Pi** and **your PC**.

1. **Download NoMachine**

   [NoMachine Download Page](https://www.nomachine.com/download)

2. **Install on Raspberry Pi (ARM64 DEB package)**

   ```bash
   sudo dpkg -i nomachine_<version>_arm64.deb
   ```

3. **Install on PC (AMD64 DEB package)**

   ```bash
   sudo dpkg -i nomachine_<version>_amd64.deb
   ```

4. **Access Pi GUI**

   * Open NoMachine on your **PC**
   * Connect to the **Raspberry Pi IP**
   * Login with `ubuntu` user and password

##### Reference:

[NoMachine Remote Access Guide](paste link)

---

### Install ROS 2 Jazzy Base on Raspberry Pi

For the Pi, install **ROS 2 base packages** (no GUI tools, since the Pi is for onboard computing):

1. **Set locale**

   ```bash
   sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   ```

2. **Add ROS 2 Jazzy Repo**

   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
   sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list'
   ```

3. **Install ROS 2 Base**

   ```bash
   sudo apt update
   sudo apt install ros-jazzy-ros-base
   ```

4. **Source ROS 2**

   ```bash
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

##### Reference:

[ROS 2 Jazzy on ARM64 Install Guide](paste link)

---

## Summary of Step 2

At the end of this step:

* **PC is set up with dual boot Ubuntu 24.04 + ROS 2 Jazzy Desktop**
* **Raspberry Pi is set up with Ubuntu 24.04 Server + Lubuntu GUI + NoMachine + ROS 2 Jazzy Base**

---

## Useful Links (To Be Added)

| Resource              | Link         |
| --------------------- | ------------ |
| Ubuntu 24.04 Download | <paste link> |
| Raspberry Pi Imager   | <paste link> |
| ROS 2 Jazzy Install   | <paste link> |
| NoMachine Install     | <paste link> |
| Lubuntu Desktop       | <paste link> |

---





## Contribution & Future Work

This documentation is meant to serve as a **starting point for future interns and students**.
Please feel free to contribute by:

* Adding detailed steps for advanced control, ROS 2 integration, mapping, etc.
* Improving hardware integration
* Running field tests and collecting data

For any questions, contact **Chandan Singh Chauhan** or your project supervisor at IIT Guwahati.
