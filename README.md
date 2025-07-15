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

# **Step 3: Setting Up ArduCam PTZ Camera (IMX219) on Raspberry Pi (Ubuntu 24.04)**

## Objective

This step enables you to:

* **Use the ArduCam PTZ camera (IMX219 sensor)** for onboard vision tasks.
* Control the **pan, tilt, zoom, focus, and IR cut** features via **Python and ROS 2 topics**.
* Stream the camera feed in ROS 2 with **/image\_raw** topic publishing.

This is a **critical part of the project** because it allows:

* Onboard visual data capture
* Remote control of the camera's mechanical components (PTZ system)
* Integration with autonomous missions or mapping tasks

---

## Hardware

| Component                | Purpose                                     |
| ------------------------ | ------------------------------------------- |
| **ArduCam PTZ Module**   | Pan-Tilt-Zoom-Focus camera system           |
| **IMX219 Sensor**        | 8MP camera module (Raspberry Pi compatible) |
| **MG995 Servos (2 pcs)** | Pan and Tilt mechanism                      |

---

## Why Custom Installation?

* **Official ArduCam installation guide is outdated for Ubuntu 24.04.**
* So, we follow general steps from ArduCam documentation, but adapt them manually for **Ubuntu 24.04 on Raspberry Pi 4**.

---

## **Installation Steps**

### **Step 3.1: Install Required Dependencies**

Run the following commands **on the Raspberry Pi**:

```bash
sudo apt-get update && sudo apt-get upgrade

sudo apt install -y cmake meson libboost-dev libboost-program-options-dev \
libgnutls28-dev openssl libtiff5-dev libjpeg-dev libpng-dev \
libyaml-dev libepoxy-dev libcamera-dev libdrm-dev libexif-dev \
qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 \
libglib2.0-dev libgstreamer-plugins-base1.0-dev \
python3-pip git python3-smbus python3-opencv

sudo pip3 install jinja2 pyyaml ply
sudo pip3 install --upgrade meson
```

---

### **Step 3.2: Compile libcamera**

Since Raspberry Pi OS support is baked into libcamera but **Ubuntu requires manual compilation**, follow:

```bash
git clone --branch stable https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build
ninja -C build
sudo ninja -C build install
```

##### Reference:

[Libcamera GitHub](https://github.com/raspberrypi/libcamera)

---

### **Step 3.3: Compile libcamera-apps**

```bash
git clone https://github.com/raspberrypi/libcamera-apps.git
cd libcamera-apps
cmake .
make -j4
sudo make install
```

##### Reference:

[Libcamera-apps GitHub](https://github.com/raspberrypi/libcamera-apps)

---

### **Step 3.4: Compile python-kms++**

```bash
git clone https://github.com/ArduCAM/python-kmsplusplus.git
cd python-kmsplusplus
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

---

### **Step 3.5: Install Picamera2**

Picamera2 is the Python library for camera control using libcamera.

```bash
sudo apt install -y python3-picamera2
```

If unavailable, install from source:

```bash
git clone https://github.com/raspberrypi/picamera2.git
cd picamera2
python3 setup.py install
```

##### Reference:

[Picamera2 GitHub](https://github.com/raspberrypi/picamera2)

---

### **Step 3.6: Configure Camera in `/boot/firmware/config.txt`**

Edit the config file:

```bash
sudo vi /boot/firmware/config.txt
```

Make these changes:

* Disable auto camera detection:

  ```
  camera_auto_detect=0
  ```

* Add IMX219 overlay:

  ```
  dtoverlay=imx219
  ```

---

### **Step 3.7: Enable `/dev/dma_heap` Access**

To avoid permission issues with libcamera on Ubuntu, create a udev rule:

```bash
sudo nano /etc/udev/rules.d/raspberrypi.rules
```

Add:

```
SUBSYSTEM=="dma_heap", GROUP="video", MODE="0660"
```

Then add your user to the **video group**:

```bash
sudo usermod -a -G video $USER
```

**Reboot** the Raspberry Pi after this step.

---

## **Step 3.8: Run ArduCam PTZ Example**

Clone the ArduCam PTZ controller repository:

```bash
git clone https://github.com/ArduCAM/PTZ-Camera-Controller.git
cd PTZ-Camera-Controller
sudo python3 FocuserExample.py
```

This Python script allows manual control of:

* **Pan / Tilt** via MG995 servos
* **Zoom / Focus / IR Cut** using the ArduCam lens system

##### Reference:

[ArduCAM PTZ Controller GitHub](https://github.com/ArduCAM/PTZ-Camera-Controller)

---

## **Step 3.9: Use ROS 2 `camera_launch` Node (Custom Node)**

Once the ArduCam PTZ is set up, you can use **`camera_launch`**, a custom ROS 2 node developed for this project.

### What `camera_launch` Does:

* Publishes camera feed to:

  ```
  /image_raw  (sensor_msgs/Image)
  ```

* Provides ROS 2 topics to control the camera:

  | Topic    | Purpose              |
  | -------- | -------------------- |
  | `/pan`   | Control pan angle    |
  | `/tilt`  | Control tilt angle   |
  | `/zoom`  | Control zoom         |
  | `/focus` | Adjust focus         |
  | `/ircut` | Toggle IR cut filter |

This allows **integrating the PTZ camera into ROS 2 pipelines**, including:

* Mapping
* Object tracking
* Visual inspection
* Autonomous control loops

---

## **Summary of Step 3**

At the end of this step, the boat has **full onboard camera capabilities**, including:

* **Streaming images via ROS 2**
* **Controlling the PTZ camera system remotely using ROS 2 topics or Python**

---

## **Why This Is Important**

This camera setup is a **critical part of the autonomous boat project** because:

* It allows for **visual feedback and live streaming** during missions.
* It enables **mechanical control (pan, tilt, zoom)** for tracking and scanning.
* It can be integrated into **future computer vision, mapping, and AI pipelines**.

This is not just camera testing—it’s an **integral sensor system for the autonomous boat platform**.

---

## Useful Links (To Be Added)

| Resource               | Link         |
| ---------------------- | ------------ |
| ArduCam PTZ Controller | <paste link> |
| Libcamera Docs         | <paste link> |
| Picamera2 Docs         | <paste link> |
| Python-KMS++           | <paste link> |

---

# **Step 4: Basic Pixhawk & Raspberry Pi Integration with MAVProxy + DroneKit**

## **Objective**

This step allows you to:

* **Connect the Raspberry Pi directly to the Pixhawk**
* Use **MAVProxy and DroneKit-Python** to **arm the boat, control RC outputs, and send basic commands from Python scripts**

### Why Do This?

This step is not for full autonomous control yet—it’s to:

* Give a **basic understanding of MAVLink contzrol using Python**
* Help students **interact with the Pixhawk from the onboard computer**
* Test simple movement commands before full ROS 2 integration

---

## **Hardware Setup (Power & Connection)**

| Connection                 | Purpose                                                                                                    |
| -------------------------- | ---------------------------------------------------------------------------------------------------------- |
| **Raspberry Pi Power**     | Powered via **Buck Converter** taking **12V AUX output from Power Module**, converted to **5V USB output** |
| **USB-C Cable**            | Connects **Buck Converter USB output** to **Raspberry Pi USB-C power port**                                |
| **Pixhawk → Raspberry Pi** | Use **USB data cable** or **TELEM port via UART to USB converter** for MAVLink communication               |

### **Why Use Buck Converter + USB-C?**

* This method provides **double safety** for the Raspberry Pi by:

  * Isolating Pi power from main ESC/thruster circuit
  * Preventing overvoltage and ensuring stable 5V supply

---

## **Software Installation on Raspberry Pi**

### **Step 4.1: Install MAVProxy**

MAVProxy is a **command-line ground control station** and **MAVLink proxy tool**.

```bash
sudo apt update
sudo apt install python3-pip
pip3 install MAVProxy
```

To run MAVProxy:

```bash
sudo mavproxy.py --master=/dev/ttyACM0 --baudrate 115200 --aircraft myboat
```

* Replace `/dev/ttyACM0` with the correct port (use `ls /dev/tty*` to check)
* If using UART, set correct serial device like `/dev/ttyUSB0`

##### Reference:

[MAVProxy Docs](https://ardupilot.github.io/MAVProxy/html/index.html)

---

### **Step 4.2: Install DroneKit-Python**

DroneKit allows you to **write Python scripts** to control Pixhawk.

```bash
pip3 install dronekit dronekit-sitl pymavlink
```

##### Reference:

[DroneKit-Python GitHub](https://github.com/dronekit/dronekit-python)

---

## **Basic Python Example: Arm and Control the Boat**

Here is a **simple Python script** to:

* Connect to Pixhawk
* Arm the boat
* Send throttle commands (using RC override)

```python
from dronekit import connect, VehicleMode
import time

# Connect to the Pixhawk (replace with your port)
vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)

# Arm the vehicle
vehicle.mode = VehicleMode("MANUAL")
vehicle.armed = True

while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)

print("Armed!")

# Send RC override to control thrusters
# RC channels: 1- Roll, 2- Pitch, 3- Throttle, 4- Yaw
# Set channel 3 (Throttle) to mid (1500 = neutral), >1500 = forward

vehicle.channels.overrides['3'] = 1600  # Move forward
time.sleep(5)
vehicle.channels.overrides['3'] = 1500  # Stop

# Disarm
vehicle.armed = False
vehicle.close()
```

---

## **Summary of Step 4**

At the end of this step:

* You can **power the Raspberry Pi from the boat’s 12V system safely**.
* The **Raspberry Pi can communicate with the Pixhawk over USB**.
* You can run **basic Python scripts using MAVProxy and DroneKit** to:

  * Arm the boat
  * Control RC outputs (like throttle)
  * Disarm the boat after testing

---

## **Why This Step Matters**

While this step is **not for final mission control**, it is important for:

* Teaching **how to send commands to Pixhawk using Python**
* Giving a **feel of MAVLink protocol and basic scripting**
* **Testing hardware connections safely before full autonomy**

This step makes sure future students can quickly prototype simple movements before dealing with complex ROS 2 autonomy stacks.

---

## **Next Steps**

In the next phases, you will:

* **Integrate ROS 2 and MAVROS for full autonomous control**
* Use the camera and sonar in feedback loops
* Plan and execute autonomous missions like mapping or waypoint navigation

---

## Useful Links (To Be Added)

| Resource                   | Link         |
| -------------------------- | ------------ |
| MAVProxy Docs              | <paste link> |
| DroneKit GitHub            | <paste link> |
| ArduPilot RC Override Docs | <paste link> |

---
This is an **important step** because it moves from **basic Python control (Step 4)** to **ROS 2-based communication with Pixhawk**, which is the standard for:

* Autonomous robotics
* Multi-sensor integration
* Mission-level control

---

# **Step 5: Install MAVROS on PC and Raspberry Pi (ROS 2 Integration with Pixhawk)**

## **Objective**

After completing this step, you will be able to:

* **Arm and disarm the boat using ROS 2 commands**
* Use the **Pixhawk autopilot directly with ROS 2 via MAVROS**
* Integrate boat control into a **ROS 2 node system** for future autonomy pipelines

---

## **What is MAVROS?**

**MAVROS** is a **ROS 2 package that bridges MAVLink (Pixhawk’s protocol) with ROS 2 topics and services**.

It allows you to:

* Arm/disarm the boat
* Control RC channels (override)
* Access GPS, IMU, battery, sonar, and more
* Send waypoint and mission commands

---

## **5.1 Install MAVROS on the Laptop (Ground Station)**

### **Step 5.1.1: Install dependencies**

```bash
sudo apt update
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras
```

---

### **Step 5.1.2: Install GeographicLib datasets**

```bash
sudo apt install geographiclib-tools
sudo geographiclib-get-geoids egm96-5
```

##### Reference:

[MAVROS Install Guide](paste link)
[GeographicLib Reference](paste link)

---

## **5.2 Install MAVROS on Raspberry Pi**

On the **Raspberry Pi (Ubuntu 24.04 + ROS 2 Base)**:

### **Step 5.2.1: Install MAVROS**

```bash
sudo apt update
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras
```

---

### **Step 5.2.2: Install GeographicLib datasets**

```bash
sudo apt install geographiclib-tools
sudo geographiclib-get-geoids egm96-5
```

---

## **5.3 Connect Pixhawk to Raspberry Pi**

Use either:

* **USB Cable** (recommended for first tests)
* Or **TELEM2 port via UART to USB converter**

Pixhawk should appear as `/dev/ttyACM0` or `/dev/ttyUSB0`.
Check with:

```bash
ls /dev/tty*
```

---

## **5.4 Run MAVROS Node**

### **On the Raspberry Pi or PC (example for Pi):**

```bash
ros2 launch mavros mavros.launch.py fcu_url:=serial:///dev/ttyACM0:115200
```

* Replace `/dev/ttyACM0` with your port if different.
* `fcu_url` defines how MAVROS communicates with Pixhawk.

---

## **5.5 Arm and Disarm the Boat Using ROS 2**

Once MAVROS is running, you can **arm or disarm the boat directly from ROS 2**.

### **Using ROS 2 CLI:**

To **arm the boat**:

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

To **disarm the boat**:

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

---

### **Using Python ROS 2 Node Example:**

Create a Python node to arm the boat:

```python
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool

class BoatArmer(Node):
    def __init__(self):
        super().__init__('boat_armer')
        self.client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        self.arm()

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('Boat Armed!')
        else:
            self.get_logger().info('Arming Failed')

def main(args=None):
    rclpy.init(args=args)
    node = BoatArmer()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## **Why This Step Is Important**

After this step, the system is no longer just a hardware test platform:

* The **Pixhawk and Raspberry Pi are fully integrated via ROS 2**
* You can **control the boat in ROS 2 pipelines**, setting the foundation for:

  * Autonomous missions
  * Sensor fusion
  * State estimation
  * Path planning
  * Advanced research

---

## **Project Status After Step 5**

| Task                              | Status |
| --------------------------------- | ------ |
| Pixhawk Manual Setup              | ✅      |
| Raspberry Pi Setup                | ✅      |
| Camera System Setup               | ✅      |
| Python MAVLink Control (DroneKit) | ✅      |
| **ROS 2 MAVROS Integration**      | ✅      |

---

## **Next Steps (Optional Future Work)**

1. **Integrate sensors (Sonar, GPS, Camera) into ROS 2 system**
2. **Implement autonomous navigation (path planning, mapping)**
3. **Build data logging and mission replay systems**
4. **Add safety features (failsafe scripts, watchdog nodes)**

---

## Useful Links (To Be Added)

| Resource                | Link         |
| ----------------------- | ------------ |
| MAVROS Docs             | <paste link> |
| ROS 2 Service Call Docs | <paste link> |
| DroneKit-Python         | <paste link> |

---





## Contribution & Future Work

This documentation is meant to serve as a **starting point for future interns and students**.
Please feel free to contribute by:

* Adding detailed steps for advanced control, ROS 2 integration, mapping, etc.
* Improving hardware integration
* Running field tests and collecting data

For any questions, contact **Chandan Singh Chauhan** or your project supervisor at IIT Guwahati.
