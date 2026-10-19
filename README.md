# 🤖 3-DOF Robotic Arm (RViz + Gazebo + Arduino + Slider Control)

Yeh project 3 Degree of Freedom (3-DOF) robotic arm ke liye hai jisme:
1. **Slider Control GUI**: 3 Sliders (Joint 1, Joint 2, Joint 3) se 0° se 180° tak control hota hai.
2. **RViz 2**: Arm live synchronize hoke move hota hai (`/joint_states`).
3. **Gazebo**: Physics simulation me arm real-time move hota hai (`/arm_controller/joint_trajectory`).
4. **Arduino Hardware**: Serial port (`115200` baud) ke through 3 real servo motors synchronized move hoti hain!

---

## 🔌 Hardware Circuit & Wiring

### Servo Pin Mapping:
- **Servo 1 (Base / Waist Rotation)**: Arduino Pin **9** (PWM)
- **Servo 2 (Shoulder Pitch)**: Arduino Pin **10** (PWM)
- **Servo 3 (Elbow Pitch)**: Arduino Pin **11** (PWM)

### Power Connections:
> ⚠️ **Important**: 3 Servos ko Arduino ke 5V pin se directly power mat dein (Arduino restart ho sakta hai ya overheat hoga). External 5V 2A–3A Power Supply ya Battery use karein!
- Servos **VCC (Red wire)** ➡️ External 5V (+)
- Servos **GND (Brown/Black wire)** ➡️ External GND (-) **AND** Arduino GND (Common Ground zaruri hai)
- Servos **Signal (Orange/Yellow wire)** ➡️ Arduino Pins 9, 10, 11

---

## 🛠 Step 1: Arduino Code Upload (Arduino IDE)

1. Arduino IDE open karein.
2. File open karein:
   ```
   three_dof_arm/arduino_3dof/arduino_3dof.ino
   ```
3. Board aur Port select karein (e.g. *Arduino Uno / Nano* aur `/dev/ttyACM0` ya `/dev/ttyUSB0`).
4. **Upload (Ctrl+U)** click karein.
5. Upload hone ke baad Arduino IDE ka **Serial Monitor** kholein (Baud rate: **115200**).
6. Test karne ke liye Serial Monitor me type karein:
   ```text
   90,90,90
   ```
   ya
   ```text
   45,120,60
   ```
   Servos move hongi aur response aayega: `ACK: Joint1=45 deg | Joint2=120 deg | Joint3=60 deg`.
7. **Important**: Python script chalane se pehle Arduino IDE ka Serial Monitor **close** kar dein (taaki Python serial port open kar sake).

---

## 🚀 Step 2: Sab Kuch Ek Saath Start Karein (RViz + Gazebo + Sliders + Arduino)

Terminal me yeh command chalayein:

```bash
cd ~/three_dof_arm
./start_all.sh
```

Is command se:
1. Purane processes clean honge.
2. **Gazebo** start hoga aur 3-DOF arm spawn hoga.
3. Controllers activate honge.
4. **RViz** open hoga 3D arm model ke saath.
5. **Interactive Slider GUI** open hoga jo automatically Arduino ko detect karke connect kar lega!

---

## 🎛 Step 3: Slider Control

Open hui GUI window me:
- **Joint 1 Slider (Base)**: 0° se 180° (Default: 90°)
- **Joint 2 Slider (Shoulder)**: 0° se 180° (Default: 90°)
- **Joint 3 Slider (Elbow)**: 0° se 180° (Default: 90°)
- **Home Button**: Sabhi motors ko 90° center position par le aata hai.
- **Auto Sweep Demo**: Arm ka automatic demonstration movement run karta hai.
- **Serial Status**: Dikhata hai Arduino connected hai ya nahi.

---

## 📁 Files Summary

- [arduino_3dof.ino](file:///home/anmol/three_dof_arm/arduino_3dof/arduino_3dof.ino): Arduino C++ code (Serial parser + 3 Servos).
- [three_dof_arm.urdf](file:///home/anmol/three_dof_arm/urdf/three_dof_arm.urdf): 3-DOF Robot Arm URDF model with ros2_control.
- [controllers.yaml](file:///home/anmol/three_dof_arm/config/controllers.yaml): ros2_control configuration for Gazebo.
- [gui_controller.py](file:///home/anmol/three_dof_arm/gui_controller.py): Python Tkinter GUI slider controller (RViz, Gazebo & Serial sync).
- [bridge.py](file:///home/anmol/three_dof_arm/bridge.py): Standalone ROS 2 `/joint_states` to Serial bridge.
- [launch_sim.py](file:///home/anmol/three_dof_arm/launch/launch_sim.py): Gazebo simulation launch file.
- [start_all.sh](file:///home/anmol/three_dof_arm/start_all.sh): 1-click startup bash script.
