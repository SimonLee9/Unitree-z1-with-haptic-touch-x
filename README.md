# Index

---

## Setting

### z1_examples
   
   ~/z1_ws/src/z1_ros/z1_examples

### z1_bringup

   ~/z1_ws/src/z1_ros/z1_bringup

### tcpip_package

   ~/catkin_ws/src/tcpip_package  

### getdata_package

   ~/catkin_ws/src/getdata_package

---

# 0. Operating sequence

## 1. Z1 - Ready

- Simulation or Real

## 2. Z1 - Operation with TCP/IP

1. Robot PC ↔ Haptic PC [TCP/IP]
2. TCP/IP → Robot PC → Z1
3. Force sensor data from arduino

---

# 1. Z1 - Ready

### Case1) Run with ROS moveit

- **Case 1-1) Simulation**
    
    ```bash
    source ~/z1_ws/devel/setup.bash
    roslaunch z1_bringup sim_arm.launch UnitreeGripperYN:=true rviz:=true
    ```
    
    - hotkey
        
        ```bash
        # cd z1_ws
        # sh z-sim
        ```
        
- **Case 1-2) Real**
    
    ```bash
    source ~/z1_ws/devel/setup.bash
    roslaunch z1_bringup real_arm.launch rviz:=true
    ```
    
    - hotkey
        
        ```bash
        # cd z1_ws
        # sh z-real
        ```
        

### Case 2) Run with udp

- **Case 2-1) Simulation**
    
    ```bash
    roslaunch z1_bringup sim_ctrl.launch
    ```
    
- **Case 2-2) Real**
    
    ```bash
    roslaunch z1_bringup real_ctrl.launch
    ```
    

# 2. Z1 - Operation with TCP/IP

### 1. Robot PC ↔ Haptic PC [TCP/IP]

```bash
rosrun tcpip_package tcpip_node.py
#path : catkin_ws/src/tcpip_package/src - tcpip_node.py
```

- hotkey
    
    ```bash
    # cd catkin_ws
    # sh tcp
    ```
    

### 2. TCP/IP → Robot PC → Z1

```bash
rosrun z1_examples move_group_interface_tcp
#path : z1_ws/src/z1_ros/z1_example_examples - move_group_interface.cpp
```

- hotkey
    
    ```bash
    # cd z1_ws
    # sh tcpmove
    ```
    

---

### 3. getting torque data & publish to TCP node

```bash
rosrun z1_examples get_torque.py

# Publish topic about "Joint_ta"
```

---

### etc

```bash
# cd z1_ws/src/z1_ros/z1_sdk/scripts/
python3 example_joint_ctrl.py
```

```bash
# cd z1_ws/src/z1_ros/z1_sdk/scripts/
python3 example_lowcmd.py
```

# 3. Force sensor data from arduino

### 1. Run - rosserial

```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
```

- If you have error about permission
    
    ```bash
    sudo chmod a+rw /dev/ttyACM0
    ```
    

### 2. Run - node

```bash
rosrun getdata_package getdata_node.py
```

- hotkey
    
    ```bash
    # cd z1_ws
    # sh gedata
    ```
