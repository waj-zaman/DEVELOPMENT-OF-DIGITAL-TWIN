## In this file we will discuss the entire workflow and the challenges that we faced in completing this project.

## PHASE 1 - INSTALLATION AND SETUP

1. Installed Ubuntu [ Jammy Jellyfish ]
2. Installed ROS2 [ Humble Hawksbill ] :
    1. Set Locale :
    ```bash
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    ```
    2. Add ROS2 Repo :
    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update
    ```
    3. Added ROS2 GPG Key :
    ```bash
    sudo apt install curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```
    4. Added ROS2 Source list :
    ```bash
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
    5. Install ROS2 Humble Desktop :
    ```bash
    sudo apt update
    sudo apt install ros-humble-desktop
    ```
    6. Source ROS2 :
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
    7. Check installation :
    ```bash
    ls /opt/ros/
    ```
3. Installed Gazebo [ Ignition Fortress ] :
    1. Add Fortress Repo :
    ```bash
    sudo apt update
    sudo apt install curl gnupg lsb-release
    ```
    ```bash
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
    ```
    2. Add Gazebo Key :
    ```bash
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    ```
    3. Install Gazebo Fortress :
    ```bash
    sudo apt update
    sudo apt install ignition-fortress
    ```
    4. Launch Gazebo :
    ```bash
    ign gazebo
    ```
4. Installed VSCode [ Using App Store ]
5. Installed GIT 
    1. Update the packages :
    ```bash
    sudo apt update
    sudo apt install git
    ```
    2. Verify Git :
    ```bash
    git --version
    ```
6. Created a Repo and connected to local repo.
7. Initiated a ROS Workspace.
    1. Created a empty Directory.
    2. Created a src directory inside.
    3. Terminal [~/Desktop/ros2_ws/] :
    ```bash
    colcon build
    ```
    4. Source the workspace :
    ```bash
    source install/setup.bash
    ```
8. Connected the Remote Repo to local git repo.

**NOTE : While pushing the local data to the remote repo, there is a chance that src folder (being empty) might be neglected. To tackle that create a simple hidden file and then push again.**

## PHASE 2 - PRACTICAL ROS2 FOR DIGITAL TWIN :

- Before we proceed, lets look at the ideal steps of creating nodes. 
1. Create A workspace along with src folder inside.
2. Build the workspace using COLCON BUILD tool.
3. Inside Src folder, create a workspace using command 
```bash
ros2 pkg create my_package --build-type ament_python --dependencies rclpy
```
4. In this workspace go inside the folder with same workspace name and here create the python files which will act as nodes. 
5. After creation make this files executable using the command :
Terminal [/ros2_ws/src/my_package/ ]
```bash
chmod +x my_package/node1.py
```
6. Then in setup.py file in the package folder --> in the last section add the executable names that we will use to call this files.
7. In the same file add the external packages that we imported into our project like rclpy in the depend tags section.


**Practice Robot in ROS2 before Digital Twin :**
1. Create my_robot_description package [ FOllow the general Steps ]
2. Create a directory to store robot.urdf file.
Terminal [~/Desktop/ros2_ws/src/my_robot_description/my_robot_description/]
```bash
mkdir urdf
```
3. Create a basic robot.xacro file :
    1. Create a empty xacro file using gedit : 
    Terminal [~/Desktop/ros2_ws/src/my_robot_description/my_robot_description/urdf] :
    ```bash
    gedit robot.xacro
    ```
    2. In the robot.xacro file, enter the code :
    ```xml
    <?xml version="1.0"?>
    <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">

    <!-- Chassis Link-->
    <link name="chassis">
        <visual>
        <geometry>
            <box size="0.5 0.3 0.1"/>
        </geometry>
        <material name="blue"/>
        </visual>
        <collision>
        <geometry>
            <box size="0.5 0.3 0.1"/>
        </geometry>
        </collision>
        <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0"
                iyy="0.01" iyz="0.0"
                izz="0.01"/>
        </inertial>
    </link>

    <!-- Left Wheel Link-->
    <link name="left_wheel">
        <visual>
        <geometry>
            <cylinder radius="0.05" length="0.02"/>
        </geometry>
        <material name="black"/>
        </visual>
    </link>

    <!-- Right Wheel Link-->
    <link name="right_wheel">
        <visual>
        <geometry>
            <cylinder radius="0.05" length="0.02"/>
        </geometry>
        <material name="black"/>
        </visual>
    </link>

    <!-- Joints For the Above Links-->
    <joint name="left_wheel_joint" type="continuous">
        <parent link="chassis"/>
        <child link="left_wheel"/>
        <origin xyz="-0.2 0.15 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
    </joint>

    <joint name="right_wheel_joint" type="continuous">
        <parent link="chassis"/>
        <child link="right_wheel"/>
        <origin xyz="-0.2 -0.15 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
    </joint>

    </robot>
    ```
    **Note : We are creating the file using gedit and not touch, because gedit opens the file in an editor if the file exists, or creates a new file and open that in the editor. Both way it is more efficient than touch.**

    3. Verify the file using Rviz :
        1. Go to the package folder :
        ```bash
        cd ~/Desktop/ros2_ws/src/my_robot_description/my_robot_description
        ```
        2. Create a launch Folder :
        ```bash
        mkdir launch
        cd launch
        gedit display_robot.launch.py
        ```
        3. Enter the Following code in the created file :
        ```bash
        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'robot_description': open(
                        '/home/wajahath/Desktop/ros2_ws/src/my_robot_description/my_robot_description/urdf/robot.xacro'
                    ).read()}]
                ),
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen'
                )
            ])

        ```
        4. Save and Close the file.
        5. Build the Workspace again now :
        ```bash
        cd ~/Desktop/ros2_ws
        colcon build
        source install/setup.bash
        ```
        6. Launch The Robot in RViz :
        ```bash
        ros2 launch my_robot_description display_robot.launch.py
        ```      

**HERE WE FACED AN ERROR : The World was being created and visualized but the robot was not being spawned in the world.**

- There was no robot added. We added the robot using the add button in rviz. 
     Add --> RobotModel --> ok
     In display panel --> Description_source = Topic --> Description Topic = /robot_description
- Now the robot is spawned but there are two main errors.
    1. No Tf Data. 
    2. left_wheel --> No tansform from left_wheel to chassis
        right_wheel --> No transform from right_wheel to chassis


Here is a **clean, structured, markdown-formatted summary** that fits perfectly into your notes, following the same style and flow you are already using.
I rewrote your debugging story into a clear *PHASE 3 – DEBUGGING THE ROBOT MODEL PIPELINE* section.

You can paste this directly into your notes.

---

## PHASE 3 – DEBUGGING THE ROBOT MODEL PIPELINE (DETAILED SUMMARY)

In this phase, we encountered several issues while launching the robot in RViz and resolved them step-by-step. This section documents **what went wrong**, **why it happened**, and **how we fixed it**, so the debugging workflow is easy to reference later.

---

### **ISSUE 1 — XACRO FILE NOT FOUND & XML PARSING ERROR**

### **Symptoms:**

* Launch failed with errors:

  ```
  FileNotFoundError: ...robot.xacro
  AttributeError: module 'xml' has no attribute 'parsers'
  ```
* RViz launched, but robot was not visible.

### **Root Cause:**

The URDF/XACRO files existed only inside:

```
src/my_robot_description/my_robot_description/urdf/
```

but were **not installed** into the workspace:

```
install/my_robot_description/share/my_robot_description/urdf/
```

ROS2 launch uses the **installed** path, so it could not find `robot.xacro`.

The XML error was only a secondary error caused by the missing file.

---

### **Fix Steps:**

#### **1. Verified xacro file paths**

```bash
ls src/my_robot_description/my_robot_description/urdf
ls install/my_robot_description/share/my_robot_description/urdf
```

Found: **files missing in install folder**.

---

#### **2. Inspected package metadata**

* `setup.cfg` had correct `package_data`
* BUT `setup.py` did **not** install the URDF files

This caused colcon to build the files but not install them.

---

#### **3. Edited `setup.py` to install URDF files**

Added:

```python
('share/' + package_name + '/urdf', [
    'my_robot_description/urdf/robot.xacro',
    'my_robot_description/urdf/robot.urdf',
    'my_robot_description/urdf/test_robot.xacro'
]),
```

---

#### **4. Rebuilt workspace**

```bash
colcon build --packages-select my_robot_description
source install/setup.bash
```

After rebuild, URDF appeared in install folder → **Issue 1 resolved.**

---

# ### **ISSUE 2 — RViz SHOWED ERRORS ABOUT MISSING TRANSFORMS**

### **Symptoms in RViz:**

```
No transform from left_wheel to chassis
No transform from right_wheel to chassis
No TF data
```

### **TF tree inspection:**

```bash
ros2 run tf2_tools view_frames
```

Result:

```
frame_yaml='[]'
```

This confirmed: **No TF frames were being published.**

---

### **Root Cause:**

`robot_state_publisher` only publishes transforms when:

* URDF is loaded  ✔ (we confirmed this)
* `/joint_states` topic is being published ❌ (NOT happening)

Your robot has **continuous joints**:

```xml
type="continuous"
```

→ These require joint positions from `/joint_states`.

No `joint_state_publisher` = **No TF → Robot not visible in RViz.**

---

# ### **Fix Steps:**

### **1. Checked if joint_state_publisher was running**

```bash
ros2 node list
```

Result:

```
/robot_state_publisher
/rviz
```

❌ No joint_state_publisher.

---

### **2. Verified package availability**

```bash
ros2 pkg executables joint_state_publisher
ros2 pkg executables joint_state_publisher_gui
```

Result:

```
Package ... not found
```

These packages were **not installed**.

---

### **3. Installed missing packages**

```bash
sudo apt install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui
```

---

### **4. Updated launch file to start both publishers**

Added:

```python
Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
),

Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
),
```

Full pipeline:

* `joint_state_publisher` → publishes joint angles
* `robot_state_publisher` → publishes TF tree from URDF + joint states
* `rviz2` → displays robot

---

### **5. Rebuilt & launched**

```bash
colcon build
source install/setup.bash
ros2 launch my_robot_description display_robot.launch.py
```

Ran:

```bash
ros2 node list
```

Now visible:

```
/joint_state_publisher
/joint_state_publisher_gui
```

---

### **6. Verified TF**

```bash
ros2 run tf2_tools view_frames
```

Now TF tree was populated:

```
chassis
 ├── left_wheel
 └── right_wheel
```

---

# ### **FINAL RESULT**

After fixing installation paths + launching joint_state_publisher:

* Robot loads correctly in RViz
* TF tree is fully populated
* Continuous joints update in real-time
* Wheel controller also launched successfully
* No RViz transform errors
* Entire ROS2 robot description pipeline working end-to-end

---

# ### ✔ LEARNINGS FROM THIS DEBUGGING SESSION

1. **Xacro must be installed into install/share**, not just present in src.
2. `setup.py`’s `data_files` controls installation of assets in ament_python packages.
3. `robot_state_publisher` alone is not enough — it *requires* joint states.
4. For continuous/revolute joints:

   * You MUST run joint_state_publisher (or publish manually via /joint_states).
5. TF debugging tools:

   * `ros2 param get /robot_state_publisher robot_description`
   * `ros2 run tf2_tools view_frames`
   * `ros2 topic echo /joint_states --once`
6. RViz errors almost always trace back to TF or missing robot_description.

- Now when i run the command in the terminal as 
```xml
ros2 launch my_robot_controller digital_robot.launch.py
```
it runs fine