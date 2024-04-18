# UR_SIM2REAL

## FYI

This package is used for robot mimic.

There are two UR3e robots in our scenario, one is real robot and the other is simulated robot generated by ursim. We can program in ursim's User Interface and a robot controller will publish joint states and io states in ROS2. The controller defined here will subscribe to these topics and publish them to the real robot.

## Procedures

1. launch ursim in docker

   ```bash
   # navigate to script folder
   # (optional) chmod +x run_ursim.bash
   ./run_ursim.bash
   ```

   It will first create a network (subnet `192.168.56.0`) for ursim and the simualted robot ip is `192.168.56.101`.

2. ROS2 Env: **Humble**

   Launch **simulated** robot driver in ROS2

   ```bash
   ros2 launch launch_ur_sim.launch.py
   # OR
   ros2 launch launch_ur_sim.launch.py robot_ip:=192.168.56.101 script_sender_port:=50012
   ```

> [!NOTE]
>
> Here we change default `script_sender_port` to `50012` to avoid **port conflicts** when we have two robots.
>
> we have changed 4 ports for ur_robot_driver to ` 50011` - `50014`

> [!IMPORTANT]
>
> change Host IP and Custom Port `(50012)` in External Control Setting

3. ROS2 Env: **Humble**

   Launch **real** robot driver in ROS2

   ```bash
   ros2 launch launch_ur_real.launch.py
   # OR
   ros2 launch launch_ur_real.launch.py robot_ip:=192.168.2.120
   ```

   For real robot, we load a `joint_mimic_controller` to track joints of simulated robots. 

> [!WARNING]
>
> when you activate `joint_mimic_controller`, it may cause fast tracking motion if totally different configuration of these two robots.

4. Activate `joint_mimic_controller` 

   ```bash
   # rqt -> robot tools -> controller manager
   rqt
   # OR via CLT ros2 control
   
   # activate joint_mimic_controller
   ./activate_mimic.bash
   # deactivate joint_mimic_controller
   ./deactivate_mimic.bash
   ```

## TODO:

- [x]  before activate controller, let simulated robot track real robot first!
- [x] io_mimic_controller did not work
