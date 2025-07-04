# Launch Commands

```bash
ros2 launch dijkstra_planner dijkstra_navigation.launch.py use_sim_time:=True map:=src/ps1map1.yaml
ros2 launch turtlebot3_gazebo turtlebot3_small_house_simple.launch.py
```

---

# Important Note

## 🔍 1. Locate the Behavior Tree File

Find the installation path of the `nav2_bt_navigator` package:

```bash
ros2 pkg prefix nav2_bt_navigator
```

You’ll get an output like:

```
/opt/ros/humble
```

Navigate to the behavior trees directory:

```bash
cd /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/
ls
```

You should see files such as:

```
navigate_to_pose_w_replanning_and_recovery.xml
```

---

## ✏️ 2. Edit the File

Open the file with your preferred editor. For example, using nano:

```bash
sudo nano navigate_to_pose_w_replanning_and_recovery.xml
```

Or with VS Code:

```bash
sudo code navigate_to_pose_w_replanning_and_recovery.xml
```

---

## 🔁 3. Modify the Planner ID

Find the following line:

```xml
<ComputePathToPose ... planner_id="GridBased" ... />
```

Change it to:

```xml
<ComputePathToPose ... planner_id="Dijkstra" ... />
```

Save and exit the editor.
