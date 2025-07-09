
---
# 🧭 Using a Custom Planner Algorithm in TurtleBot3

# 🎥 **Demo Video**

You can view the complete demonstration here:

<video src="/home/anish/ps1_dj_ws/ps4_complete.mp4" controls autoplay loop width="640">
    Your browser does not support the video tag.
</video>

# 🚀 **Launch Commands**

```bash
# 🗺️ Launch Navigation Stack with Dijkstra Planner
ros2 launch dijkstra_planner dijkstra_navigation.launch.py use_sim_time:=True map:=src/ps1map1.yaml

# 🏠 Launch Custom Small House Gazebo World
ros2 launch turtlebot3_gazebo turtlebot3_small_house_simple.launch.py
```

---



# ⚙️ **Important Note: Configure the Behavior Tree**

## 🔍 **1️⃣ Locate the Behavior Tree XML**

First, find where the **`nav2_bt_navigator`** package is installed:

```bash
ros2 pkg prefix nav2_bt_navigator
```

Example output:

```
/opt/ros/humble
```

Go to the **Behavior Trees** folder:

```bash
cd /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/
ls
```

You should see files like:

```
navigate_to_pose_w_replanning_and_recovery.xml
```

---

## ✏️ **2️⃣ Edit the Behavior Tree**

Open the XML file in your preferred editor. Example with `nano`:

```bash
sudo nano navigate_to_pose_w_replanning_and_recovery.xml
```

Or with **VS Code**:

```bash
sudo code navigate_to_pose_w_replanning_and_recovery.xml
```

---

## 🔁 **3️⃣ Update the Planner ID**

Inside the XML, look for:

```xml
<ComputePathToPose ... planner_id="GridBased" ... />
```

✅ **Replace it with your custom Dijkstra planner ID:**

```xml
<ComputePathToPose ... planner_id="Dijkstra" ... />
```

💾 **Save** the file and **exit** the editor.

---

**📌 Done!**
Your Navigation2 Behavior Tree now uses **Dijkstra** instead of the default GridBased planner.

---

