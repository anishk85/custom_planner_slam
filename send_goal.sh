#!/bin/bash
# filepath: /home/anish/ps1_ws/send_goal.sh

# Function to send goal
send_goal() {
    local x=$1
    local y=$2
    local name=$3
    
    echo "🎯 Sending goal: $name at ($x, $y)"
    
    ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: 'map'
pose:
  position: {x: $x, y: $y, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"
}

# Predefined room goals
kitchen() {
    send_goal 3.0 1.0 "Kitchen"
}

bedroom() {
    send_goal 3.0 -2.0 "Bedroom"
}

bathroom() {
    send_goal -1.0 -2.0 "Bathroom"
}

living_room() {
    send_goal 0.0 0.0 "Living Room"
}

hallway() {
    send_goal 1.0 -1.0 "Hallway"
}

# Custom goal function
custom_goal() {
    if [ $# -eq 2 ]; then
        send_goal $1 $2 "Custom Location"
    else
        echo "Usage: custom_goal <x> <y>"
        echo "Example: custom_goal 2.5 1.5"
    fi
}

# Show available commands
help() {
    echo "🏠 Available room commands:"
    echo "  kitchen     - Go to kitchen"
    echo "  bedroom     - Go to bedroom"
    echo "  bathroom    - Go to bathroom"
    echo "  living_room - Go to living room"
    echo "  hallway     - Go to hallway"
    echo "  custom_goal <x> <y> - Go to custom coordinates"
    echo "  help        - Show this help"
}

# Execute command based on argument
case "$1" in
    kitchen)     kitchen ;;
    bedroom)     bedroom ;;
    bathroom)    bathroom ;;
    living_room) living_room ;;
    hallway)     hallway ;;
    custom_goal) custom_goal $2 $3 ;;
    help|*)      help ;;
esac
