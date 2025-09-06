#!/bin/bash

# This script sets the mode of the robot by publishing to the /set_mode topic.
# Usage: ./mode_set.sh <mode>

MODE=$1 # Store the first argument as MODE

if [ -z "$MODE" ]; then # Check if MODE is empty. -z means: "Is this string empty?"
  echo "Usage: ./mode_set.sh [standby|mapping|delivery|manual]"
  exit 1
fi

echo "Setting mode to $MODE"

# Kill any previous mode launch
pkill -f delivery.launch.py
pkill -f mapping.launch.py
pkill -f manual.launch.py

# Standby stays alive always, only launch new mode
case $MODE in
  mapping)
    echo "Launching mapping mode..."
    ros2 launch msds_modes mapping.launch.py & # Run in background
    ;;
  delivery)
    echo "Launching delivery mode..."
    ros2 launch msds_modes delivery.launch.py &
    ;;
  manual)
    echo "Launching manual mode..."
    ros2 launch msds_modes manual.launch.py &
    ;;
  standby)
    echo "Switched to standby mode (other modes stopped)"
    ;;
  *) # * is a wildcard that matches anything else i.e default case
    echo "$MODE is an invalid mode. Use standby, mapping, delivery, or manual."
    exit 1
    ;;
esac

# chmod +x mode_set.sh
# ./mode_set.sh mapping
# ./mode_set.sh delivery
# ./mode_set.sh manual
# ./mode_set.sh standby