#!/bin/bash

if [ -f "lib_runner.sh" ]; then
    source "lib_runner.sh"
elif [ -f "PoC/lib_runner.sh" ]; then
    source "PoC/lib_runner.sh"
else
    echo "Error: lib_runner.sh not found."
    exit 1
fi

clean_storage_folder

components=(
    "Navigation_System"
    "Robot_Reader"
    "Robot_Writer"
    "Gazebo_Rumbo"
    "Battery"
)

initialize_components "${components[@]}"

world_file="use_case_resources/library/library.sdf"

GAZEBO_MODEL_PATH=$(pwd) \
ROBOTS_YAML=use_case_resources/library/robots.yaml \
ros2 launch rumbo_gazebo rumbo_library.launch.py use_proxies:=false \
world:="$world_file" map:=use_case_resources/library/library_map.yaml \
"$@"

