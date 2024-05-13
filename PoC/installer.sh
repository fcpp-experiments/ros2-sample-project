#!/bin/bash

LOG_PREFIX="[INSTALLER]"

POC_FOLDER="./PoC"

log() {
    echo "$LOG_PREFIX $@"
}

log "Starting installation script..."

if [ -d "$POC_FOLDER" ]; then
    WORKING_DIR="$POC_FOLDER"
else
    WORKING_DIR="."
fi

cd "$WORKING_DIR" || exit

log "Use $WORKING_DIR as working directory"

compile_package() {
    local component="$1"

    log "Searching for component: $component"

    # Find component case-insensitively
    local component_dir
    component_dir=$(find -L . -maxdepth 1 -type d -iname "*$component*" | head -n 1)

    if [[ -z "$component_dir" ]]; then
        log "Component $component not found."
        return
    fi

    # Resolve symbolic links
    component_dir=$(readlink -f "$component_dir")

    log "Component directory found: $component_dir"

    # Convert component name to lower case
    local component_lower
    component_lower=$(echo "$component" | tr '[:upper:]' '[:lower:]')

    # Save current directory
    local current_dir
    current_dir=$(pwd)

    cd "$component_dir" || exit

    case "$(basename "$component_lower")" in
        *"ap_engine"*)
            log "Updating AP_Engine..."
            ./make.sh clean
            log "Compiling target ap_engine..."
            ./make.sh gui build -O ap_engine
            ;;        
        *)
            # Update with standard logic
            log "Updating ROS2 component: $component_dir"
            install_ros2_component
            ;;
    esac

    log "Installed component successfully: $component_lower"

    # Go back to the parent directory
    cd "$current_dir" || exit
}

install_ros2_component() {
    log "Clearing build directories..."
    rm -rf build/ log/ install/
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y -i --os="$OS"
    log "Build..."
    colcon build --symlink-install
}

compile_components() {
    local components=("$@")

    for component in "${components[@]}"; do
        compile_package "$component"
    done
}

components=(
    "Navigation_System"
    "Robot_Reader"
    "Robot_Writer"
    "Gazebo_Rumbo"
    "Battery"
    "AP_Engine"
)

# Update submodules
log "Updating submodules..."
git submodule update --init --recursive
log "Submodules updated."

# Initialize rosdep only if it hasn't been initialized before
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    log "Initializing rosdep..."
    sudo rosdep init
    log "Rosdep initialized."
fi

# Declare the OS used
OS="ubuntu:jammy"
log "Operating system: $OS"

compile_components "${components[@]}"