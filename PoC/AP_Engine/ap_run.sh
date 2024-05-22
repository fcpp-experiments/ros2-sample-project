#!/bin/bash

CLEAN=${1:-false}

if [ "$CLEAN" = "true" ]; then
    echo "Cleaning..."
    ./make.sh clean
fi

echo "Compiling target ap_engine..."

# Run AP with simulator
AP_ROUND_PERIOD=0.5 AP_COMM_RANGE=100 AP_ROBOT_COUNT=3 ./make.sh gui run -O -DAP_ENGINE_DEBUG=true ap_engine
