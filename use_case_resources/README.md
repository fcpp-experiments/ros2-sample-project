# How to run
Execute commands from the root of repository.

## AP
To build:
```bash
cd ./PoC/AP_Engine
./make.sh gui build -O -DAP_COMM_RANGE=5 ap_engine
```

To run:
```bash
cd ./PoC/AP_Engine/bin   
AP_ROUND_PERIOD=0.5 AP_WEARABLE_COUNT=0 AP_ROBOT_COUNT=5 AP_SIMULATOR_OFFSET_X=0.4 ./run/ap_engine
```

## Simulation
You can use enable or disable automatic dock after reaching goal. `$DOCK` can be 1 (disable), 2 (enable).

```bash
./PoC/rumbo_library_run.sh dock_enable:=$DOCK
```

## Out of order robot
```bash
# $ROBOT_NAME can be tb3_1, tb3_2 etc...
./PoC/out_of_order.sh $ROBOT_NAME
```