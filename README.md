# Coppelia Simulation

## Current state and potential TODOs

We made effords into adapting the Coppelia Sim into the MRS UAV System.
We used the pre-existing multi-rotor UAV model and attached a _customization_ python script to it.
The [python script](./coppelia_resources/controller.py) handles the low-level control and provides a ROS interface to the low-level controllers that the script implements.
Despite lot of effords, everything about this solution is slow:

- the simulator's python API is very slow (approx. 1 ms per call),
- the python itself is very slow,
- it appears the the handling of the callbacks by the simulator is also very slow.

Therefore, this solution struggles to reach realtime factor 1.0 even with a single UAV.

## Contents for this repository

- The folder `./coppelia_simulator` is dedicated to the Coppelia simulator.
- The folder `./coppelia_resources` containts the Coppelia _scene_ file, the `controller.py` for the embedded UAV controller and the `clock.py` for ROS clock publisher. These two scripts are parts of the _customization_ scripts included in the scene.
- The folder `./ros_packages` contains the [API Plugin](https://github.com/ctu-mrs/mrs_uav_hw_api) that interfaces the simulator to the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system).

## How To start

1. Download coppelia simulator from [downloads](https://www.coppeliarobotics.com/downloads) and unpack it into the `./coppelia_simulator` subfolder.
2. Build this package alongside the MRS UAV System.
3. Start `./tmux/start.sh`.

# Binding custom scripts from the scene

## UAV control script

```
#python
#luaExec additionalFuncs={'sysCall_dynCallback'}

include controller
```

## Clock publisher script

```
#python
#luaExec additionalFuncs={'sysCall_dynCallback'}

include clock
```

# Other remarks

Binding callback for the dynamics step does not work when following official tutorials.
The [tutorial](https://www.coppeliarobotics.com/helpFiles/en/dynCallbackFunctions.htm) refers to a version of API that has not been implemented (yet), or it does not work.
However, the _legacy_ API works:

```
#python
#luaExec additionalFuncs={'sysCall_dynCallback'}

def sysCall_dynCallback(inData):
    pass
```
