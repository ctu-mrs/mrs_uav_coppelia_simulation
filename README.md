# Coppelia Simulation

## Current state and potential TODOs

We tried to adapt the Coppelia Sim into the MRS UAV System.
We used the pre-existing multi-rotor UAV model and attached a _customization_ python script.
The [python script](./mrs_uav_coppelia_simulation/coppelia_resources/controller.py) handles the low-level control and provides a ROS interface to the low-level controllers that the script implements.
Despite many efforts, everything about this solution is slow:

- the simulator's python API is very slow (approx. 1 ms per call),
- the python itself is very slow,
- The handling of the callbacks by the simulator is also very slow.

Therefore, this solution struggles to reach real-time factor 1.0 even with a single UAV.

## Contents for this repository

- The folder `./mrs_uav_coppelia_simulation/coppelia_resources` contains the Coppelia _scene_ file, the `controller.py` for the embedded UAV controller, and the `clock.py` for the ROS clock publisher. These two scripts are parts of the _customization_ scripts included in the scene.
- The folder `./mrs_uav_coppelia_api` contains the [API Plugin](https://github.com/ctu-mrs/mrs_uav_hw_api) that interfaces the simulator to the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system).

## How To Start

Start the Tmuxinator session in `./mrs_uav_coppelia_simulation/tmux`.

# (Development remarks) Binding custom scripts from the scene

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

## Other remarks

The binding callback for the dynamics step does not work when following official tutorials.
The [tutorial](https://www.coppeliarobotics.com/helpFiles/en/dynCallbackFunctions.htm) refers to a version of API that has yet to be implemented (yet) or does not work.
However, the _legacy_ API works:

```
#python
#luaExec additionalFuncs={'sysCall_dynCallback'}

def sysCall_dynCallback(inData):
    pass
```
