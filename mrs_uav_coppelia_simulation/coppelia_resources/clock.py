# CoppeliaSim child script - clock publisher
# Publishes /clock using the simROS2 plugin API.
# Loaded as an embedded script in the CoppeliaSim scene.

def sysCall_init():

    sim = require('sim')
    simROS2 = require('simROS2')

    self.sim = sim
    self.simROS2 = simROS2

    self.publisher_clock = simROS2.createPublisher('/clock', 'rosgraph_msgs/msg/Clock')

def sysCall_dynCallback(data):

    if data['afterStep']:
        return

    dt = float(data['dt'])
    passes = float(data['passCnt'])

    cur_time = self.sim.getSimulationTime() + passes * dt

    t = self.simROS2.timeFromFloat(cur_time)

    self.simROS2.publish(self.publisher_clock, {'clock': t})

def sysCall_actuation():
    pass

def sysCall_sensing():
    pass

def sysCall_nonSimulation():
    pass

def sysCall_beforeSimulation():
    pass

def sysCall_afterSimulation():
    pass

def sysCall_cleanup():
    self.simROS2.shutdownPublisher(self.publisher_clock)
