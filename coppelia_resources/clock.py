import rospy
import rosnode

from rosgraph_msgs.msg import Clock as Clock

def sysCall_dynCallback(data):

    if data['afterStep']:
        return

    global publisher_clock
    global last_time

    dt = float(data['dt'])
    passes = float(data['passCnt'])

    cur_time = sim.getSimulationTime() + passes*dt

    sim_time = Clock()
    sim_time.clock = rospy.Time.from_sec(cur_time)
    publisher_clock.publish(sim_time)

    last_time = cur_time

def sysCall_actuation():
    pass

def sysCall_sensing():
    pass

def sysCall_init():

    global publisher_clock
    global last_time

    last_time = 0

    rospy.init_node('coppelia_manager', anonymous=True)

    publisher_clock = rospy.Publisher('/clock', Clock, queue_size=1)

def sysCall_nonSimulation():
    pass

def sysCall_beforeSimulation():
    pass

def sysCall_afterSimulation():
    pass

def sysCall_cleanup():
    pass
