import rospy
import rosnode

import numpy as np

from mrs_msgs.msg import HwApiActuatorCmd as HwApiActuatorCmd
from mrs_msgs.msg import HwApiControlGroupCmd as HwApiControlGroupCmd
from mrs_msgs.msg import HwApiAttitudeRateCmd as HwApiAttitudeRateCmd
from mrs_msgs.msg import HwApiAttitudeCmd as HwApiAttitudeCmd
from mrs_msgs.msg import HwApiAccelerationHdgRateCmd as HwApiAccelerationHdgRateCmd
from mrs_msgs.msg import HwApiVelocityHdgRateCmd as HwApiVelocityHdgRateCmd
from sensor_msgs.msg import Imu as Imu
from nav_msgs.msg import Odometry as Odometry

from tf.transformations import quaternion_matrix
from scipy.spatial.transform import Rotation as R

# #{ PID

class PID:

    def __init__(self, kp, ki, kd):

        self.last_error = 0
        self.integral = 0

        self.kp = kp
        self.ki = ki
        self.kd = kd

    def control(self, reference, state, dt):

        error = reference - state

        self.integral = self.integral + error * dt

        difference = 0

        if dt > 0:

            difference = (error - self.last_error) / dt

            self.last_error = error

        return self.kp * error + self.kd * difference + self.ki * self.integral

# #} end of PID

class Controller:

    # #{ init()

    def __init__(self):

        rospy.init_node('coppelia_wrapper', anonymous=True)

        self.uav_handle = sim.getObject('.')

        self.uav_name = sim.getObjectName(self.uav_handle).split("#")[0]

        topic_prefix = '/coppelia_simulator/'+self.uav_name+'/'

        print("topic_prefix: {}".format(topic_prefix))

        self.propellerHandles = [sim.getObject('/'+self.uav_name+'/propeller[3]/respondable'),
                                 sim.getObject('/'+self.uav_name+'/propeller[1]/respondable'),
                                 sim.getObject('/'+self.uav_name+'/propeller[0]/respondable'),
                                 sim.getObject('/'+self.uav_name+'/propeller[2]/respondable')]

        self.attitude_rate = [0.0, 0.0, 0.0]
        self.vel_body = np.array([0, 0, 0])
        self.R = np.eye(3)
        self.pose = np.array([0, 0, 0])

        self.motor_throttles = np.array([0.0, 0.0, 0.0, 0.0])

        self.rotor_dirs = [-1.0, -1.0, +1.0, +1.0]

        self.actuator_cmd              = None
        self.control_group_cmd         = None
        self.attitude_rate_cmd         = None
        self.attitude_cmd              = None
        self.acceleration_hdg_rate_cmd = None
        self.velocity_hdg_rate_cmd     = None

        self.first_start = True

        ## | -------------------------- gains ------------------------- |

        self.attitude_gains = np.array([8.0, 8.0, 8.0])
        self.attitude_rate_gains = np.array([4.0, 4.0, 4.0])

        ## | --------------------------- PID -------------------------- |

        self.pid_vel_x = PID(1.0, 0.0, 0.1)
        self.pid_vel_y = PID(1.0, 0.0, 0.1)
        self.pid_vel_z = PID(1.0, 0.0, 0.1)

        ## | ----------------------- uav params ----------------------- |

        self.arm_length = 0.5
        self.body_height = 0.1

        ## | ------------------------ dynamics ------------------------ |

        propeller_mass = sim.getShapeMass(self.propellerHandles[0])
        self.uav_mass = sim.getShapeMass(self.uav_handle) + 4*propeller_mass

        print("self.uav_mass: {}".format(self.uav_mass))

        self.calculateInertia()

        ## | -------------------------- mixer ------------------------- |

        self.allocation_matrix = np.array(
            [[-0.70711, -0.70711, -1, 1],
            [0.70711,   0.70711,  -1, 1],
            [0.70711,   -0.70711, 1,  1],
            [-0.70711,  0.70711,  1,  1]]
        )

        ## | ----------------------- propulsion ----------------------- |

        self.force_constant = 0.0000049000
        self.torque_constant = 0.07
        self.rpm_min = 0
        self.rpm_max = 1000

        ## | --------------- initialize propeller poses --------------- |

        self.getSensorData(0.01)

        ## | ----------------------- subscribers ---------------------- |

        self.sub_actuator_cmd              = rospy.Subscriber(topic_prefix+'actuator_cmd', HwApiActuatorCmd, self.callbackActuatorCmd, queue_size                                 = 1)
        self.sub_control_group_cmd         = rospy.Subscriber(topic_prefix+'control_group_cmd', HwApiControlGroupCmd, self.callbackControlGroupCmd, queue_size                        = 1)
        self.sub_attitude_rate_cmd         = rospy.Subscriber(topic_prefix+'attitude_rate_cmd', HwApiAttitudeRateCmd, self.callbackAttitudeRateCmd, queue_size                    = 1)
        self.sub_attitude_cmd              = rospy.Subscriber(topic_prefix+'attitude_cmd', HwApiAttitudeCmd, self.callbackAttitudeCmd, queue_size                                 = 1)
        self.sub_acceleration_hdg_rate_cmd = rospy.Subscriber(topic_prefix+'acceleration_hdg_rate_cmd', HwApiAccelerationHdgRateCmd, self.callbackAccelerationHdgRateCmd, queue_size = 1)
        self.sub_velocity_hdg_rate_cmd = rospy.Subscriber(topic_prefix+'velocity_hdg_rate_cmd', HwApiVelocityHdgRateCmd, self.callbackVelocityHdgRateCmd, queue_size = 1)

        ## | ----------------------- publishers ----------------------- |

        self.publisher_odom = rospy.Publisher(topic_prefix+'odometry', Odometry, queue_size=1)
        self.publisher_imu = rospy.Publisher(topic_prefix+'imu', Imu, queue_size=1)

    # #} end of init()

    # #{ decodeMatrix()

    def decodeMatrix(self, mat):

        R = np.array([[mat[0], mat[1], mat[2]],
                      [mat[4], mat[5], mat[6]],
                      [mat[8], mat[9], mat[10]]])

        pose = np.array([mat[3], mat[7], mat[11]])

        return pose, R

    # #} end of decodeMatrix()

    # #{ getSensorData()

    def getSensorData(self, dt):

        self.object_matrix = sim.getObjectMatrix(self.uav_handle, sim.handle_world)

        # current orientation
        self.pose, self.R = self.decodeMatrix(self.object_matrix)

        if (self.first_start):
            self.first_start = False
            self.last_R = self.R
            self.last_pose = self.pose
            return

        # calculate the angular rate
        omega = self.R.transpose().dot((self.R-self.last_R)/dt)
        self.attitude_rate=np.array([omega[2][1], omega[0][2], omega[1][0]])

        vel_world = (self.pose - self.last_pose)/dt
        self.vel_body = self.R.transpose().dot(vel_world)

        self.last_R = self.R
        self.last_pose = self.pose

        ## | --------------------- propeller poses -------------------- |

    # #} end of calculateVelocities()

    # #{ lowLevelControl()

    def lowLevelControl(self):

        ## | ------------------------- control ------------------------ |

        if self.acceleration_hdg_rate_cmd != None:

            des_tilt, des_hdg_rate, des_throttle = self.accelerationController(self.acceleration_hdg_rate_cmd)

            self.attitude_rate_cmd = self.tiltHdgRateController(des_tilt, des_hdg_rate, des_throttle)

        if self.attitude_cmd != None:
            self.attitude_rate_cmd = self.attitudeController(self.attitude_cmd)

        if self.attitude_rate_cmd != None:
            self.control_group_cmd = self.attitudeRateController(self.attitude_rate_cmd)

        if self.control_group_cmd != None:
            self.actuator_cmd = self.mixer(self.control_group_cmd)

    # #} end of lowLevelControl()

    # #{ highLevelControl()

    def highLevelControl(self, dt):

        ## | ------------------------- control ------------------------ |

        if self.velocity_hdg_rate_cmd != None:

            self.acceleration_hdg_rate_cmd = self.velocityController(self.velocity_hdg_rate_cmd, dt)

    # #} end of highLevelControl()

    # #{ applyForceOnPropellers()

    def applyForceOnPropellers(self, forces, torques):

        for i in range(4):

            propellerHandle = self.propellerHandles[i]

            rotDir = self.rotor_dirs[i]
            prop_force = self.R.dot(np.array([0, 0, self.force_torque_multiplier*forces[i]]))
            prop_torque = self.R.dot(np.array([0, 0, self.force_torque_multiplier*rotDir*torques[i]]))

            sim.addForceAndTorque(propellerHandle, prop_force.tolist(), prop_torque.tolist())

    # #} end of applyForceOnPropellers()

    # #{ applyMotorSignals()

    def applyMotorSignals(self):

        if not self.actuator_cmd:
            return

        forces = [0, 0, 0, 0]
        torques = [0, 0, 0, 0]

        for i in range(0, len(self.actuator_cmd.motors)):
            rpm = self.actuator_cmd.motors[i]*(self.rpm_max - self.rpm_min) + self.rpm_min
            forces[i] = self.force_constant*rpm*rpm
            torques[i] = self.torque_constant*forces[i]

        total_g = np.sum(forces)/9.81

        self.applyForceOnPropellers(forces, torques)

    # #} end of applyMotorSignals()

    # #{ calculateInertia()

    def calculateInertia(self):

        self.uav_inertia = np.eye(3)

        self.uav_inertia[0, 0] = self.uav_mass * (3.0 * self.arm_length * self.arm_length + self.body_height * self.body_height) / 12.0;
        self.uav_inertia[1, 1] = self.uav_mass * (3.0 * self.arm_length * self.arm_length + self.body_height * self.body_height) / 12.0;
        self.uav_inertia[2, 2] = (self.uav_mass * self.arm_length * self.arm_length) / 2.0;

    # #} end of getInertia()

    # #{ setAccelerationCmd()

    def setAccelerationCmd(self, cmd):

        self.acceleration_hdg_rate_cmd = cmd

    # #} end of setAccelerationCmd

    # #{ setVelocityCmd()

    def setVelocityCmd(self, cmd):

        self.velocity_hdg_rate_cmd = cmd

    # #} end of setVelocityCmd

    # #{ setForceTorqueMultiplier()

    def setForceTorqueMultiplier(self, multiplier):

        self.force_torque_multiplier = multiplier

    # #} end of setForceTorqueMultiplier()

    # #{ publishOdometry()

    def publishOdometry(self, time):

        ## Assemble odometry_msg
        odometry_msg = Odometry()

        # odometry_msg.header.stamp = rospy.Time.now()
        odometry_msg.header.stamp = rospy.Time.from_sec(time)
        odometry_msg.header.frame_id = "world"
        odometry_msg.child_frame_id = "fcu"

        odometry_msg.pose.pose.position.x = self.pose[0]
        odometry_msg.pose.pose.position.y = self.pose[1]
        odometry_msg.pose.pose.position.z = self.pose[2]

        r = R.from_matrix(self.R)
        q = r.as_quat()

        odometry_msg.pose.pose.orientation.x = q[0]
        odometry_msg.pose.pose.orientation.y = q[1]
        odometry_msg.pose.pose.orientation.z = q[2]
        odometry_msg.pose.pose.orientation.w = q[3]

        odometry_msg.twist.twist.angular.x = self.attitude_rate[0]
        odometry_msg.twist.twist.angular.y = self.attitude_rate[1]
        odometry_msg.twist.twist.angular.z = self.attitude_rate[2]

        odometry_msg.twist.twist.linear.x = self.vel_body[0]
        odometry_msg.twist.twist.linear.y = self.vel_body[1]
        odometry_msg.twist.twist.linear.z = self.vel_body[2]

        self.publisher_odom.publish(odometry_msg)

    # #} end of publishOdometry()

    # #{ publishImu()

    def publishImu(self, time):

        ## Assemble odometry_msg
        imu_msg = Imu()

        # odometry_msg.header.stamp = rospy.Time.now()
        imu_msg.header.stamp = rospy.Time.from_sec(time)
        imu_msg.header.frame_id = "fcu"

        imu_msg.angular_velocity.x = self.attitude_rate[0]
        imu_msg.angular_velocity.y = self.attitude_rate[1]
        imu_msg.angular_velocity.z = self.attitude_rate[2]

        self.publisher_imu.publish(imu_msg)

    # #} end of publishOdometry()

# --------------------------------------------------------------
# |                     controller methods                     |
# --------------------------------------------------------------

    # #{ velocityController()

    def velocityController(self, velocity_hdg_rate_cmd, dt):

        des_vel = np.array([velocity_hdg_rate_cmd.velocity.x, velocity_hdg_rate_cmd.velocity.y, velocity_hdg_rate_cmd.velocity.z])
        cur_vel = self.R.dot(self.vel_body)

        action_x = self.pid_vel_x.control(des_vel[0], cur_vel[0], dt)
        action_y = self.pid_vel_y.control(des_vel[1], cur_vel[1], dt)
        action_z = self.pid_vel_z.control(des_vel[2], cur_vel[2], dt)

        ## | ------------------------- output ------------------------- |

        accceleration_hdg_rate_cmd = HwApiAccelerationHdgRateCmd()

        accceleration_hdg_rate_cmd.acceleration.x = action_x
        accceleration_hdg_rate_cmd.acceleration.y = action_y
        accceleration_hdg_rate_cmd.acceleration.z = action_z

        accceleration_hdg_rate_cmd.heading_rate = velocity_hdg_rate_cmd.heading_rate

        return accceleration_hdg_rate_cmd

    # #} end of velocityController()

    # #{ accelerationController()

    def accelerationController(self, acceleration_hdg_rate_cmd):

        ## | ---------------------- extract data ---------------------- |

        des_hdg_rate = acceleration_hdg_rate_cmd.heading_rate
        des_acc = acceleration_hdg_rate_cmd.acceleration

        ## | ------------------------- control ------------------------ |

        des_force = np.array([des_acc.x, des_acc.y, des_acc.z + 9.81])*self.uav_mass

        des_F_scalar = des_force.dot(self.R[:, 2]) / 4.0

        if des_F_scalar < 0:
            des_F_scalar = 0

        des_rpm = math.sqrt(des_F_scalar / self.force_constant)

        des_throttle = (des_rpm - self.rpm_min) / (self.rpm_max - self.rpm_min)

        ## | ------------------ construct the output ------------------ |

        return des_force, des_hdg_rate, des_throttle

    # #} end of accelerationController()

    # #{ so3Transform()

    def so3Transform(self, body_z, heading):

        # desired heading vec
        des_heading_vec = np.array([math.cos(heading),
                                    math.sin(heading),
                                    0])

        body_z = body_z / np.linalg.norm(body_z)

        # desired orientation matrix
        des_R = np.eye(3)

        des_R[:, 2] = body_z
        des_R[:, 1] = np.cross(des_R[:, 2], des_heading_vec)
        des_R[:, 1] = des_R[:, 1] / np.linalg.norm(des_R[:, 1])
        des_R[:, 0] = np.cross(des_R[:, 1], des_R[:, 2])
        des_R[:, 0] = des_R[:, 0] / np.linalg.norm(des_R[:, 0])

        return des_R

    # #} end of so3Transform()

    # #{ attitudeController()

    def attitudeController(self, attitude_cmd):

        ## | -------------------- extract the data -------------------- |

        des_R = quaternion_matrix([attitude_cmd.orientation.x, attitude_cmd.orientation.y, attitude_cmd.orientation.z, attitude_cmd.orientation.w])[0:3,0:3]

        ## | ------------------------- control ------------------------ |

        R_error = 0.5 * (des_R.transpose().dot(self.R) - self.R.transpose().dot(des_R))
        R_error_vec = np.array([(R_error[1, 2]-R_error[2, 1])/2.0, (R_error[2, 0]-R_error[0, 2])/2.0, (R_error[0, 1]-R_error[1, 0])/2.0])

        des_rate = np.multiply(R_error_vec, self.attitude_gains)

        ## | ------------------ construct the output ------------------ |

        attitude_rate_cmd = HwApiAttitudeRateCmd()

        attitude_rate_cmd.body_rate.x = des_rate[0]
        attitude_rate_cmd.body_rate.y = des_rate[1]
        attitude_rate_cmd.body_rate.z = des_rate[2]

        attitude_rate_cmd.throttle = attitude_cmd.throttle

        return attitude_rate_cmd

    # #} end of attitudeController()

    # #{ tiltHdgRateController()

    def tiltHdgRateController(self, tilt, hdg_rate, throttle):

        ## | -------------------- extract the data -------------------- |

        try:
            cur_heading = math.atan2(self.R[1, 0], self.R[0, 0])
        except:
            cur_heading = 0
            pass

        des_R = self.so3Transform(tilt, cur_heading)

        ## | ------------------------- control ------------------------ |

        R_error = 0.5 * (des_R.transpose().dot(self.R) - self.R.transpose().dot(des_R))
        R_error_vec = np.array([R_error[1, 2]-R_error[2, 1], R_error[2, 0]-R_error[0, 2], R_error[0, 1]-R_error[1, 0]]) * 0.5

        # TODO the heading rate should be converted to the intrinsic yaw rate
        des_rate = np.multiply(R_error_vec, self.attitude_gains) + np.array([0, 0, hdg_rate])

        ## | ------------------ construct the output ------------------ |

        attitude_rate_cmd = HwApiAttitudeRateCmd()

        attitude_rate_cmd.body_rate.x = des_rate[0]
        attitude_rate_cmd.body_rate.y = des_rate[1]
        attitude_rate_cmd.body_rate.z = des_rate[2]

        attitude_rate_cmd.throttle = throttle

        return attitude_rate_cmd

    # #} end of tiltHdgRateController()

    # #{ attitudeRateController()

    def attitudeRateController(self, attitude_rate_cmd):

        ## | -------------------- extract the data -------------------- |

        des_rate = np.array([attitude_rate_cmd.body_rate.x, attitude_rate_cmd.body_rate.y, attitude_rate_cmd.body_rate.z])
        des_throttle = attitude_rate_cmd.throttle

        ## | ------------------------- control ------------------------ |

        rate_error = des_rate - self.attitude_rate

        inertia_diag = np.diag(self.uav_inertia)

        gains = np.multiply(self.attitude_rate_gains, inertia_diag)

        action = np.multiply(rate_error, gains)

        ## | ------------------- prepare the output ------------------- |

        control_group_cmd = HwApiControlGroupCmd()

        control_group_cmd.roll     = action[0]
        control_group_cmd.pitch    = action[1]
        control_group_cmd.yaw      = action[2]
        control_group_cmd.throttle = des_throttle

        return control_group_cmd

    # #} end of rateController()

    # #{ mixer()

    def mixer(self, control_group_cmd):

        ## | -------------------- decode the input -------------------- |

        ctrl_groups = np.array([control_group_cmd.roll, control_group_cmd.pitch, control_group_cmd.yaw, control_group_cmd.throttle])

        motors = self.allocation_matrix.dot(ctrl_groups)

        minimum = np.min(motors)

        if minimum < 0.0:
            motors = motors + np.abs(minimum)

        maximum = np.max(motors)

        ## | ---------------------- desaturation ---------------------- |

        if maximum > 1.0:

            if ctrl_groups[3] > 0.05:
                for i in range(0, 3):
                    ctrl_groups[i] = ctrl_groups[i] / (np.mean(motors) / ctrl_groups[3]);

                motors = self.allocation_matrix.dot(ctrl_groups)

            else:
                motors = motors / maximum

        ## | ------------------- prepare the output ------------------- |

        actuators = HwApiActuatorCmd()

        for i in range(0, len(motors)):
            actuators.motors.append(motors[i])

        return actuators

    # #} end of mixer()

# --------------------------------------------------------------
# |                        ROS callbacks                       |
# --------------------------------------------------------------

    # #{ callbackActuatorCmd()

    def callbackActuatorCmd(self, msg):

        self.actuator_cmd              = msg
        self.control_group_cmd         = None
        self.attitude_rate_cmd         = None
        self.attitude_cmd              = None
        self.acceleration_hdg_rate_cmd = None
        self.velocity_hdg_rate_cmd     = None

    # #} end of callbackActuatorCmd()

    # #{ callbackControlGroupCmd()

    def callbackControlGroupCmd(self, msg):

        self.actuator_cmd              = None
        self.control_group_cmd         = msg
        self.attitude_rate_cmd         = None
        self.attitude_cmd              = None
        self.acceleration_hdg_rate_cmd = None
        self.velocity_hdg_rate_cmd     = None

    # #} end of callbackControlGroupCmd()

    # #{ callbackAttitudeRateCmd()

    def callbackAttitudeRateCmd(self, msg):

        self.actuator_cmd              = None
        self.control_group_cmd         = None
        self.attitude_rate_cmd         = msg
        self.attitude_cmd              = None
        self.acceleration_hdg_rate_cmd = None
        self.velocity_hdg_rate_cmd     = None

    # #} end of callbackAttitudeRateCmd()

    # #{ callbackAttitudeCmd()

    def callbackAttitudeCmd(self, msg):

        self.actuator_cmd              = None
        self.control_group_cmd         = None
        self.attitude_rate_cmd         = None
        self.attitude_cmd              = msg
        self.acceleration_hdg_rate_cmd = None
        self.velocity_hdg_rate_cmd     = None

    # #} end of callbackAttitudeCmd()

    # #{ callbackAccelerationHdgRateCmd()

    def callbackAccelerationHdgRateCmd(self, msg):

        self.actuator_cmd              = None
        self.control_group_cmd         = None
        self.attitude_rate_cmd         = None
        self.attitude_cmd              = None
        self.acceleration_hdg_rate_cmd = msg
        self.velocity_hdg_rate_cmd     = None

    # #} end of callbackAccelerationHdgRateCmd()

    # #{ callbackVelocityHdgRateCmd()

    def callbackVelocityHdgRateCmd(self, msg):

        self.actuator_cmd              = None
        self.control_group_cmd         = None
        self.attitude_rate_cmd         = None
        self.attitude_cmd              = None
        # self.acceleration_hdg_rate_cmd = None
        self.velocity_hdg_rate_cmd     = msg

    # #} end of callbackAccelerationHdgRateCmd()

# --------------------------------------------------------------
# |                     Coppelia callbacks                     |
# --------------------------------------------------------------

# #{ syscall_dynCallback()

def sysCall_dynCallback(data):

    global controller

    dt = data['dt']
    n_passes = float(data['totalPasses'])
    multiplier = 1.0 / n_passes

    controller.setForceTorqueMultiplier(multiplier)

    if not data['afterStep']:

      controller.getSensorData(dt)

      controller.lowLevelControl()

    controller.applyMotorSignals()

# #} end of syscall_dynCallback()

# #{ sysCall_actuation()

def sysCall_actuation():

    global controller
    global last_actuation_time

    current_time = sim.getSimulationTime()
    dt = current_time - last_actuation_time
    last_actuation_time = current_time

    controller.highLevelControl(dt)

# #} end of sysCall_actuation()

# #{ sysCall_sensing()

def sysCall_sensing():

    global controller

    t = sim.getSimulationTime()

    controller.publishImu(t)
    controller.publishOdometry(t)

# #} end of sysCall_sensing()

# #{ sysCall_init()

def sysCall_init():

    print("initializing controller")

    global controller
    controller = Controller()

    global last_actuation_time
    last_actuation_time = 0

    print("controller initialized")

# #} end of sysCall_init()

# #{ sysCall_nonSimulation()

def sysCall_nonSimulation():
    pass

# #} end of sysCall_nonSimulation()

# #{ sysCall_beforeSimulation()

def sysCall_beforeSimulation():

    global controller
    controller = Controller()

    global last_actuation_time
    last_actuation_time = 0

    # give zero-velocity command after start
    cmd = HwApiVelocityHdgRateCmd()
    cmd.velocity.x = 0.0
    cmd.velocity.y = 0.0
    cmd.velocity.z = 0.0
    cmd.heading_rate = 0.0
    controller.setVelocityCmd(cmd)

# #} end of sysCall_beforeSimulation()

# #{ sysCall_afterSimulation()

def sysCall_afterSimulation():
    pass

# #} end of sysCall_afterSimulation()

# #{ sysCall_cleanup()

def sysCall_cleanup():
    pass

# #} end of sysCall_cleanup()
