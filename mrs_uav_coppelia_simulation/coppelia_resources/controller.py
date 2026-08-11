# CoppeliaSim child script - UAV controller
# Loaded as an embedded Python script in the CoppeliaSim scene.
# Uses the simROS2 plugin API (not rclpy directly).
# Messages are plain Python dicts matching ROS2 message structures.

import numpy as np
import math
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

    def __init__(self, sim, simROS2):

        self.sim = sim
        self.simROS2 = simROS2

        self.uav_handle = sim.getObject('.')

        self.uav_name = sim.getObjectAlias(self.uav_handle, 1).split("#")[0]

        topic_prefix = '/coppelia_simulator/' + self.uav_name + '/'

        print("topic_prefix: {}".format(topic_prefix))

        self.propellerHandles = [sim.getObject('/' + self.uav_name + '/propeller[3]/respondable'),
                                 sim.getObject('/' + self.uav_name + '/propeller[1]/respondable'),
                                 sim.getObject('/' + self.uav_name + '/propeller[0]/respondable'),
                                 sim.getObject('/' + self.uav_name + '/propeller[2]/respondable')]

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
        self.uav_mass = sim.getShapeMass(self.uav_handle) + 4 * propeller_mass

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

        self.sub_actuator_cmd = simROS2.createSubscription(
            topic_prefix + 'actuator_cmd',
            'mrs_msgs/msg/HwApiActuatorCmd',
            self.callbackActuatorCmd)

        self.sub_control_group_cmd = simROS2.createSubscription(
            topic_prefix + 'control_group_cmd',
            'mrs_msgs/msg/HwApiControlGroupCmd',
            self.callbackControlGroupCmd)

        self.sub_attitude_rate_cmd = simROS2.createSubscription(
            topic_prefix + 'attitude_rate_cmd',
            'mrs_msgs/msg/HwApiAttitudeRateCmd',
            self.callbackAttitudeRateCmd)

        self.sub_attitude_cmd = simROS2.createSubscription(
            topic_prefix + 'attitude_cmd',
            'mrs_msgs/msg/HwApiAttitudeCmd',
            self.callbackAttitudeCmd)

        self.sub_acceleration_hdg_rate_cmd = simROS2.createSubscription(
            topic_prefix + 'acceleration_hdg_rate_cmd',
            'mrs_msgs/msg/HwApiAccelerationHdgRateCmd',
            self.callbackAccelerationHdgRateCmd)

        self.sub_velocity_hdg_rate_cmd = simROS2.createSubscription(
            topic_prefix + 'velocity_hdg_rate_cmd',
            'mrs_msgs/msg/HwApiVelocityHdgRateCmd',
            self.callbackVelocityHdgRateCmd)

        ## | ----------------------- publishers ----------------------- |

        self.publisher_odom = simROS2.createPublisher(topic_prefix + 'odometry', 'nav_msgs/msg/Odometry')
        self.publisher_imu  = simROS2.createPublisher(topic_prefix + 'imu', 'sensor_msgs/msg/Imu')

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

        self.object_matrix = self.sim.getObjectMatrix(self.uav_handle, self.sim.handle_world)

        # current orientation
        self.pose, self.R = self.decodeMatrix(self.object_matrix)

        if (self.first_start):
            self.first_start = False
            self.last_R = self.R
            self.last_pose = self.pose
            return

        # calculate the angular rate
        omega = self.R.transpose().dot((self.R - self.last_R) / dt)
        self.attitude_rate = np.array([omega[2][1], omega[0][2], omega[1][0]])

        vel_world = (self.pose - self.last_pose) / dt
        self.vel_body = self.R.transpose().dot(vel_world)

        self.last_R = self.R
        self.last_pose = self.pose

    # #} end of getSensorData()

    # #{ lowLevelControl()

    def lowLevelControl(self):

        ## | ------------------------- control ------------------------ |

        if self.acceleration_hdg_rate_cmd is not None:

            des_tilt, des_hdg_rate, des_throttle = self.accelerationController(self.acceleration_hdg_rate_cmd)

            self.attitude_rate_cmd = self.tiltHdgRateController(des_tilt, des_hdg_rate, des_throttle)

        if self.attitude_cmd is not None:
            self.attitude_rate_cmd = self.attitudeController(self.attitude_cmd)

        if self.attitude_rate_cmd is not None:
            self.control_group_cmd = self.attitudeRateController(self.attitude_rate_cmd)

        if self.control_group_cmd is not None:
            self.actuator_cmd = self.mixer(self.control_group_cmd)

    # #} end of lowLevelControl()

    # #{ highLevelControl()

    def highLevelControl(self, dt):

        ## | ------------------------- control ------------------------ |

        if self.velocity_hdg_rate_cmd is not None:

            self.acceleration_hdg_rate_cmd = self.velocityController(self.velocity_hdg_rate_cmd, dt)

    # #} end of highLevelControl()

    # #{ applyForceOnPropellers()

    def applyForceOnPropellers(self, forces, torques):

        for i in range(4):

            propellerHandle = self.propellerHandles[i]

            rotDir = self.rotor_dirs[i]
            prop_force = self.R.dot(np.array([0, 0, self.force_torque_multiplier * forces[i]]))
            prop_torque = self.R.dot(np.array([0, 0, self.force_torque_multiplier * rotDir * torques[i]]))

            self.sim.addForceAndTorque(propellerHandle, prop_force.tolist(), prop_torque.tolist())

    # #} end of applyForceOnPropellers()

    # #{ applyMotorSignals()

    def applyMotorSignals(self):

        if not self.actuator_cmd:
            return

        forces = [0, 0, 0, 0]
        torques = [0, 0, 0, 0]

        motors = self.actuator_cmd['motors']

        for i in range(0, len(motors)):
            rpm = motors[i] * (self.rpm_max - self.rpm_min) + self.rpm_min
            forces[i] = self.force_constant * rpm * rpm
            torques[i] = self.torque_constant * forces[i]

        self.applyForceOnPropellers(forces, torques)

    # #} end of applyMotorSignals()

    # #{ calculateInertia()

    def calculateInertia(self):

        self.uav_inertia = np.eye(3)

        self.uav_inertia[0, 0] = self.uav_mass * (3.0 * self.arm_length * self.arm_length + self.body_height * self.body_height) / 12.0
        self.uav_inertia[1, 1] = self.uav_mass * (3.0 * self.arm_length * self.arm_length + self.body_height * self.body_height) / 12.0
        self.uav_inertia[2, 2] = (self.uav_mass * self.arm_length * self.arm_length) / 2.0

    # #} end of calculateInertia()

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

        r = R.from_matrix(self.R)
        q = r.as_quat()  # [x, y, z, w]

        t = self.simROS2.timeFromFloat(time)

        odometry_msg = {
            'header': {
                'stamp': t,
                'frame_id': 'world',
            },
            'child_frame_id': 'fcu',
            'pose': {
                'pose': {
                    'position': {
                        'x': float(self.pose[0]),
                        'y': float(self.pose[1]),
                        'z': float(self.pose[2]),
                    },
                    'orientation': {
                        'x': float(q[0]),
                        'y': float(q[1]),
                        'z': float(q[2]),
                        'w': float(q[3]),
                    },
                },
            },
            'twist': {
                'twist': {
                    'linear': {
                        'x': float(self.vel_body[0]),
                        'y': float(self.vel_body[1]),
                        'z': float(self.vel_body[2]),
                    },
                    'angular': {
                        'x': float(self.attitude_rate[0]),
                        'y': float(self.attitude_rate[1]),
                        'z': float(self.attitude_rate[2]),
                    },
                },
            },
        }

        self.simROS2.publish(self.publisher_odom, odometry_msg)

    # #} end of publishOdometry()

    # #{ publishImu()

    def publishImu(self, time):

        t = self.simROS2.timeFromFloat(time)

        imu_msg = {
            'header': {
                'stamp': t,
                'frame_id': 'fcu',
            },
            'angular_velocity': {
                'x': float(self.attitude_rate[0]),
                'y': float(self.attitude_rate[1]),
                'z': float(self.attitude_rate[2]),
            },
            'linear_acceleration': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
            },
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'w': 1.0,
            },
        }

        self.simROS2.publish(self.publisher_imu, imu_msg)

    # #} end of publishImu()

# --------------------------------------------------------------
# |                     controller methods                     |
# --------------------------------------------------------------

    # #{ velocityController()

    def velocityController(self, velocity_hdg_rate_cmd, dt):

        vel = velocity_hdg_rate_cmd['velocity']
        des_vel = np.array([vel['x'], vel['y'], vel['z']])
        cur_vel = self.R.dot(self.vel_body)

        action_x = self.pid_vel_x.control(des_vel[0], cur_vel[0], dt)
        action_y = self.pid_vel_y.control(des_vel[1], cur_vel[1], dt)
        action_z = self.pid_vel_z.control(des_vel[2], cur_vel[2], dt)

        ## | ------------------------- output ------------------------- |

        return {
            'acceleration': {'x': action_x, 'y': action_y, 'z': action_z},
            'heading_rate': velocity_hdg_rate_cmd['heading_rate'],
        }

    # #} end of velocityController()

    # #{ accelerationController()

    def accelerationController(self, acceleration_hdg_rate_cmd):

        ## | ---------------------- extract data ---------------------- |

        des_hdg_rate = acceleration_hdg_rate_cmd['heading_rate']
        des_acc = acceleration_hdg_rate_cmd['acceleration']

        ## | ------------------------- control ------------------------ |

        des_force = np.array([des_acc['x'], des_acc['y'], des_acc['z'] + 9.81]) * self.uav_mass

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

        ori = attitude_cmd['orientation']
        q = [ori['x'], ori['y'], ori['z'], ori['w']]
        des_R = R.from_quat(q).as_matrix()

        ## | ------------------------- control ------------------------ |

        R_error = 0.5 * (des_R.transpose().dot(self.R) - self.R.transpose().dot(des_R))
        R_error_vec = np.array([(R_error[1, 2] - R_error[2, 1]) / 2.0,
                                 (R_error[2, 0] - R_error[0, 2]) / 2.0,
                                 (R_error[0, 1] - R_error[1, 0]) / 2.0])

        des_rate = np.multiply(R_error_vec, self.attitude_gains)

        ## | ------------------ construct the output ------------------ |

        return {
            'body_rate': {
                'x': float(des_rate[0]),
                'y': float(des_rate[1]),
                'z': float(des_rate[2]),
            },
            'throttle': attitude_cmd['throttle'],
        }

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
        R_error_vec = np.array([R_error[1, 2] - R_error[2, 1],
                                 R_error[2, 0] - R_error[0, 2],
                                 R_error[0, 1] - R_error[1, 0]]) * 0.5

        # TODO the heading rate should be converted to the intrinsic yaw rate
        des_rate = np.multiply(R_error_vec, self.attitude_gains) + np.array([0, 0, hdg_rate])

        ## | ------------------ construct the output ------------------ |

        return {
            'body_rate': {
                'x': float(des_rate[0]),
                'y': float(des_rate[1]),
                'z': float(des_rate[2]),
            },
            'throttle': float(throttle),
        }

    # #} end of tiltHdgRateController()

    # #{ attitudeRateController()

    def attitudeRateController(self, attitude_rate_cmd):

        ## | -------------------- extract the data -------------------- |

        rate = attitude_rate_cmd['body_rate']
        des_rate = np.array([rate['x'], rate['y'], rate['z']])
        des_throttle = attitude_rate_cmd['throttle']

        ## | ------------------------- control ------------------------ |

        rate_error = des_rate - self.attitude_rate

        inertia_diag = np.diag(self.uav_inertia)

        gains = np.multiply(self.attitude_rate_gains, inertia_diag)

        action = np.multiply(rate_error, gains)

        ## | ------------------- prepare the output ------------------- |

        return {
            'roll':     float(action[0]),
            'pitch':    float(action[1]),
            'yaw':      float(action[2]),
            'throttle': float(des_throttle),
        }

    # #} end of attitudeRateController()

    # #{ mixer()

    def mixer(self, control_group_cmd):

        ## | -------------------- decode the input -------------------- |

        ctrl_groups = np.array([
            control_group_cmd['roll'],
            control_group_cmd['pitch'],
            control_group_cmd['yaw'],
            control_group_cmd['throttle'],
        ])

        motors = self.allocation_matrix.dot(ctrl_groups)

        minimum = np.min(motors)

        if minimum < 0.0:
            motors = motors + np.abs(minimum)

        maximum = np.max(motors)

        ## | ---------------------- desaturation ---------------------- |

        if maximum > 1.0:

            if ctrl_groups[3] > 0.05:
                for i in range(0, 3):
                    ctrl_groups[i] = ctrl_groups[i] / (np.mean(motors) / ctrl_groups[3])

                motors = self.allocation_matrix.dot(ctrl_groups)

            else:
                motors = motors / maximum

        ## | ------------------- prepare the output ------------------- |

        return {'motors': [float(m) for m in motors]}

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

    # #} end of callbackVelocityHdgRateCmd()

# --------------------------------------------------------------
# |                     Coppelia callbacks                     |
# --------------------------------------------------------------

# #{ sysCall_dynCallback()

def sysCall_dynCallback(data):

    dt = data['dt']
    n_passes = float(data['totalPasses'])
    multiplier = 1.0 / n_passes

    self.controller.setForceTorqueMultiplier(multiplier)

    if not data['afterStep']:

        self.controller.getSensorData(dt)

        self.controller.lowLevelControl()

    self.controller.applyMotorSignals()

# #} end of sysCall_dynCallback()

# #{ sysCall_actuation()

def sysCall_actuation():

    current_time = self.sim.getSimulationTime()
    dt = current_time - self.last_actuation_time
    self.last_actuation_time = current_time

    self.controller.highLevelControl(dt)

# #} end of sysCall_actuation()

# #{ sysCall_sensing()

def sysCall_sensing():

    t = self.sim.getSimulationTime()

    self.controller.publishImu(t)
    self.controller.publishOdometry(t)

# #} end of sysCall_sensing()

# #{ sysCall_init()

def sysCall_init():

    print("initializing controller")

    sim = require('sim')
    simROS2 = require('simROS2')

    self.sim = sim
    self.simROS2 = simROS2

    self.controller = Controller(sim, simROS2)

    self.last_actuation_time = 0

    print("controller initialized")

# #} end of sysCall_init()

# #{ sysCall_nonSimulation()

def sysCall_nonSimulation():
    pass

# #} end of sysCall_nonSimulation()

# #{ sysCall_beforeSimulation()

def sysCall_beforeSimulation():

    sim = require('sim')
    simROS2 = require('simROS2')

    self.sim = sim
    self.simROS2 = simROS2

    self.controller = Controller(sim, simROS2)

    self.last_actuation_time = 0

    # give zero-velocity command after start
    cmd = {
        'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'heading_rate': 0.0,
    }
    self.controller.setVelocityCmd(cmd)

# #} end of sysCall_beforeSimulation()

# #{ sysCall_afterSimulation()

def sysCall_afterSimulation():
    pass

# #} end of sysCall_afterSimulation()

# #{ sysCall_cleanup()

def sysCall_cleanup():
    pass

# #} end of sysCall_cleanup()
