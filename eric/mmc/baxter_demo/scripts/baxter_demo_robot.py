#!/usr/bin/env python3

import roboticstoolbox as rtb
import spatialmath as sm
import spatialgeometry as sg
import numpy as np
import qpsolvers as qp
from spatialmath.base.symbolic import pi
import swift

import rospy
from baxter_core_msgs.msg import JointCommand
from rospy.client import load_command_line_node_params
from sensor_msgs.msg import JointState

class BaxterDemo():

    def __init__(self):
        baxter = rtb.models.Baxter()

        # Define set of pose goals along a circular path
        centre_l = np.array(baxter.fkine(baxter.q, end=baxter.grippers[0]).A)[:3, 3]
        centre_r = np.array(baxter.fkine(baxter.q, end=baxter.grippers[1]).A)[:3, 3]

        theta = np.linspace(0, 2*np.pi, num=700)
        radius = 0.1
        circ_path = np.stack((radius*np.cos(theta), radius*np.sin(theta), np.zeros(len(theta))))

        self.goals_l = np.transpose(np.expand_dims(np.array(centre_l), 1) + circ_path)
        self.goals_r = np.transpose(np.expand_dims(np.array(centre_r), 1) + circ_path)

        self.goal_idx = 0

        rospy.init_node('baxter_demo', anonymous=False)

    def step_baxter_arm(self, baxter, which_ee, Tep):
        # Number of joint in the baxter which we are controlling
        n = 7

        # Set which gripper end effector is to be moved
        if which_ee == 'left' or which_ee == 'l':
            ee_idx = 0  # index for gripper
            idx_offset = 10   # hardcoded offset for qd index depending on which gripper is ee
        elif which_ee == 'right' or which_ee == 'r':
            ee_idx = 1
            idx_offset = 1   # hardcoded offset for qd index depending on which gripper is ee
        else:
            raise ValueError("specify right or left gripper with left/right or l/r")

        

        # location of original loop
        # The pose of the Baxter's end-effector
        Te = baxter.fkine(baxter.q, end=baxter.grippers[ee_idx])
        print("current location for", which_ee, "arm: ")
        Te.printline()


        # Transform from the end-effector to desired pose
        eTep = Te.inv() * Tep

        # Spatial error
        e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi/180]))
        print(which_ee, "arm error:", e)

        # Calulate the required end-effector spatial velocity for the robot
        # to approach the goal. Gain is set to 1.0
        v, arrived = rtb.p_servo(Te, Tep, 1.0)

        # Gain term (lambda) for control minimisation
        Y = 0.01

        # Quadratic component of objective function
        Q = np.eye(n + 6)

        # Joint velocity component of Q
        Q[:n, :n] *= Y

        # Slack component of Q
        Q[n:, n:] = (1 / e) * np.eye(6)

        # The equality contraints
        Aeq = np.c_[baxter.jacobe(baxter.q, end=baxter.grippers[ee_idx]), np.eye(6)]
        beq = v.reshape((6,))

        # The inequality constraints for joint limit avoidance
        Ain = np.zeros((n + 6, n + 6))
        bin = np.zeros(n + 6)

        # The minimum angle (in radians) in which the joint is allowed to approach
        # to its limit
        ps = 0.05

        # The influence angle (in radians) in which the velocity damper
        # becomes active
        pi = 0.9

        # Form the joint limit velocity damper
        Ain[:n, :n], bin[:n] = baxter.joint_velocity_damper(ps, pi, n)

        # Linear component of objective function: the manipulability Jacobian
        c = np.r_[-baxter.jacobm(end=baxter.grippers[ee_idx]).reshape((n,)), np.zeros(6)]


        # The lower and upper bounds on the joint velocity and slack variable
        lb = -np.r_[baxter.qdlim[idx_offset:idx_offset+n], 10 * np.ones(6)]
        ub = np.r_[baxter.qdlim[idx_offset:idx_offset+n], 10 * np.ones(6)]

        # Solve for the joint velocities dq
        qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub)
        qd = qd/30
        # Apply the joint velocities to the Baxter
        baxter.qd[idx_offset:idx_offset+n] = qd[:n]
        # print(baxter.qd.shape, qd.shape)

        return arrived, qd[:n]

    def step_baxter(self, baxter, pose_left, pose_right, dt, joint_names):
        # Initialise and start loop
        left_arrived = False
        right_arrived = False

        # # Set the desired end-effector pose (end effector frame)
        # Tep_left = baxter.fkine(baxter.q, end=baxter.grippers[0]) * sm.SE3(pose_left[0], pose_left[1], pose_left[2])
        # Tep_right = baxter.fkine(baxter.q, end=baxter.grippers[1]) * sm.SE3(pose_right[0], pose_right[1], pose_right[2])
        # # print("Tep_left: ")
        # # Tep_left.printline()
        # print(Tep_left.A)

        # Set the desired end-effector pose (base frame)
        T_left = baxter.fkine(baxter.q, end=baxter.grippers[0]).A
        T_left[0][3] = pose_left[0]
        T_left[1][3] = pose_left[1]
        T_left[2][3] = pose_left[2]

        T_right = baxter.fkine(baxter.q, end=baxter.grippers[1]).A
        T_right[0][3] = pose_right[0]
        T_right[1][3] = pose_right[1]
        T_right[2][3] = pose_right[2]

        Tep_left = sm.SE3(T_left)   # sm.SE3(pose_left[0], pose_left[1], pose_left[2])
        Tep_right = sm.SE3(T_right) # sm.SE3(pose_right[0], pose_right[1], pose_right[2])
        # print(Tep_left.A)


        l_target = sg.Sphere(0.01, color=[0.2, 0.4, 0.65, 0.5], base=Tep_left)
        #swift_env.add(l_target)
        r_target = sg.Sphere(0.01, color=[0.2, 0.4, 0.65, 0.5], base=Tep_right)
        #swift_env.add(r_target)

        while not (left_arrived and right_arrived):
            # Set velocities for each arm
            if not left_arrived:
                # Set the desired end-effector pose
                left_arrived, left_qd = self.step_baxter_arm(baxter, 'left', Tep_left)
                print('left: ', left_arrived)
            if not right_arrived:
                # Set the desired end-effector pose
                
                right_arrived, right_qd = self.step_baxter_arm(baxter, 'right', Tep_right)
                print('right: ', right_arrived)

            # Step the simulator
            #swift_env.step(dt)
            
            # Publish to baxter
            l_pub = rospy.Publisher('/robot/limb/left/joint_command', JointCommand, queue_size=10)
            r_pub = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)     

            l_command = JointCommand()
            l_command.mode = JointCommand.VELOCITY_MODE  # velocity control mode
            l_command.names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
            l_command.command = left_qd

            r_command = JointCommand()
            r_command.mode = JointCommand.VELOCITY_MODE  # velocity control mode
            r_command.names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
            r_command.command = right_qd

            print(l_command)
            print(r_command)

            l_pub.publish(l_command)
            r_pub.publish(r_command)
        

    def callback_listener(self, data):
        print(data)
        
        combined_names = '/t'.join(data.name)

        if 'head' not in combined_names:
            return
        
        joint_names = data.name
        
        # Create a Baxter robot object
        baxter = rtb.models.Baxter()
        
        # Set joint angles to read states
        # ros_to_rtb = [0, 14, 15, 12, 13, 16, 17, 18, 10, 11, 5, 6, 3, 4, 7, 8, 9, 1, 2]

        # baxter.q = [data.position[i] for i in ros_to_rtb]
        baxter.q[12] = data.position[2]
        baxter.q[13] = data.position[3]
        baxter.q[10] = data.position[4]
        baxter.q[11] = data.position[5]
        baxter.q[14] = data.position[6]
        baxter.q[15] = data.position[7]
        baxter.q[16] = data.position[8]

        baxter.q[3] = data.position[9]
        baxter.q[4] = data.position[10]
        baxter.q[1] = data.position[11]
        baxter.q[2] = data.position[12]
        baxter.q[5] = data.position[13]
        baxter.q[6] = data.position[14]
        baxter.q[7] = data.position[15]


        # ================= Circular motion

        

        dt = 0.1

        gl = self.goals_l[self.goal_idx]
        gr = self.goals_r[self.goal_idx]

        print(self.goals_l.shape)
        print(gl.shape)
        print("going to: ")
        print("left: ", gl, "right: ", gr)
        self.step_baxter(baxter, gl, gr, dt, joint_names)

        if self.goal_idx == (len(self.goals_l) - 1):
            self.goal_idx = 0
        else:
            self.goal_idx = self.goal_idx + 1





    def baxter_joints_listener(self):
        rospy.Subscriber('/robot/joint_states', JointState, self.callback_listener)

        rospy.spin()

# def baxte_joints_publisher():
#     left = rospy.Publisher('/robot/limb/left/joint_command', JointCommand, queue_size=10)

#     right = rospy.Publisher('/robot/limb/right/joint_command', JointCommand, queue_size=10)


    
    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()


# ============================= sim ===================================== #

# # Launch the simulator Swift
# env = swift.Swift()
# env.launch(realtime=False)
# env.set_camera_pose([1.8, 0, .5], [0, 0, 0])

# l_frame = sg.Axes(0.1)
# r_frame = sg.Axes(0.1)
# env.add(l_frame)
# env.add(r_frame)

# # Create a Baxter robot object
# baxter = rtb.models.Baxter()

# # Set joint angles to ready configuration
# baxter.q = baxter.qn

# # Add the Baxter to the simulator
# env.add(baxter)

# dt = 0.1

# ========================================================================= #
demo = BaxterDemo()

demo.baxter_joints_listener()

# # Define pose goals
# pose_left = [0.08, 0.44, -0.3]
# pose_right = [0.08, -0.44, 0.2]

# # Move baxter
# step_baxter(env, baxter, pose_left, pose_right, dt)

# # ================= Circular motion

# # Define set of pose goals along a circular path
# centre_l = np.array(baxter.fkine(baxter.q, end=baxter.grippers[0]).A)[:3, 3]
# centre_r = np.array(baxter.fkine(baxter.q, end=baxter.grippers[1]).A)[:3, 3]

# theta = np.linspace(0, 2*np.pi, num=50)
# radius = 0.1
# circ_path = np.stack((radius*np.cos(theta), radius*np.sin(theta), np.zeros(len(theta))))

# goals_l = np.transpose(np.expand_dims(np.array(centre_l), 1) + circ_path)
# goals_r = np.transpose(np.expand_dims(np.array(centre_r), 1) + circ_path)

# while True:
#     for gl, gr in zip(goals_l, goals_r):
#         print(goals_l.shape)
#         print(gl.shape)
#         print("going to: ")
#         print("left: ", gl, "right: ", gr)
#         step_baxter(env, baxter, gl, gr, dt)