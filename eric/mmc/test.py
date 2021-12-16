from webbrowser import Chrome
import swift
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np

# robot = rtb.models.DH.Panda()
# T = SE3(0.7, 0.2, 0.1) * SE3.OA([0, 1, 0], [0, 0, -1])
# sol = robot.ikine_LM(T)
# q_pickup = sol.q
# qt = rtb.jtraj(robot.qz, q_pickup, 50)
# robot.plot(qt.q, movie='panda1.gif')

# robot = rtb.models.URDF.Panda()
# env = swift.Swift()
# env.launch()
# env.add(robot)          # add robot to the 3D scene
# for qk in qt.q:             # for each joint configuration on trajectory
#       robot.q = qk          # update the robot state
#       env.step()        # update visualization

# robot = rtb.models.UR10()
# print(robot.qdlim)




# baxter tests
robot = rtb.models.URDF.Baxter()

env = swift.Swift()
env.launch(realtime=True)
env.set_camera_pose([2, 0, 0], [0, 0, 0])
env.add(robot)

t = np.arange(10)
N = 100

traj = rtb.jtraj(robot.q, robot.qn, N)
print(traj.q.shape)


for q in traj.q:
    robot.q = q
    env.step()

print(robot.q)
print(len(robot.q))

fk_left = robot.fkine(robot.q, end='left_gripper_base')

print(fk_left)


# while True:
#     pass



# ========================================================================== #

# # Launch the simulator Swift
# env = swift.Swift()
# env.launch(realtime=True)
# env.set_camera_pose([1.8, 0, .5], [0, 0, 0])

# # Create a Baxter robot object
# baxter = rtb.models.Baxter()

# # Set joint angles to ready configuration
# baxter.q = baxter.qn

# # Add the Baxter to the simulator
# env.add(baxter)

# # Number of joint in the baxter which we are controlling
# n = 7

# # Set the desired end-effector pose
# Tep = baxter.fkine(baxter.q, end=baxter.grippers[0]) * sm.SE3(0.08, 0.44, 0.2)

# arrived = False

# while not arrived:

#     # The pose of the Baxter's end-effector
#     Te = baxter.fkine(baxter.q, end=baxter.grippers[0])

#     # Transform from the end-effector to desired pose
#     eTep = Te.inv() * Tep

#     # Spatial error
#     e = np.sum(np.abs(np.r_[eTep.t, eTep.rpy() * np.pi/180]))

#     # Calulate the required end-effector spatial velocity for the robot
#     # to approach the goal. Gain is set to 1.0
#     v, arrived = rtb.p_servo(Te, Tep, 1.0)
#     print(arrived)

#     # Gain term (lambda) for control minimisation
#     Y = 0.01

#     # Quadratic component of objective function
#     Q = np.eye(n + 6)

#     # Joint velocity component of Q
#     Q[:n, :n] *= Y

#     # Slack component of Q
#     Q[n:, n:] = (1 / e) * np.eye(6)

#     # The equality contraints
#     Aeq = np.c_[baxter.jacobe(baxter.q, end=baxter.grippers[0]), np.eye(6)]
#     beq = v.reshape((6,))

#     # The inequality constraints for joint limit avoidance
#     Ain = np.zeros((n + 6, n + 6))
#     bin = np.zeros(n + 6)

#     # The minimum angle (in radians) in which the joint is allowed to approach
#     # to its limit
#     ps = 0.05

#     # The influence angle (in radians) in which the velocity damper
#     # becomes active
#     pi = 0.9

#     # Form the joint limit velocity damper
#     Ain[:n, :n], bin[:n] = baxter.joint_velocity_damper(ps, pi, n)

#     # Linear component of objective function: the manipulability Jacobian
#     c = np.r_[-baxter.jacobm(end=baxter.grippers[0]).reshape((n,)), np.zeros(6)]

#     # The lower and upper bounds on the joint velocity and slack variable
#     lb = -np.r_[baxter.qdlim[-(n+2):-2], 10 * np.ones(6)]
#     ub = np.r_[baxter.qdlim[-(n+2):-2], 10 * np.ones(6)]

#     # Solve for the joint velocities dq
#     qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub)

#     # Apply the joint velocities to the Baxter
#     baxter.qd[-(n+2):-2] = qd[:n]
#     print(baxter.qd.shape, qd.shape)

#     # Step the simulator by 50 ms
#     env.step(0.1)


# ======================================================== #
# # Launch the simulator Swift
# env = swift.Swift()
# env.launch(realtime=True)
# env.set_camera_pose([1.8, 0, .5], [0, 0, 0])

# # Create a Baxter robot object
# baxter = rtb.models.Baxter()

# # Set joint angles to ready configuration
# baxter.q = baxter.qn

# # Add the Baxter to the simulator
# env.add(baxter)

# # Define pose goals
# pose_left = [0.08, 0.44, -0.3]
# pose_right = [0.08, -0.44, 0.2]

# # Initialise and start loop
# left_arrived = False
# right_arrived = False

# Tep_left = baxter.fkine(baxter.q, end=baxter.grippers[0]) * sm.SE3(pose_left[0], pose_left[1], pose_left[2])
# Tep_right = baxter.fkine(baxter.q, end=baxter.grippers[1]) * sm.SE3(pose_right[0], pose_right[1], pose_right[2])

# while not (left_arrived and right_arrived):
#     # Set velocities for each arm
#     if not left_arrived:
#         # Set the desired end-effector pose
#         left_arrived = step_arm(baxter, 'left', Tep_left)
#         print('left: ', left_arrived)
#     if not right_arrived:
#         # Set the desired end-effector pose
        
#         right_arrived = step_arm(baxter, 'right', Tep_right)
#         print('right: ', right_arrived)

#     # Step the simulator
#     env.step(0.1)

