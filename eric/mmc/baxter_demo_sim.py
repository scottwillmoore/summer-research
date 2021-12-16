import roboticstoolbox as rtb
import spatialmath as sm
import spatialgeometry as sg
import numpy as np
import qpsolvers as qp
from spatialmath.base.symbolic import pi
import swift


def step_baxter_arm(baxter, which_ee, Tep):
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

    # Apply the joint velocities to the Baxter
    baxter.qd[idx_offset:idx_offset+n] = qd[:n]
    # print(baxter.qd.shape, qd.shape)
    print('velocities:', qd[:n])

    return arrived

def step_baxter(swift_env, baxter, pose_left, pose_right, dt=0.1):
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
    swift_env.add(l_target)
    r_target = sg.Sphere(0.01, color=[0.2, 0.4, 0.65, 0.5], base=Tep_right)
    swift_env.add(r_target)

    while not (left_arrived and right_arrived):
        # Set velocities for each arm
        if not left_arrived:
            # Set the desired end-effector pose
            left_arrived = step_baxter_arm(baxter, 'left', Tep_left)
            print('left: ', left_arrived)
        if not right_arrived:
            # Set the desired end-effector pose
            
            right_arrived = step_baxter_arm(baxter, 'right', Tep_right)
            print('right: ', right_arrived)

        # Step the simulator
        swift_env.step(dt)

# Launch the simulator Swift
env = swift.Swift()
env.launch(realtime=True)
env.set_camera_pose([1.8, 0, .5], [0, 0, 0])

l_frame = sg.Axes(0.1)
r_frame = sg.Axes(0.1)
env.add(l_frame)
env.add(r_frame)

# Create a Baxter robot object
baxter = rtb.models.Baxter()

# Set joint angles to ready configuration
baxter.q = baxter.qn

# Add the Baxter to the simulator
env.add(baxter)

dt = 0.1

# # Define pose goals
# pose_left = [0.08, 0.44, -0.3]
# pose_right = [0.08, -0.44, 0.2]

# # Move baxter
# step_baxter(env, baxter, pose_left, pose_right, dt)

# ================= Circular motion

# Define set of pose goals along a circular path
centre_l = np.array(baxter.fkine(baxter.q, end=baxter.grippers[0]).A)[:3, 3]
centre_r = np.array(baxter.fkine(baxter.q, end=baxter.grippers[1]).A)[:3, 3]

theta = np.linspace(0, 2*np.pi, num=50)
radius = 0.1
circ_path = np.stack((radius*np.cos(theta), radius*np.sin(theta), np.zeros(len(theta))))

goals_l = np.transpose(np.expand_dims(np.array(centre_l), 1) + circ_path)
goals_r = np.transpose(np.expand_dims(np.array(centre_r), 1) + circ_path)

while True:
    for gl, gr in zip(goals_l, goals_r):
        print(goals_l.shape)
        print(gl.shape)
        print("going to: ")
        print("left: ", gl, "right: ", gr)
        step_baxter(env, baxter, gl, gr, dt)