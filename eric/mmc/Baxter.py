#!/usr/bin/env python

import numpy as np
from roboticstoolbox.robot.ERobot import ERobot
from spatialmath import SE3


class Baxter(ERobot):
    """
    Class that imports a Baxter URDF model

    ``Baxter()`` is a class which imports a Baxter robot definition
    from a URDF file.  The model describes its kinematic and graphical
    characteristics.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.URDF.Baxter()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration
    - qs, arm is stretched out in the x-direction
    - qn, arm is at a nominal non-singular configuration

    .. codeauthor:: Jesse Haviland
    .. sectionauthor:: Peter Corke
    """

    def __init__(self):

        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "baxter_description/urdf/baxter_bot.urdf"
        )
        # print(urdf_filepath)
        # print("====================================== number of links:", len(links))
        # print("====================================== links:", links)
        # for i, link in enumerate(links):
        #     print(i, link)

        super().__init__(
            links,
            name=name,
            manufacturer="Rethink",
            base_link=links[0],
            gripper_links=[links[50], links[56]],
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        # self.grippers[0].tool = SE3(0, 0, 0.1034)

        self.qdlim = np.array(
            [10000, 10000, 1.5, 1.5, 1.5, 1.5, 4.0, 4.0, 4.0, 1.5, 1.5, 1.5, 1.5, 4.0, 4.0, 4.0, 10000, 10000, 1.5, 1.5, 1.5, 1.5, 4.0, 4.0, 4.0, 1.5, 1.5, 1.5, 1.5, 4.0, 4.0, 4.0]
        )

        self.addconfiguration("qz", np.zeros(self.n))

        self.addconfiguration(
            "qr", np.zeros(self.n)
        )

        neu = [-4.875246055746629e-07, -1.5394258110465822e-10, -0.020833000269383728, -1.189972011753838, 1.9400294787022645, -0.0800000014515021, -0.9999845805893166, 0.6699967180466144, 1.030008744611373, -0.4999997026333549, 0.020833298459408452, 1.2980118078276757e-10, 1.1899720108865521, 1.9400294782207883, 0.08000000114257322, -0.9999845809809731, -0.6699967427602509, 1.0300087584583828, 0.49999971413654354]

        ros_to_rtb = [0, 14, 15, 12, 13, 16, 17, 18, 10, 11, 5, 6, 3, 4, 7, 8, 9, 1, 2]

        neu[:] = [neu[i] for i in ros_to_rtb]

        # neutral position
        self.addconfiguration("qn", np.array(neu))

        # rostopic joint names: ['head_pan', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
        
        # roboticstoolbox joint names: ['head', 'right_upper_shoulder', 'right_lower_shoulder', 'right_upper_elbow', 'right_lower_elbow', 'right_upper_forearm', 'right_lower_forearm', 'right_wrist', 'r_gripper_l_finger', 'r_gripper_r_finger', 'left_upper_shoulder', 'left_lower_shoulder', 'left_upper_elbow', 'left_lower_elbow', 'left_upper_forearm', 'left_lower_forearm', 'left_wrist', 'l_gripper_l_finger', 'l_gripper_r_finger']


if __name__ == "__main__":  # pragma nocover

    robot = Baxter()
    print(robot)

    for link in robot.links:
        print(link.name)
        print('joint?: ', link.isjoint)
        print(len(link.collision))
        print(link.jindex)

    print()

    for link in robot.grippers[0].links:
        print(link.name)
        print('joint?: ', link.isjoint)
        print(len(link.collision))
