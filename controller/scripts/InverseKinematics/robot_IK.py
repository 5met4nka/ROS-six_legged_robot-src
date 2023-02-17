#!/usr/bin/env python3

import numpy as np
from math import sqrt, atan2, asin, sin, cos, pi, acos
from RoboticsUtilities.Transformations import homog_transform_inverse,homog_transform

class InverseKinematics(object):
    def __init__(self, bodyDimensions, legDimensions):

        # Body dimensions
        self.bodyLength = bodyDimensions[0]
        self.bodyWidth1 = bodyDimensions[1]
        self.bodyWidth2 = bodyDimensions[2]

        # Leg dimensions
        self.l1 = legDimensions[0]
        self.l2 = legDimensions[1]
        self.l3 = legDimensions[2]
        self.l4 = legDimensions[3]

        self.cnt = 0

    def get_local_positions(self,leg_positions,dx,dy,dz,roll,pitch,yaw):
        """
        Compute the positions of the end points in the shoulder frames.
        """
        #print(leg_positions)
        leg_positions = (np.block([[leg_positions],[np.array([1,1,1,1,1,1])]])).T
        #print("leg_positions : \n" + str(leg_positions))

        # Transformation matrix, base_link_world => base_link
        T_blwbl = homog_transform(dx,dy,dz,roll,pitch,yaw)

        # Transformation matrix, base_link_world => FR1
        T_blwFR1 = np.dot(T_blwbl, homog_transform(+0.5*self.bodyLength,
                          +0.5*self.bodyWidth1,0,0,0,0))

        # Transformation matrix, base_link_world => FL1
        T_blwFL1 = np.dot(T_blwbl, homog_transform(+0.5*self.bodyLength,
                          -0.5*self.bodyWidth1,0,0,0,0))


        # Transformation matrix, base_link_world => FR1
        T_blwMR1 = np.dot(T_blwbl, homog_transform(0,
                          +0.5*self.bodyWidth2,0,0,0,0))

        # Transformation matrix, base_link_world => FL1
        T_blwML1 = np.dot(T_blwbl, homog_transform(0,
                          -0.5*self.bodyWidth2,0,0,0,0))


        # Transformation matrix, base_link_world => RR1
        T_blwRR1 = np.dot(T_blwbl, homog_transform(-0.5*self.bodyLength,
                          +0.5*self.bodyWidth1,0,0,0,0))

        # Transformation matrix, base_link_world => RL1
        T_blwRL1 = np.dot(T_blwbl, homog_transform(-0.5*self.bodyLength,
                          -0.5*self.bodyWidth1,0,0,0,0))

        # Local coordinates
        pos_FR = np.dot(homog_transform_inverse(T_blwFR1),leg_positions[0])
        pos_FL = np.dot(homog_transform_inverse(T_blwFL1),leg_positions[1])
        pos_MR = np.dot(homog_transform_inverse(T_blwMR1),leg_positions[2])
        pos_ML = np.dot(homog_transform_inverse(T_blwML1),leg_positions[3])        
        pos_RR = np.dot(homog_transform_inverse(T_blwRR1),leg_positions[4])
        pos_RL = np.dot(homog_transform_inverse(T_blwRL1),leg_positions[5])

        return(np.array([pos_FR[:3],
                         pos_FL[:3],
                         pos_MR[:3],
                         pos_ML[:3],
                         pos_RR[:3],
                         pos_RL[:3]]))

    def inverse_kinematics(self,leg_positions,dx,dy,dz,roll,pitch,yaw):

        positions = self.get_local_positions(leg_positions,dx,dy,dz,roll,pitch,yaw)

        angles = []

        for i in range(6):

            x = positions[i][0]
            y = positions[i][1]
            z = positions[i][2]

            if(i == 0 or i == 1):
                theta1 = atan2(x,y) - pi/4*(1+2*i)
            elif(i == 2 or i == 3):
                theta1 = atan2(x,y) - pi*(i-2)
            else:
                theta1 = atan2(x,y) + pi/4*(1+2*(i-4))

            if(theta1 < -pi):
                theta1 = theta1 + 2*pi

            theta2 = -asin((0.07475 + z)/self.l2)
            theta3 = theta2
            theta4 = theta2
            
            angles.append(theta1)
            angles.append(theta2)
            angles.append(theta3)
            angles.append(theta4)
        self.cnt += 1

        return angles
