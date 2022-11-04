#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import division
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import sys
import numpy as np
from numpy import linalg
from random import random
# import rospy
# from sensor_msgs.msg import PointCloud2
# from sensor_msgs.msg import PointField

class EllipsoidTool:
    """Some stuff for playing with ellipsoids"""

    def __init__(self):
        pass

    def getMinVolEllipse(self, P=None, tolerance=0.01):
        """ Find the minimum volume ellipsoid which holds all the points

        Based on work by Nima Moshtagh
        http://www.mathworks.com/matlabcentral/fileexchange/9542
        and also by looking at:
        http://cctbx.sourceforge.net/current/python/scitbx.math.minimum_covering_ellipsoid.html
        Which is based on the first reference anyway!

        Here, P is a numpy array of N dimensional points like this:
        P = [[x,y,z,...], <-- one point per line
             [x,y,z,...],
             [x,y,z,...]]

        Returns:
        (center, radii, rotation)

        """
        (N, d) = np.shape(P)
        d = float(d)

        # Q will be our working array
        Q = np.vstack([np.copy(P.T), np.ones(N)])
        QT = Q.T

        # initializations
        err = 1.0 + tolerance
        u = (1.0 / N) * np.ones(N)

        # Khachiyan Algorithm
        while err > tolerance:
            V = np.dot(Q, np.dot(np.diag(u), QT))
            M = np.diag(np.dot(QT, np.dot(linalg.inv(V), Q)))  # M the diagonal vector of an NxN matrix
            j = np.argmax(M)
            maximum = M[j]
            step_size = (maximum - d - 1.0) / ((d + 1.0) * (maximum - 1.0))
            new_u = (1.0 - step_size) * u
            new_u[j] += step_size
            err = np.linalg.norm(new_u - u)
            u = new_u

        # center of the ellipse
        center = np.dot(P.T, u)

        # the A matrix for the ellipse
        A = linalg.inv(
            np.dot(P.T, np.dot(np.diag(u), P)) -
            np.array([[a * b for b in center] for a in center])
        ) / d

        # Get the values we'd like to return
        U, s, rotation = linalg.svd(A)
        radii = 1.0 / np.sqrt(s)

        return center, radii, rotation

    def getEllipsoidVolume(self, radii):
        """Calculate the volume of the blob"""
        return 4. / 3. * np.pi * radii[0] * radii[1] * radii[2]

    def getEllipsoidPoint(self, center, radii, rotation):
        # 1.均分100个方位角和仰角
        theta = np.linspace(0.0, 2.0 * np.pi, 100)  # 方位角
        phi = np.linspace(0.0, np.pi, 100)  # 仰角

        # 2.每对方位角和仰角对应椭球上的一个点，在椭球自身坐标系下表示该点
        x = radii[0] * np.outer(np.cos(theta), np.sin(phi))
        y = radii[1] * np.outer(np.sin(theta), np.sin(phi))
        z = radii[2] * np.outer(np.ones_like(theta), np.cos(phi))

        x = x.reshape(-1, 1)
        y = y.reshape(-1, 1)
        z = z.reshape(-1, 1)

        ellipsoid_point = np.hstack((x, y, z))

        # 3.将每个点从椭球自身坐标系变换到世界坐标系
        # rotate accordingly
        for i in range(ellipsoid_point.shape[0]):
            ellipsoid_point[i] = np.dot(ellipsoid_point[i], rotation) + center

        return ellipsoid_point


# def talker(points):

#     pub = rospy.Publisher('ellipsoid_topic', PointCloud2, queue_size=5)
#     rospy.init_node('ellipsoid_pointcloud_publisher_node', anonymous=True)
#     rate = rospy.Rate(1)
#     points = points / 1000

#     while not rospy.is_shutdown():

#         msg = PointCloud2()
#         msg.header.stamp = rospy.Time().now()
#         msg.header.frame_id = "base"

#         if len(points.shape) == 3:
#             msg.height = points.shape[1]
#             msg.width = points.shape[0]
#         else:
#             msg.height = 1
#             msg.width = len(points)

#         msg.fields = [
#             PointField('x', 0, PointField.FLOAT32, 1),
#             PointField('y', 4, PointField.FLOAT32, 1),
#             PointField('z', 8, PointField.FLOAT32, 1)]
#         msg.is_bigendian = False
#         msg.point_step = 12
#         msg.row_step = msg.point_step * points.shape[0]
#         msg.is_dense = False
#         msg.data = np.asarray(points, np.float32).tostring()
#         print(points[:3,:3])
#         pub.publish(msg)
#         print("published...")
#         rate.sleep()



if __name__ == "__main__":
    pass