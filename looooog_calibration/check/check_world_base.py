#!/usr/bin/python
# -*- coding: utf-8 -*-
#此check函数基于标定程序进行修改，删除了计算变换矩阵和保存数据的部分，只使用其中读取数据结果的内容
import threading
from copy import deepcopy
import yaml
import rospy
import roslib
import tf
import numpy
from geometry_msgs.msg import Pose
import os

filefolder = os.path.dirname(os.path.dirname(__file__)) #上级文件夹

def pose2matrix(pose): #此函数可根据读取的机械臂姿态获得机械臂基座坐标系到机械臂末端坐标系的变化矩阵
    Position = pose[0:3].reshape(3,1)
    qx = pose[3]
    qy = pose[4]
    qz = pose[5]
    qw = pose[6]

    Rot = numpy.array([[2 * qw ** 2 + 2 * qx ** 2 - 1, 2 * qx * qy - 2 * qw * qz, 2 * qx * qz + 2 * qw * qy],
                    [2 * qx * qy + 2 * qw * qz, 2 * qw ** 2 + 2 * qy ** 2 - 1, 2 * qy * qz - 2 * qw * qx],
                    [2 * qx * qz - 2 * qw * qy, 2 * qy * qz + 2 * qw * qx, 2 * qw ** 2 + 2 * qz ** 2 - 1]])

    TransformationMatrix = numpy.vstack((numpy.hstack((Rot, Position)),
                                        numpy.array([0,0,0,1])))
    return TransformationMatrix

def rotMatrix2AngleAxis(Rot):
    theta = numpy.arccos((numpy.trace(Rot)-1)/2)
    ax = Rot - Rot.transpose()
    a = numpy.array([ax[2,1], ax[0,2], ax[1,0]]).reshape(3,1)
    scale = numpy.sqrt(numpy.dot(a.transpose(), a))
    if scale>0.00001:
        a = a/scale
        a = a*theta
    else:
        a = numpy.zeros([3,1])

    return a

def opencv_matrix_constructor(loader, node):
    mapping = loader.construct_mapping(node, deep=True)
    mat = numpy.array(mapping["data"])
    if mapping["cols"] > 1:
        mat.resize(mapping["rows"], mapping["cols"])
    else:
        mat.resize(mapping["rows"], )
    return mat
yaml.add_constructor(u"tag:yaml.org,2002:opencv-matrix", opencv_matrix_constructor)

def opencv_matrix_representer(dumper, mat):
    if mat.ndim > 1:
        mapping = {'rows': mat.shape[0], 'cols': mat.shape[1], 'dt': 'f', 'data': mat.reshape(-1).tolist()}
    else:
        mapping = {'rows': mat.shape[0], 'cols': 1, 'dt': 'f', 'data': mat.tolist()}
    return dumper.represent_mapping(u"tag:yaml.org,2002:opencv-matrix", mapping)
yaml.add_representer(numpy.ndarray, opencv_matrix_representer)


class CalibrateWorldBase:
    def __init__(self):
        rospy.on_shutdown(self._shutdown)
        
        self._terminate_all_thread = False

        self._current_robot_pose = [0]*7
        
        #此矩阵为标定时所用探头到机械臂末端的变换矩阵，0.09162为探头长度
        self._Ttip_eef = numpy.array([[1.0, 0.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0, 0.0],
                                    [0.0, 0.0, 1.0, 0.09162],
                                    [0.0, 0.0, 0.0, 1.0]])
        numpy.save(os.path.join(filefolder, 'calibration_files', 'transform_Ttip_eef.npy'),self._Ttip_eef)
        self._child_frame = 'wrist_3_link'
        self._parent_frame = 'base'

        self._calibration_file = os.path.join(filefolder, 'calibration_files', 'transforms.yaml')

        # mutex to protect data in multiple threads
        self._point_data_save_mutex = threading.Lock()

        # # publish fake frames for testing
        # self._thread_publish_fixed_frames = threading.Thread(target=self._publish_fixed_frames)
        # self._thread_publish_fixed_frames.start()

        # keep listence end effector pose and update variables
        self._thread_update_robot_pose = threading.Thread(target=self._update_robot_pose)
        self._thread_update_robot_pose.start()

        # save current eef pose when user press s
        self._thread_calibrate = threading.Thread(target=self._calibrate)
        self._thread_calibrate.start()

        rospy.spin()
        

    def _shutdown(self):
        self._terminate_all_thread = True
        self._thread_calibrate.join()
        self._thread_update_robot_pose.join()
        # self._thread_publish_fixed_frames.join()
        pass


    def _publish_fixed_frames(self):
        rospy.loginfo('start publish_fixed_frames')
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(10.0)
        i = 1
        while not (rospy.is_shutdown() or self._terminate_all_thread):
            rate.sleep()
            i = i+1
            br.sendTransform([i, 2.0, 3.0], [0.1, 0.2, 0.3, numpy.sqrt(0.86)], rospy.Time.now(), self._child_frame, self._parent_frame)


    def _update_robot_pose(self):
        rospy.loginfo('start update_robot_pose')
        tf_listener = tf.TransformListener()

        rate = rospy.Rate(20.0)
        while not (rospy.is_shutdown() or self._terminate_all_thread):
            rate.sleep()
            try:
                tf_listener.waitForTransform(self._parent_frame, self._child_frame, rospy.Time(0), rospy.Duration(3.0))
                (trans, rot) =  tf_listener.lookupTransform(self._parent_frame, self._child_frame, rospy.Time(0))
                with self._point_data_save_mutex:
                    self._current_robot_pose = trans + rot

            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("CalibrateWorldBase did not get transform between base and world")
                rospy.sleep(1.0)
                continue


    def _save_pose(self):
        rospy.loginfo('start save_pose')
        Poseeef_base = numpy.zeros([4,7]) 
        # Poseeef_base[0,:]= numpy.array([-0.80684661, -0.0296737 ,  0.08137786, -0.99997513,  0.00124283,   -0.00573344,  0.00391359])
        # Poseeef_base[1,:]= numpy.array([-0.72662636, -0.02751858,  0.08204175, -0.99997274,  0.00180992,   -0.0056562 ,  0.00438702])
        # Poseeef_base[2,:]= numpy.array([-0.72656286,  0.01257621,  0.08115722, -0.99997262,  0.00182258,   -0.00568656,  0.00436998])
        # Poseeef_base[3,:]= numpy.array([-0.80650982,  0.01094983,  0.08167092, -0.99997506,  0.00124933,   -0.00574651,  0.00390991])                        
        while not (rospy.is_shutdown() or self._terminate_all_thread):
            usr_input = raw_input("Press s# to save current eef position as corner # point, eg: s2 to 2nd corner. Yes you can override previous values. Or type done to finish collecting four points\n")
            with self._point_data_save_mutex:
                if usr_input == 'done':
                    break
                elif usr_input == 's1':
                    Poseeef_base[0,:] = self._current_robot_pose
                elif usr_input == 's2':
                    Poseeef_base[1,:] = self._current_robot_pose
                elif usr_input == 's3':
                    Poseeef_base[2,:] = self._current_robot_pose
                elif usr_input == 's4':
                    Poseeef_base[3,:] = self._current_robot_pose
                else:
                    rospy.logerr('your input {} is illeague!\n'.format(usr_input))
        numpy.save(os.path.join(filefolder, 'calibration_files', 'Poseeef_base.npy'),Poseeef_base) 
        rospy.loginfo("four points are \n{}".format(Poseeef_base))
        print("Poseeef_base=",Poseeef_base)
        return Poseeef_base


    def _calibrate(self):
        PoseV_pts_base = self._save_pose()

        Ppts_base = numpy.zeros([4,3])
        Rpts_base = numpy.zeros([4,3,3])
        Tpts_base = numpy.zeros([4,4,4])

        for i in range(4):
            Tpts_base[i,:,:] = numpy.dot(pose2matrix(PoseV_pts_base[i,:]), self._Ttip_eef)
            Ppts_base[i,:]=Tpts_base[i,0:3,3].reshape(1,3)
            print("Ppt_base",Ppts_base)
            Rpts_base[i,:,:]=Tpts_base[i,0:3,0:3]
            [-0.76304927, -0.02954149,  0.04839884]

if __name__ == "__main__":
    rospy.loginfo("start recording four points on the chess board. Please place the chess board in right position, and guide robot calibration tip to the four corners in the anti-clock order, starting from the point closest to robot base, then press 's' to save the position.")
    rospy.init_node('calibrate_world_base')
    check= CalibrateWorldBase()
    poseeef_base = numpy.load(os.path.join(filefolder, 'calibration_files', 'Poseeef_base.npy'))
    Ttip_eef = numpy.load(os.path.join(filefolder, 'calibration_files', 'transform_Ttip_eef.npy'))
    Tpts_base = numpy.zeros([4,4,4])
    checkpoints_raw = numpy.zeros([4,3])

    filename = os.path.join(filefolder, 'calibration_files', 'transforms.yaml')
    with open(filename, 'r') as yaml_file:
        data = yaml.load(yaml_file)
    baseToworld_matrix = numpy.array(data['H_BaseToworld']['data']).reshape(4, 4)

    for i in range(4):
        Tpts_base[i,:,:] = numpy.dot(pose2matrix(poseeef_base[i,:]), Ttip_eef)
        checkpoints_raw[i,:]=Tpts_base[i,0:3,3].reshape(1,3)
    print(checkpoints_raw)
    print("check_raw=",checkpoints_raw)
    ones = numpy.ones([4,1])
    checkpoints = numpy.hstack((checkpoints_raw,ones))
    checkpoints_base = numpy.dot(baseToworld_matrix,checkpoints.T)
    print("check_result = ",checkpoints_base.T)