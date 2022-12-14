#!/usr/bin/python
# -*- coding: utf-8 -*-
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
filename = os.path.join(filefolder, 'calibration_files', 'transforms.yaml')


def pose2matrix(pose):
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

def hornQuar(A, B): #此程序输入为标定点在世界坐标系和机械臂基座坐标系下的坐标，计算原理来源知乎：https://zhuanlan.zhihu.com/p/436410920
    """
    % Registers two sets of 3DoF data
    % Assumes A and B are d,n sets of data
    % where d is the dimension of the system
    % typically d = 2,3
    % and n is the number of points
    % typically n>3
    %
    % Mili Shah
    % July 2014
    :param A:
    :param B:
    :return:
    """

    d, n = A.shape
    # Mean Center Data
    print("A=",A)
    Ac = numpy.mean(A,1).reshape(3,1)
    Bc = numpy.mean(B,1).reshape(3,1)
    print("Ac=",Ac)
    print("Bc=",Bc)
    A_reset = A - numpy.kron(Ac, numpy.ones(n))
    B_reset = B - numpy.kron(Bc, numpy.ones(n))
    print("A_reset",A_reset)
    print("B_reset=",B_reset)
    A_reset[2,:] = Ac[2,0]
    print("newA_reset=",A_reset)
    # Calculate Optimal Rotation
    m = numpy.zeros((3,3))
    m = numpy.dot(A_reset,B_reset.T)
    print("m=",m)

    u,sigma,v = numpy.linalg.svd(m)
    print("uv,sigma",u,v,sigma)
    
    RB_A = numpy.dot(v, numpy.transpose(u))
    PB_A = Bc-numpy.dot(RB_A,Ac)
    print("RB_A=",RB_A)
    print("PB_A=",PB_A)
    return RB_A, PB_A

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

def write_to_opencv_matrix(file_name, numpy_mat_name, numpy_mat):
    shape = list(numpy_mat.shape)
    if len(shape) == 1:
        rows = 1
        cols = shape[0]
    elif len(shape) == 2:
        rows = shape[0]
        cols = shape[1]
    else:
        raise('write_to_opencv_matrix can only convert and write 2d numpy array')

    list_mat = numpy_mat.reshape([1, rows*cols]).tolist()[0]
    with open(file_name, 'a') as f:
        f.write(numpy_mat_name + ':\n')
        f.write('  rows: ' + str(rows) + '\n')
        f.write('  cols: ' + str(cols) + '\n')
        f.write('  dt: f\n')
        f.write('  data: [')
        for i in range(rows*cols):
            f.write(' ' + str(list_mat[i]))
            if i < rows*cols-1:
                f.write(',')
        f.write(']\n')

class CalibrateWorldBase:
    def __init__(self):
        rospy.on_shutdown(self._shutdown)
        
        self._terminate_all_thread = False

        self._current_robot_pose = [0]*7
        
        self._Ttip_eef = numpy.array([[1.0, 0.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0, 0.0],
                                    [0.0, 0.0, 1.0, 0.09162],
                                    [0.0, 0.0, 0.0, 1.0]])

        self._child_frame = 'wrist_3_link'
        self._parent_frame = 'base'

        self._calibration_file = filename

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
            Rpts_base[i,:,:]=Tpts_base[i,0:3,0:3]


        #pts_world is four points in world frame
        Ppts_world = numpy.array( [ [0.0,  0.08,  0.0001],
                                    [0.08, 0.08,  0.0001],
                                    [0.08, 0.04,  0.0001],
                                    [0.00, 0.04,  0.0001] ] )
        print("Ppts_world",Ppts_world)


        #Calculcate RP from four point in base frame and world frame
        Rworld_base, Pworld_base = hornQuar(Ppts_base.transpose(), Ppts_world.transpose())


        Rbase_world = Rworld_base.transpose()
        Pbase_world = numpy.dot(-Rbase_world, Pworld_base)

        H_Bw = numpy.vstack((numpy.hstack((Rbase_world, Pbase_world)), numpy.array([0.0, 0.0, 0.0, 1.0])))
        H_wB = numpy.vstack((numpy.hstack((Rworld_base, Pworld_base)), numpy.array([0.0, 0.0, 0.0, 1.0])))

        Rvector_base_world = rotMatrix2AngleAxis(Rbase_world)

        write_to_opencv_matrix(self._calibration_file, 'R_worldToBase', Rbase_world)
        write_to_opencv_matrix(self._calibration_file, 'T_worldToBase', Pbase_world)
        write_to_opencv_matrix(self._calibration_file, 'H_worldToBase', H_Bw)
        write_to_opencv_matrix(self._calibration_file, 'R_BaseToworld', Rworld_base)
        write_to_opencv_matrix(self._calibration_file, 'T_BaseToworld', Pworld_base)
        write_to_opencv_matrix(self._calibration_file, 'H_BaseToworld', H_wB)

if __name__ == "__main__":
    rospy.loginfo("start recording four points on the chess board. Please place the chess board in right position, and guide robot calibration tip to the four corners in the anti-clock order, starting from the point closest to robot base, then press 's' to save the position.")
    rospy.init_node('calibrate_world_base')
    CalibrateWorldBase()
