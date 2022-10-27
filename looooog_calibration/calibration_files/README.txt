本文件中保存有标定时产生的数据文件，为方便初次使用时快速了解各个文件的用途，现对这些文件做说明：

1.transforms.yaml:最重要的文件，保存有相机内参矩阵、外参矩阵、机械臂与世界坐标系的变换矩阵。此套标定程序中的每一个程序都使用了这个文件。

2.raw_img.jpg：显示相机标定拍到的最原始图像。由calibrate_world_camera.py生成。

3.four_corners.jpg：显示相机标定时用到的角点群中的四个顶点，用于指导机械臂标定，机械臂标定时需要根据"红"→"黄"→"绿"→"蓝"的颜色顺序进行标定，
并且红点到黄点的方向为世界坐标系x轴正方向，红点到绿点的顺序为世界坐标系y轴正方向。

4.corners.npy:保存相机标定时所有的角点像素坐标，用于检测标定结果。由calibrate_world_camera.py生成，由check_world_camera.py使用。

5.check_base_camera.jpg:显示机械臂与相机综合标定的结果。由check_base_camera.py生成。

6.transform_Ttip_eef.npy:保存机械臂末端到探测顶针的变换矩阵。由check_world_base.py生成，由check_world_base.py和check_base_camera.py使用。

7.Poseeef_base.npy:保存检测时机械臂传入的四个检测点的姿态信息。由check_world_base.py生成，由check_world_base.py和check_base_camera.py使用。

8.check_world_camera：相机标定检测的结果图片。由check_world_camera.py生成。