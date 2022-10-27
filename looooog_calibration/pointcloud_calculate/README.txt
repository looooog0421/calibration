此文件夹程序用于获取物体图像、生成点云并发布到ros节点中

save_image.py程序用于获取物体图像，使用时需要先launch深度相机，然后点击运行，待出现实时彩色图和深度图后，先按下“s”拍摄一张无物体的背景图，再放上物体按“s”生成有物体的图像。完成之后“q”即可退出程序。

objectPoint.py和pointEllipsoid.py分别是从物体的图像生成物体点云、从物体点云生成物体外接最小椭球的计算代码。实际操作时不需要运行这两个程序。

main_object.py和main_ellipsoid.py分别是生成物体点云和椭球点云的代码，

main_object.py只调用了objectPoint.py，

而main_ellipsoid.py调用了objectPoint.py和pointEllipsoid.py两个程序，因此可以直接从图像生成椭球点云。

roscore之后这两个程序都可以直接使用，无需其他操作。