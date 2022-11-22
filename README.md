## LVI-SAM 原仓库链接
https://github.com/TixiaoShan/LVI-SAM

## 数据集下载
LVI-SAM原作者录制的handheld.bag

链接: https://pan.baidu.com/s/1yJeSZ_GVj1tTejG4KbNH5Q 提取码: iw9x 

## 运行
先运行launch file，再运行rosbag提供数据：
```
roslaunch lvi_sam run.launch
rosbag play /home/zhoush/Documents/dataset/LVI-SAM/handheld-002.bag
```
![img](./doc/output1.png)

尝试降低bag的播放速度[参考](https://blog.csdn.net/learning_tortosie/article/details/116051761)
```
rosbag info /path/to/handheld.bag -r 0.5
```

