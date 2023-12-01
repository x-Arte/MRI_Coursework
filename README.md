# Task1

## 定轴问题 

参考资料：[详解PUMA 560机械臂的改进D-H参数和标准D-H参数表示 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/392320782)

![e07eb523568ccc63df3d04f339afa2b](E:\User\Documents\WeChat Files\wxid_h4df72nw6rt321\FileStorage\Temp\e07eb523568ccc63df3d04f339afa2b.jpg)

## 初始角度

由于这样定轴joint2存在初始角度问题，设为pi/2

joint5的角度根据实际机器人调整/ coppelia初始为竖直向下

# Task 2

## 1. 各个关节matlab 和 coppelia对应的不一样

### 注意：配图错误，因为coppelia和现实不一样，以matlab为准,解决方案更改为coppelia里面的angle进行取反

### joint2

####  x,y,z轴

对比

<img src="C:\Users\zmy\AppData\Roaming\Typora\typora-user-images\image-20231128195947287.png" alt="image-20231128195947287"  />

<img src="C:\Users\zmy\AppData\Roaming\Typora\typora-user-images\image-20231128200058081.png" alt="image-20231128200058081"  />

#### 姿态对比（不一致）：

+ pi/6

##### coppelia

![image-20231128200850714](C:\Users\zmy\AppData\Roaming\Typora\typora-user-images\image-20231128200850714.png)

##### matlab

![image-20231128201136308](C:\Users\zmy\AppData\Roaming\Typora\typora-user-images\image-20231128201136308.png)



#### 解决方案

取负

![image-20231128203737757](C:\Users\zmy\AppData\Roaming\Typora\typora-user-images\image-20231128203737757.png)

### joint3

####  x,y,z轴对比

<img src="C:\Users\zmy\AppData\Roaming\Typora\typora-user-images\image-20231128195947287.png" alt="image-20231128195947287"  />

![image-20231128203058503](C:\Users\zmy\AppData\Roaming\Typora\typora-user-images\image-20231128203058503.png)

#### 姿态对比（不一致）：

pi/6

##### coppelia

![image-20231128203248335](C:\Users\zmy\AppData\Roaming\Typora\typora-user-images\image-20231128203248335.png)

#####  matlab



![image-20231128202556934](C:\Users\zmy\AppData\Roaming\Typora\typora-user-images\image-20231128202556934.png)

#### 解决方案

matlab角度取负

![image-20231128203840406](C:\Users\zmy\AppData\Roaming\Typora\typora-user-images\image-20231128203840406.png)

****

### joint4

#### x,y,z轴对比

![image-20231128212332742](C:\Users\zmy\AppData\Roaming\Typora\typora-user-images\image-20231128212332742.png)

#### 姿态对比 (一致)

##### coppelia

![image-20231128203957337](C:\Users\zmy\AppData\Roaming\Typora\typora-user-images\image-20231128203957337.png)

![image-20231128213018166](C:\Users\zmy\AppData\Roaming\Typora\typora-user-images\image-20231128213018166.png)"# MRI_Coursework" 
