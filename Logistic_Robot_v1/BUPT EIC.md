
# BUPT EIC  

北京邮电大学鸿雁战队2023工程创新与实践大赛电控代码仓库

## 使用说明

上电后，调整机械臂到指定位置后，按下A版上的白键锁定

## 机械臂动作说明

### 旋转2006简称Ⅰ，升降2006简称Ⅱ

#### 运行前需Ⅰ要标定的位置

| 编号 | a | b |
| :-----:| :----: |  :----: |
| 描述 | 向内 |向外|

起始位置应该与机械臂最近的一条边垂直，且正对着4015载物盘

#### 运行前升降要标定的位置

| 编号 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
| :-----:| :----: | :----: | :----: | :----: | :----: | :----: | -------| -------|
| 描述 | 车内物料抓取预备点 |车内物料抓取点| 原料区物料抓取预备点 |原料区物料抓取点 | 地面物料预备点 | 地面取放点 | 细加工区叠放预备点 | 细加工区放置点 |

---

### 机械臂任务码设计及流程

基本动作：

1. arm底盘旋转：初始化，向外，向内

2. 升降机构：初始化（车上物料盘），向上，向下

3. 物料盘：初始化，物料加一

4. 摄像头舵机：二维码位置，物料位置

5. 机械爪：打开，关闭

   

### 启动区

task0: 初始化任务：

​	物料底盘初始化，arm底盘初始化，升降初始化（略高于物料），舵机二维码位置

### 二维码区

task1：读取二维码任务：

​	arm底盘向外， 机械爪打开

### 原料区

{

​	task2：预抓取：

​		 舵机物料位置，升降预备抓取位置，（车预备位置）

​	task3：原料区抓取：

​		升降降低位置，机械爪关闭

​	task4：物料放置车上：

​		升降抬升位置，arm底盘向内，升降降低至放置位置，机械爪打开，升降抬升

​	task5：

​		物料加一

}

task2-5执行三次

### 粗加工区

{

​	task6：预放置：

​		arm底盘向内，升降初始化（略高于物料），机械爪打开，物料盘转至对应物料，升降向下至物料处，机械爪关闭，升降抬升位置，arm底盘向外

​	task7：放置：

​		升降降低，机械爪打开，升降抬升

}

task6-7执行三次

{

​	task8：预抓取：

​		 arm底盘向外，升降预备抓取位置，（车预备位置）

​	task9：抓取：

​		升降降低位置，机械爪关闭

​	task10：物料放置车上：

​		升降抬升位置，arm底盘向内，升降降低至放置位置，机械爪打开

​	task11：

​		物料加一

}

task8-11执行三次

### 细加工区

{

​	task12：预放置：

​		arm底盘向内，升降初始化（略高于物料），机械爪打开，物料盘转至对应物料，升降向下至物料处，机械爪关闭，升降抬升位置，arm底盘向外

​	task13：放置：

​		升降降低，机械爪打开，升降抬升

}

task12-13执行三次

## 进入下一轮原料区

原料区、粗加工区任务重做

### 叠放

{

​	task14：预放置：

​		arm底盘向内，升降初始化（略高于物料），机械爪打开，物料盘转至对应物料，升降向下至物料处，机械爪关闭，升降抬升位置，arm底盘向外

​	task15：放置：

​		升降降低，机械爪打开，升降抬升

}

task14-15执行三次

### 通信接口说明

---

usart6 - 与NUC的收发和交流\
uart8 - 与action的收发和交流\
uart7 - 调试使用接口\
dbus - 遥控器接口\
can - 和电机交互\
can1 - MS4015\
can2 - M2006&M3508

---

## Work To be done & optimized

· **待完成** 影响是否能够完成项目的事项\
· **待优化** 对已有的功能进行性能提高/模块解耦/可维护性增强\
· **待添加** 无关紧要，锦上添花

## 开发准则

### 禁止过度摸鱼

### 禁止在临界区使用os延迟

#### 临界资源

指一次仅仅允许一个线程访问的共享资源，它可以是一个具体的硬件设备，也可以是一个变量，可以是缓冲区等，这些都称为临界资源，或者共享资源，多个任务同时运行，只能允许一个任务访问的资源。

#### 临界区

每个进程中访问临界资源的那段代码称为临界区，
每次只允许一个进程进入临界区，进入后，不允许其他进程进入。不论是硬件临界资源还是软件临界资源，多个进程必须互斥的对它进行访问。多个进程涉及到同一个临界资源的的临界区称为相关临界区。使用临界区时，一般不允许其运行时间过长，只要运行在临界区的线程还没有离开，其他所有进入此临界区的线程都会被挂起而进入等待状态。

### 禁止在使用开发版自带的xt30接口接电机

电机的反向电动势可能会烧毁开发板。