
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

#### 运行前需Ⅱ要标定的位置

| 编号 | 1 | 2 | 3 | 4 | 5 | 6 |
| :-----:| :----: | :----: | :----: | :----: | :----: | :----: |
| 描述 | 起始位置 |二维码读取位置| 抓取原料区的物块位置 |抓取物料盘上物块位置 | 最高位置（高过物块） | 最低位置（放于地面）

 ---

### 机械臂任务码设计及流程

起始状态 ：高度正好能读到二维码，机械爪关闭，4015调整角度到正对着爪子\
 **id-1** Ⅱ 1 -》2，Ⅰ a-》b，机械爪张开，舵机调整位置准备读取二维码\
 **id-2** Ⅱ 2 -》3，（此时小车向前调整位置）\
 **id-3** 机械爪闭合，Ⅱ 3 -》5，Ⅰb-》a，Ⅱ 5 -》4，机械爪放开，Ⅱ 4 -》5，4015旋转120°，I a-》b\
 **id-4** 到达粗加工区后，机械爪闭合，2到达最高位置，1旋转180°，准备对位\
 **id-4** 对位完成，张开机械爪将物块放下，2到达最高位置，1旋转反向，2下降至抓取位置

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
