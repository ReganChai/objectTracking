# objectTracking


## 说明

### **publish.cpp**
- 从摄像头获取 ROS Image，然后将其转换为 OpenCV Image;

- 鼠标选定目标，然后使用 Camshift 算法跟踪目标;

- 将目标的位置偏差值发送到 `Topic` --- “ chatter ” 上;

### **listener.cpp**
- 订阅 `Topic` --- “ chatter ” ;

- 将订阅信息通过串口传输到机器人控制器(如DSP)，从而控制机器人运行;

