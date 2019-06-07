# Kalman implementation in Keil
## kalman算法
5个公式，2个预测，3个更新
<div align=center>![](https://github.com/bbidong/kalman-in-keil/blob/master/images/kalman.jpg)
- Q:过程噪声协方差
R:测量噪声协方差
- Q/R决定响应速度，比值越大，响应越快
调试时，可以固定Q的值，不断调整R的值
## Documentation
- **kalman.c**：kalman tool
- **gimbal_task.c**：用kalman算法控制云台yaw轴
## Details
云台yaw只有个角度值，根据上位机发送的两个相邻角度值计算速度，选择状态空间为$$x={yaw_angle,yaw_v}^T$$
