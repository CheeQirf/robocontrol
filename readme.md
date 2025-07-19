串口4 DMA发送 CAN1 CAN2 中断接收测试过了
# 日志使用方法
log_i 信息层级
log_e 错误层级
log_w 警告信息
**不要使用printf 如果要非日志打印 使用myprintf（）**

# 线程TODO
## 1. CAN处理线程 
can中断信号量发送到解析线程，线程中把每个消息发至对应数据结构体 
需要文件app目录先can_task.c can_task.h
## 2. RS485通信线程 和 宇树 电机通信
## 3. 控制线程


# TODO
## 1. 先把每个CAN设备的数据获取
## 2. PID调参
## 3.控制逻辑