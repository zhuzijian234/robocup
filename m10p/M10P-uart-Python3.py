#!/usr/bin/env python
# -*- coding:UTF-8 -*-
from __future__ import print_function
import serial

# 定义一个函数来解析数据包
def parse_data(data,len):
    start_angle = (data[0]*256+data[1])/100.0  # 计算起始角度
    speed = 2500000/(data[2]*256+data[3]) # 计算转速
    distances = []  # 初始化距离列表

    # 遍历数据包中的距离数据
    for x in range(4, len-12, 2):
        if data[x] & 0x80:
            distance = (data[x] & 0x7F)*256+data[x+1]  # 计算距离
        else:
            distance = data[x] * 256 + data[x + 1]
#  打印某个角度范围的距离：
      #   if start_angle<280 and start_angle>260:
      #       print(distance)
        distances.append(distance)  # 添加到列表
    return speed, start_angle, distances

# 定义一个函数来打印数据包的内容
def print_data(speed, start_angle, distances,last_angle):
    if last_angle - start_angle > 100:
        print("*******************************")

    # 打印转速、起始角度和数据点
    print("转速:", speed, end="\t")
    print("起始角度:", start_angle, end="\t")
    print("数据【距离（mm）】：", end="\t")

    # 打印每个数据点的距离
    for distance in distances:
        print(distance, end="\t")
    print("\n")

# 主程序开始
if __name__ == '__main__':
    last_angle = 0  # 初始化上一个角度
    # ser = serial.Serial('/dev/wheeltec_lidar', 512000)    # ubuntu，如果未修改串口别名，可通过 ll /dev 查看雷达具体端口再进行更改
    ser = serial.Serial("COM11", 512000, timeout=5)          # window 通过设备管理器查看串口号
    # 循环读取数据
    while True:
        try:
            data = ser.read(1)  # 读取1个字节的数据
            if data[0] == 0xA5:
                data = ser.read(1)  # 读取1个字节的数据
                if data[0] == 0x5A:
                    data = ser.read(1)
                    if data[0] == 0x00:
                        data = ser.read(1)
                        list_len = data[0]  # 获取一帧总字节长度
                        data_len = list_len-4  # 除去帧头剩余长度
                        data = ser.read(data_len)  # 读取剩余长度的数据
                        # 解析和打印数据包
                        speed, start_angle, distances = parse_data(data, data_len)
                        print_data(speed, start_angle, distances, last_angle)
                        last_angle = start_angle  # 更新上一个角度
        except Exception as e:
            pass  # 忽略异常
