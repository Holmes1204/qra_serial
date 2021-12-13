/**
 * @file node.cpp
 * @author holmes (hdasddfgg@126.com)
 * @brief the serial communication between stm32 and ros
 * @version 0.1
 * @date 2021-05-09
 * 
 * @copyright WTRobo Copyright (c) 2021
 * 
 */
#include <qra_serial.h>
#include <iostream>
const uint8_t ender[2] = {0x0d, 0x0a}; //定义消息头
const uint8_t header[2] = {0x55, 0xaa};

serial::Serial sp;



int Serial_Init(const std::string &port,uint32_t baud_rate) //串口初始化
{
    //创建timeout
    static serial::Timeout time_out = serial::Timeout::simpleTimeout(100);
    //static do not release time_out until the code finish
    //设置要打开的串口名称
    sp.setPort(port);
    //设置串口通信的波特率
    sp.setBaudrate(baud_rate);
    //串口设置timeout
    sp.setTimeout(time_out);
    try
    {
        //打开串口
        sp.open();
        //判断串口是否打开成功
        if (sp.isOpen())
        {
            //ROS_INFO_STREAM(port<<" is open"<<std::endl);
            std::cout << "port: " << port << " is open" << std::endl;
            return 1;
        }
        else
        {
            return 0;
        }
    }
    catch (serial::IOException &e) //捕捉输入输出异常
    {
        //ROS_ERROR_STREAM("Unable to open port:"<<port<<std::endl);
        std::cout << "U nable to open port: " << port << std::endl;
        return 0;
    }
}

//error handle

bool Serial_Transmit(uint8_t *buf,int len)
{
    if (sp.write(buf, len) == len)
    {
        return true;
    }
    return false;
}

int Serial_Receive(uint8_t *buf, uint8_t len)
{
    try
    {
        uint8_t num = len ? len : sp.available();
        if (num && sp.available() >= num)
        {
            return sp.read(buf, num);
        }
        return 0;
    }
    catch (serial::IOException &e)
    {
        //ROS_INFO("read_until error");
        return 0;
    }
}

