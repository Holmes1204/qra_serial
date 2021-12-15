/**
 * @file main.cpp
 * @author holmes (hdasddfgg@126.com)
 * @brief the serial communication between stm32 and ros
 * @version 0.1
 * @date 2021-05-09
 * 
 * @copyright WTRobo Copyright (c) 2021
 * @todo 1. debug the receive function 2. the receive & transmit data structure 
 */
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <qra_serial.h>
#include <thread>

#include <wtr_serial/actor_1.h>
#include <wtr_serial/leg.h>

int stop = 0;
int main_run;



ros::Publisher* g_pub=nullptr;


int send_msg(int type,std::vector<double> msg){
    com_msg send_msg;
    for (int i = 0; i < 2; i++)
    {
        send_msg.data_msg.header[i] = HEADER[i];
    }
    send_msg.data_msg.id = type;
    switch (type)
    {
        case ANGLE:
            
            for (int i = 0; i < 6; i++)
            {
                send_msg.data_msg.decode_msg[i] = float64_to_uint16(msg[i],-M_PI,+M_PI);
            }
            send_msg.data_msg.crc = crc_check((uint8_t*)send_msg.data_msg.decode_msg,12);
            if(!Serial_Transmit(FONT,send_msg.raw_msg,16)) return 0;

            for (int i = 6; i < 12; i++)
            {
                send_msg.data_msg.decode_msg[i] = float64_to_uint16(msg[i],-M_PI,+M_PI);
            }
            send_msg.data_msg.crc = crc_check((uint8_t*)send_msg.data_msg.decode_msg,12);
            if(!Serial_Transmit(BACK,send_msg.raw_msg,16)) return 0;

            break;

        case VEL:

            for (int i = 0; i < 6; i++)
            {
                send_msg.data_msg.decode_msg[i] = float64_to_uint16(msg[i],-M_PI,+M_PI);
            }
            send_msg.data_msg.crc = crc_check((uint8_t*)send_msg.data_msg.decode_msg,12);
            if(!Serial_Transmit(FONT,send_msg.raw_msg,16)) return 0;

            for (int i = 6; i < 12; i++)
            {
                send_msg.data_msg.decode_msg[i] = float64_to_uint16(msg[i],-M_PI,+M_PI);
            }
            send_msg.data_msg.crc = crc_check((uint8_t*)send_msg.data_msg.decode_msg,12);
            if(!Serial_Transmit(BACK,send_msg.raw_msg,16)) return 0;

            break;

        case TORQUE:

            for (int i = 0; i < 6; i++)
            {
                send_msg.data_msg.decode_msg[i] = float64_to_uint16(msg[i],-M_PI,+M_PI);
            }
            send_msg.data_msg.crc = crc_check((uint8_t*)send_msg.data_msg.decode_msg,12);
            if(!Serial_Transmit(FONT,send_msg.raw_msg,16)) return 0;

            for (int i = 6; i < 12; i++)
            {
                send_msg.data_msg.decode_msg[i] = float64_to_uint16(msg[i],-M_PI,+M_PI);
            }
            send_msg.data_msg.crc = crc_check((uint8_t*)send_msg.data_msg.decode_msg,12);
            if(!Serial_Transmit(BACK,send_msg.raw_msg,16)) return 0;
            break;   
        
        default:
            return 0 ;
    }
    return 1;
}


void control_callback(const wtr_serial::leg::ConstPtr& msg_ptr)
{

    send_msg(ANGLE,msg_ptr->angle);
    send_msg(VEL,msg_ptr->vel);
    send_msg(TORQUE,msg_ptr->torque);
    
    return;
}


void STM32_Recieve_Handle()
{
    ros::Rate loop(50);
    static uint8_t buffer[50];
    clock_t delay = 0.1 * CLOCKS_PER_SEC, start; //convert to clock ticks
    wtr_serial::actor_1 msg;
    while (main_run)
    {
        if (Serial_Receive(FONT,buffer, 16))
        {
            for(int i=2; i<16;i++){
                //先找到一个完整的信息
                if(buffer[i-2]==0xFA&&(buffer[i-1]==0xA1||buffer[i-1]==0xA2)){
                    if(buffer[i-1]==0xA1){
                        msg.id = 0;
                    }
                    if(buffer[i-1]==0xA2){
                        msg.id = 1;
                    }

                    (*g_pub).publish(msg);
                    break;
                }
                i++;
            }
        }
        start = clock();
        while (clock() - start < delay);// time delay
    }
    return;
}



int main(int argc, char **argv) //argc是命令行总的参数个数 argv[]为保存命令行参数的字符串指针，其中第0个参数是程序的全名，以后的参数为命令行后面跟的用户输入的参数
{
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/quad", 1000, &control_callback); //订阅一条退的角度信息
    ros::Publisher pub = nh.advertise<wtr_serial::actor_1>("/leg_angle_feedback", 1000);
    ros::Rate loop_rate(100);
    ros::NodeHandle nh_private("~");
    g_pub = &pub;
    
    //同时打开初始化前后两条腿的串口
    const std::string port_font("/dev/ttyUSB0"),port_back("/dev/ttyUSB1");
    const int baud_rate = 115200;
    if(!Serial_Init(FONT,port_font,baud_rate)||!Serial_Init(BACK,port_back,baud_rate))return -1;

    main_run = 1;
    std::thread th(STM32_Recieve_Handle);
    th.detach();
    //只用于中断的接受
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    main_run = 0;
    return 0;
}