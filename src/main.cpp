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

int stop = 0;
int main_run;
double motor_feedback_data[3];
uint8_t motor_target_angle[8];


ros::Publisher* g_pub=nullptr;



void angle_callback(const wtr_serial::actor_1& msg)
{
    double angle[3];
    angle[0] = msg.agl_0; 
    angle[1] = msg.agl_1; 
    angle[2] = msg.agl_2; 
    motor_target_angle[0] = 0xFA;
    motor_target_angle[1] = 0x00;
    if (msg.id == 0)
    {
        motor_target_angle[1] = 0xA1;
    }
    if (msg.id == 1)
    {
        motor_target_angle[1] = 0xA2;
    }
    motor_target_angle[2] = ((int)angle[0])>>8;
    motor_target_angle[3] = ((uint8_t)angle[0]);
    motor_target_angle[4] = ((int)angle[1])>>8;
    motor_target_angle[5] = ((uint8_t)angle[1]);
    motor_target_angle[6] = ((int)angle[2])>>8;
    motor_target_angle[7] = ((uint8_t)angle[2]);
    if(!Serial_Transmit(motor_target_angle,8)) return ;
    //comment as debug code
    /*
    for (int i = 0; i < 3; i++)
    {
        std::cout<<angle[i]<<std::endl;
    }
    for (int i = 0; i < 8; i++)
    {
        std::cout<<(int)motor_target_angle[i]<<std::endl;
    }
    std::cout<<"----------"<<std::endl;
    */
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
        if (Serial_Receive(buffer, 16))
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
                    motor_feedback_data[0] = ((int)buffer[i]<<8)|buffer[i+1];
                    motor_feedback_data[1] = ((int)buffer[i+2]<<8)|buffer[i+3];
                    motor_feedback_data[2] = ((int)buffer[i+4]<<8)|buffer[i+5];
                    msg.agl_0=motor_feedback_data[0];
                    msg.agl_0=motor_feedback_data[1];
                    msg.agl_0=motor_feedback_data[2];
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
    ros::Subscriber sub = nh.subscribe("/quad", 1000, &angle_callback); //订阅一条退的角度信息
    ros::Publisher pub = nh.advertise<wtr_serial::actor_1>("/leg_angle_feedback", 1000);
    ros::Rate loop_rate(100);
    ros::NodeHandle nh_private("~");
    g_pub = &pub;
    std::string port;
    int baud_rate;

    nh_private.param<std::string>("serial_port",port,"/dev/ttyUSB0");
    nh_private.param<int>("serial_baudrate",baud_rate,115200);

    if(!Serial_Init(port,baud_rate))return -1;

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