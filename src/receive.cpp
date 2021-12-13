#include "node.h"
#include <ctime>


int main(int argc, char **argv) //argc是命令行总的参数个数 argv[]为保存命令行参数的字符串指针，其中第0个参数是程序的全名，以后的参数为命令行后面跟的用户输入的参数
{
    static uint8_t buffer[100];
    clock_t delay = 0.1 * CLOCKS_PER_SEC, start; //convert to clock ticks
    std::string port("/dev/ttyUSB0");
    if (!Serial_Init(port,115200))
        return -1;
    while (true)
    {
        //int a = Serial_Receive(buffer,100);
        //printf("%d\n",a);
        if (Serial_Receive(buffer, 16))
        {
            for (int i = 0; i < 16; i++)
            {
                std::cout<<int(buffer[i])<<std::endl;
            }
            std::cout<<"----------------"<<std::endl;
        }
        start = clock();
        //while (clock() - start < delay);
    }
    return 0;
}