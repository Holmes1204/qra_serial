/**
 * @file node.h
 * @author holmes (hdasddfgg@126.com)
 * @brief the serial communication between stm32 and ros
 * @version 0.1
 * @date 2021-05-09
 * 
 * @copyright WTRobo Copyright (c) 2021
 * 
 */
#include <serial/serial.h>

#ifndef QRA_SERIAL_H
#define QRA_SERIAL_H
const double PI = 3.14159265;


extern serial::Serial sp;
extern const uint8_t ender[2], header[2];

int Serial_Init(const std::string &port,uint32_t baud_rate);
bool Serial_Transmit(uint8_t *buf,int len);
int Serial_Receive(uint8_t *buf, uint8_t len = 0);


#endif //QRA_SERIAL_H