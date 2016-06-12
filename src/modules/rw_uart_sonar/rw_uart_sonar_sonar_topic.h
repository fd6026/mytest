#ifndef __RW_UART_SONAR_H_
#define __RW_UART_SONAR_H_

#include <stdint.h>
#include <uORB/uORB.h>

/*声明主题，主题名自定义*/
ORB_DECLARE(rw_uart_sonar);

/* 定义要发布的数据结构体 */
struct rw_uart_sonar_data_s{
    char datastr[5];        //原始数据
    int data;               //解析数据，单位：mm
};



#endif