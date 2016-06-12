#include <px4_posix.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>

//#include "src/drivers/px4flow/i2c_frame.h"

#include "rw_uart_sonar_topic.h"





/* 定义主题 */
ORB_DEFINE(rw_uart_sonar, struct rw_uart_sonar_data_s);

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;


__EXPORT int rw_uart_main(int argc, char *argv[]);
int rw_uart_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);
static void usage(const char *reason);

#define PX4FLOW_MAX_DISTANCE 5.0f
#define PX4FLOW_MIN_DISTANCE 0.3f


typedef struct i2c_integral_frame {
    uint16_t frame_count_since_last_readout;
    int16_t pixel_flow_x_integral;
    int16_t pixel_flow_y_integral;
    int16_t gyro_x_rate_integral;
    int16_t gyro_y_rate_integral;
    int16_t gyro_z_rate_integral;
    uint32_t integration_timespan;
    uint32_t sonar_timestamp;
    uint16_t ground_distance;
    int16_t gyro_temperature;
    uint8_t qual;
} i2c_integral_frame;



uint8_t buf[64];
int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 1152: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config);
    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;
    /* no parity, one stop bit */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int uart_init(const char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY );

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}

static void usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "usage: rw_uart {start|stop|status} [param]\n\n");
    exit(1);
}

int rw_uart_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("[YCM]missing command");
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            warnx("[YCM]already running\n");
            exit(0);
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("rw_uart",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_MAX - 5,
                         2000,
                         rw_uart_thread_main,
                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("[YCM]running");

        } else {
            warnx("[YCM]stopped");
        }

        exit(0);
    }

    usage("unrecognized command");
    exit(1);
}

int rw_uart_thread_main(int argc, char *argv[])
{

    if (argc < 2) {
        errx(1, "[YCM]need a serial port name as argument");
        usage("eg:");
    }

    const char *uart_name = argv[1];

    warnx("[YCM]opening port %s", uart_name);
    char data = '0';
    char buffer[30] = "";
    //uint16_t send_c,receiver_c;
   // send_c = 1000;
   // receiver_c = 0;

    /* * TELEM1 : /dev/ttyS1 * TELEM2 : /dev/ttyS2 * GPS : /dev/ttyS3 * NSH : /dev/ttyS5 * SERIAL4: /dev/ttyS6 * N/A : /dev/ttyS4 * IO DEBUG (RX only):/dev/ttyS0 */
    int uart_read = uart_init(uart_name);
  
    if(false == uart_read)return -1;
    if(false == set_uart_baudrate(uart_read,1152)){
        printf("[YCM]set_uart_baudrate is failed\n");
        return -1;
    }
    printf("[YCM]uart init is successful\n");

    thread_running = true;

    /*初始化数据结构体 */
    struct rw_uart_sonar_data_s sonardata;
    struct optical_flow_s report;
//    struct distance_sensor_s distance_report;
   // struct optical_flow_s report;

 //   ssize_t nread = 0;
    memset(&sonardata, 0, sizeof(sonardata));

    i2c_integral_frame my_dat_struct;
    //unsigned char *pbuff;
    //struct i2c_integral_frame* p_my_dat_struct;
    memset(&my_dat_struct, 0, sizeof(my_dat_struct));
    
    //p_my_dat_struct = &my_dat_struct;

    /* 公告主题 */
   // orb_advert_t rw_uart_sonar_pub = orb_advertise(ORB_ID(rw_uart_sonar), &sonardata);
  //  orb_advert_t _px4flow_topic = orb_advertise(ORB_ID(rw_uart_sonar), &sonardata);
    orb_advert_t _px4flow_topic = orb_advertise(ORB_ID(optical_flow), &report);
 //   orb_advert_t _distance_sensor_topic = orb_advertise(ORB_ID(distance_sensor), &distance_report);


usleep(10000000);
usleep(10000000);



    while(!thread_should_exit)
        {  
           // printf("first:\n");

            usleep(85000);
            data = 0x16;
            // printf("secondsecond:\n");
            
            write(uart_read,&data,1);
            //if(0 == (send_c--))break; hd debug
           // printf("thirdthirdthird:\n");
            usleep(10000);
           // printf("fourthfourthfourthfourth:\n");
            
            data = 0;

            while(1 == read(uart_read,&data,1))
            {            


             if(data == 0xFE)
                {
             //       printf("fivefivefivefivefivefivefivefive:\n");
                    read(uart_read,&data,1);
                    if(data == 0x1c)
                    {                    
                        for(int i = 0;i <25;++i)
                        {
                                read(uart_read,&data,1);
                                buffer[i] = data;                   
                                data = '0';
                        }
                my_dat_struct.frame_count_since_last_readout = buffer[0] + (buffer[1]<<8);
                my_dat_struct.pixel_flow_x_integral          = buffer[2] + (buffer[3]<<8);
                my_dat_struct.pixel_flow_y_integral          = buffer[4] + (buffer[5]<<8);
                my_dat_struct.gyro_x_rate_integral           = buffer[6] + (buffer[7]<<8);
                my_dat_struct.gyro_y_rate_integral           = buffer[8] + (buffer[9]<<8);
                my_dat_struct.gyro_z_rate_integral           = buffer[10]+ (buffer[11]<<8);
                my_dat_struct.integration_timespan           = buffer[12]+ (buffer[13]<<8) + (buffer[14]<<16) + (buffer[15]<<24);
                my_dat_struct.sonar_timestamp                = buffer[16] + (buffer[17]<<8) + (buffer[18]<<16) + (buffer[19]<<24);
                my_dat_struct.ground_distance                = buffer[20] + (buffer[21]<<8);
                my_dat_struct.gyro_temperature               = buffer[22] + (buffer[23]<<8);
                my_dat_struct.qual                           = buffer[24] ;
               // printf("fivefivefivefivefivefivefivefive:*********************************\n");

    report.timestamp = hrt_absolute_time();

    report.pixel_flow_x_integral = (float)(my_dat_struct.pixel_flow_x_integral) / 10000.0f;//convert to radians

    report.pixel_flow_y_integral = -(float)(my_dat_struct.pixel_flow_y_integral) / 10000.0f;//convert to radians

    report.frame_count_since_last_readout = my_dat_struct.frame_count_since_last_readout;

    report.ground_distance_m = (float)(my_dat_struct.ground_distance) / 1000.0f;//convert to meters

    report.quality = my_dat_struct.qual; //0:bad ; 255 max quality

    report.gyro_x_rate_integral = (float)(my_dat_struct.gyro_x_rate_integral) / 10000.0f; //convert to radians

    report.gyro_y_rate_integral = (float)(my_dat_struct.gyro_y_rate_integral) / 10000.0f; //convert to radians

    report.gyro_z_rate_integral = (float)(my_dat_struct.gyro_z_rate_integral) / 10000.0f; //convert to radians

    report.integration_timespan = my_dat_struct.integration_timespan; //microseconds

    report.time_since_last_sonar_update = my_dat_struct.sonar_timestamp;//microseconds

    report.gyro_temperature = my_dat_struct.gyro_temperature;//Temperature * 100 in centi-degrees Celsius

    report.sensor_id = 0;
    

                /* publish to the distance_sensor topic as well */
    
 //   distance_report.timestamp = report.timestamp;
 //   distance_report.min_distance = PX4FLOW_MIN_DISTANCE;
  //  distance_report.max_distance = PX4FLOW_MAX_DISTANCE;
  //  distance_report.current_distance = report.ground_distance_m;
  //  distance_report.covariance = 0.0f;
 //   distance_report.type = 1; //sonar type
    /* TODO: the ID needs to be properly set */
 //   distance_report.id = 0;
 //   distance_report.orientation = 8;

    orb_publish(ORB_ID(optical_flow), _px4flow_topic, &report);

//    orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &distance_report);


            //strncpy(sonardata.datastr,buffer,4);
            // sonardata.data = atoi(sonardata.datastr);
            //sonardata.data = my_dat_struct.ground_distance;

            //printf("[YCM]sonar.data=%d\n",sonardata.data);
            //orb_publish(ORB_ID(rw_uart_sonar), rw_uart_sonar_pub, &sonardata);
            //receiver_c++; hd debug

                  break;
                    }                      //    usleep(1000000);            

                }
            }
        }
       // printf("send_c =%d\n",receiver_c);hd debug
    warnx("[YCM]exiting");
    thread_running = false;
    close(uart_read);

    fflush(stdout);
    return 0;
}










/*
    report.timestamp = hrt_absolute_time();
    report.pixel_flow_x_integral = 0.01;
    report.pixel_flow_y_integral = 0.02;
    report.frame_count_since_last_readout += 1;
    report.ground_distance_m = 2.1;
    report.quality =254;
    report.gyro_x_rate_integral = 0; //convert to radians
    report.gyro_y_rate_integral = 0; //convert to radians
    report.gyro_z_rate_integral = 0; //convert to radians
    report.integration_timespan = 95000; //microseconds
    report.time_since_last_sonar_update = 95000;//microseconds
    report.gyro_temperature =3700;//Temperature * 100 in centi-degrees Celsius
    report.sensor_id = 0;        
*/

                

    //}

    /*    memcpy(&my_dat_struct, &buffer[1], 20);

    printf("[YCM]set_uart_baudrate is failed%d\n",my_dat_struct.pixel_flow_y_integral);
        printf("[YCM]set_uart_baudrate is failed%d\n",my_dat_struct.gyro_x_rate_integral);
        printf("[YCM]set_uart_baudrate is failed%d\n",my_dat_struct.gyro_y_rate_integral);
        printf("[YCM]set_uart_baudrate is failed%d\n",my_dat_struct.gyro_z_rate_integral);
        printf("[YCM]set_uart_baudrate is failed%d\n",my_dat_struct.integration_timespan);
        printf("[YCM]set_uart_baudrate is failed%d\n",my_dat_struct.sonar_timestamp);
        printf("[YCM]set_uart_baudrate is failed%d\n",my_dat_struct.ground_distance);
        printf("[YCM]set_uart_baudrate is failed%d\n",my_dat_struct.gyro_temperature);
        printf("[YCM]set_uart_baudrate is failed%d\n",my_dat_struct.qual);
        printf("[YCM]\n");
     */   
