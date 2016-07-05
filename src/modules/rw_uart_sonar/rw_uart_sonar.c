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

//#include "rw_uart_sonar_topic.h"





/* 定义主题 */
//ORB_DEFINE(rw_uart_sonar, struct rw_uart_sonar_data_s);

static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;


__EXPORT int rw_uart_sonar_main(int argc, char *argv[]);
int rw_uart_sonar_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);
static void usage(const char *reason);

#define PX4FLOW_MAX_DISTANCE 7.65f
#define PX4FLOW_MIN_DISTANCE 0.3f


//uint8_t buf[64];
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

int rw_uart_sonar_main(int argc, char *argv[])
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
                         rw_uart_sonar_thread_main,
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

int rw_uart_sonar_thread_main(int argc, char *argv[])
{

    if (argc < 2) {
        errx(1, "[YCM]need a serial port name as argument");
        usage("eg:");
    }

    const char *uart_name = argv[1];

    warnx("[YCM]opening port %s", uart_name);
    char data = '0';
    //uint16_t send_c,receiver_c;
   // send_c = 1000;
   // receiver_c = 0;

    /* * TELEM1 : /dev/ttyS1 * TELEM2 : /dev/ttyS2 * GPS : /dev/ttyS3 * NSH : /dev/ttyS5 * SERIAL4: /dev/ttyS6 * N/A : /dev/ttyS4 * IO DEBUG (RX only):/dev/ttyS0 */
    int uart_read = uart_init(uart_name);
  
    if(false == uart_read)return -1;
    if(false == set_uart_baudrate(uart_read,9600)){
        printf("[YCM]set_uart_baudrate is failed\n");
        return -1;
    }
    printf("[YCM]uart init is successful\n");

    thread_running = true;

    /*初始化数据结构体 */
  //  struct rw_uart_sonar_data_s sonardata;
 //   struct optical_flow_s report;
    struct distance_sensor_s distance_report;
   // struct optical_flow_s report;

 //   ssize_t nread = 0;
  //  memset(&sonardata, 0, sizeof(sonardata));

//    i2c_integral_frame my_dat_struct;
    //unsigned char *pbuff;
    //struct i2c_integral_frame* p_my_dat_struct;
 //   memset(&my_dat_struct, 0, sizeof(my_dat_struct));
    
    //p_my_dat_struct = &my_dat_struct;

    /* 公告主题 */
   // orb_advert_t rw_uart_sonar_pub = orb_advertise(ORB_ID(rw_uart_sonar), &sonardata);
  //  orb_advert_t _px4flow_topic = orb_advertise(ORB_ID(rw_uart_sonar), &sonardata);
//    orb_advert_t _px4flow_topic = orb_advertise(ORB_ID(optical_flow), &report);
    orb_advert_t _distance_sensor_topic = orb_advertise(ORB_ID(distance_sensor), &distance_report);


//usleep(10000000);
//usleep(10000000);



    while(!thread_should_exit)
        {  
           // printf("first:\n");
            unsigned int distanc = 0;

             //distanc = dis_h*256 +dis_l;           
            //usleep(10000);

            while(1 == read(uart_read,&data,1))
            {
               // printf("-%d",data );                
               // printf("\n",data );

              if(data == 'R')
                {
                 //   printf("*",distanc );

                    read(uart_read,&data,1);
                    //data = 0x7d - data;
                    distanc = (data-'0')*100;
                    read(uart_read,&data,1);
                    //data = 0x7d - data;
                    distanc += (data-'0')*10;
                    read(uart_read,&data,1);
                   // data = 0x7d - data;
                    distanc +=data-'0';

// printf("%d\n",distanc );
                      distanc *= 10;  //unit convert;

    distance_report.timestamp = hrt_absolute_time();
    distance_report.min_distance = PX4FLOW_MIN_DISTANCE;
    distance_report.max_distance = PX4FLOW_MAX_DISTANCE;
    distance_report.current_distance = (float)(distanc)/1000.0f;
    distance_report.covariance = 0.0f;
    distance_report.type = 1; //sonar type
    /* TODO: the ID needs to be properly set */
    distance_report.id = 0;
    distance_report.orientation = 8; 

    orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &distance_report);
                } 
               

  

            }          
     
                
        }        
      
    warnx("[YCM]exiting");
    thread_running = false;
    close(uart_read);

    fflush(stdout);
    return 0;
}










