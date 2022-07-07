#include "test_oneleg.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>

int can_sock[4];

struct motor_data_t
{
    float q_abad[4];//内外翻电机的角度
    float q_hip[4];//大腿电机的角度
    float q_knee[4];//膝盖电机的角度

    float qd_abad[4];//内外翻电机的速度
    float qd_hip[4];//大腿翻电机的速度
    float qd_knee[4];//膝盖电机的速度

    float tau_abad[4];//内外翻电机的扭矩
    float tau_hip[4];;//大腿电机的扭矩
    float tau_knee[4];;//膝盖电机的扭矩

    float temp_abad[4];//内外翻电机的温度
    float temp_hip[4];//大腿电机的温度
    float temp_knee[4];//膝盖电机的温度

    unsigned char alarm_abad[4];//内外翻电机的报警
    unsigned char alarm_hip[4];//大腿电机的报警
    unsigned char alarm_knee[4];//膝盖电机的报警
};

struct motor_command_t
{
    float tau_abad[4];//内外翻电机的扭矩
    float tau_hip[4];//大腿电机的扭矩
    float tau_knee[4];//膝盖电机的扭矩
};

struct motor_current_t
{
    float cur_abad[4];//内外翻电机的电流
    float cur_hip[4];//大腿电机的电流
    float cur_knee[4];//膝盖电机的电流
};

struct motor_data_get
{
    uint16_t position;//位置
    uint16_t speed;//速度
    uint16_t current;//电流
    float temperature;//温度
};

int init_can_socket(void) 
{
    int sock;
    struct sockaddr_can addr;
    struct ifreq ifr;
    const char *ifname = "can0";

    if((can_sock[0] = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW)) < 1) {

        printf("Error while opening socket\r\n");

        return -1;
    }
    
    printf("socket successfully established, sock = %d\n", can_sock[0]);

    strcpy(ifr.ifr_name, ifname);

    ioctl(can_sock[0], SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;

    addr.can_ifindex = ifr.ifr_ifindex;

    printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

    if(bind(can_sock[0], (struct sockaddr *)&addr, sizeof(addr)) < 0) {

        printf("Error in socket bind\r\n");

        return -2;
    }
    else{
        printf("bind socket success!\r\n");
    }

    return 0;
}

int can_frame_init(struct can_frame *frame, int can_id, int can_dlc)
{
    frame->can_id = can_id;

    frame->can_dlc = can_dlc;

    return 0;
}

int can_send(int sock, struct can_frame frame)
{
    int nbytes;
    int i;

    //拷贝buff到发送缓冲区
    //memcpy(frame.data, buf, size);

    for(i = 0; i < frame.can_dlc; i++) {
        printf("send buf = %#x\t", frame.data[i]);
    }

    printf("\n");

    // printf("frame.can_id = %d, frame.can_dlc = %d\n", frame.can_id, frame.can_dlc);
    

    //发送buff
    nbytes = write(sock, &frame, sizeof(struct can_frame));

    return nbytes;
}

int can_read_analysis(struct motor_data_t *motor_data)
{
    //临时数据，如果后面更新了KT值，则此表格也需要同步更新，包括下面代码中for循环的迭代次数
    float kt[13] =         {1, 1.7, 1.8, 1.7, 1.9, 1.9,  1.9,  1.9,  1.9,  1.7,  1.7,  1.5,  1.5};
    float kt_current[13] = {0, 0.3, 2.8, 5.8, 8.0, 10.5, 13.3, 15.7, 21.4, 28.7, 35.5, 47.1, 54.7};
    float current, fabs_current;
    uint16_t position_hex, speed_hex, current_hex, temperature_hex, alarm_hex;
    int i;
    int nbytes;
    struct can_frame read_frame;
    struct can_frame *frame;
    frame = &read_frame;

    // printf("开始接收数据\n");

    nbytes = read(can_sock[0], frame, sizeof(struct can_frame));
    if(nbytes > 0) {
        //开始数据解析
        // printf("本次接收%d个数据\n", frame->can_dlc);
        // for(i = 0; i < frame->can_dlc; i++) {
        //     printf("frame_buf[%d] = %#x\n", i, frame->data[i]);
        // }
        // printf("\n");
        if((frame->data[0] & 0xe0) == 0x20 && (frame->can_id >= 1 || frame->can_id <= 3)) {//电机id设置为1~3

            alarm_hex = frame->data[0] & 0x1f;
            position_hex = frame->data[1] << 8 | frame->data[2];
            speed_hex = (frame->data[3] << 4) | (frame->data[4] >> 4 & 0x0f);
            current_hex = ((frame->data[4] & 0x0f) << 8) | frame->data[5];
            temperature_hex = frame->data[6];

            /*******更新电机信息到结构体中*****/
            switch (frame->can_id)
            {
                case 1:
                    //内外翻电机
                    motor_data->q_abad[0] = (float)(position_hex - 32768) * (12.5 / 32768);
                    motor_data->qd_abad[0] = (float)(speed_hex - 2048) * (18.0 / 2048);

                    current = (float)(current_hex - 2048) * (30.0 / 2048);
                    // printf("current=%f\r\n", current);
                    fabs_current = fabs(current);
                    // printf("fabs_current=%f\r\n", fabs_current);
                    for (i = 0; i <= 11; i++)
                    {
                         if ((fabs_current >= kt_current[i]) && (fabs_current < kt_current[i + 1]))
                         {
                            break;
                         }
                    }
                    // printf("i=%d\r\n", i);
                    if (motor_data->qd_abad[0] < 0)
                    {
                        motor_data->tau_abad[0] = current * kt[i + 1] * -1;
                    }
                    else
                    {
                        motor_data->tau_abad[0] = current * kt[i + 1];
                    }
                    
                    // printf("motor_data->tau_abad[0]=%f\r\n", motor_data->tau_abad[0]);
                    motor_data->temp_abad[0] = (float)(temperature_hex - 50) / 2.0;
                    motor_data->alarm_abad[0] = alarm_hex;
                    break;
                case 2:
                    //大腿电机
                    motor_data->q_hip[0] = (float)(position_hex - 32768) * (12.5 / 32768);
                    motor_data->qd_hip[0] = (float)(speed_hex - 2048) * (18.0 / 2048);
                    
                    current = (float)(current_hex - 2048) * (30.0 / 2048);
                    fabs_current = fabs(current);
                    for (i = 0; i <= 11; i++)
                    {
                         if ((fabs_current >= kt_current[i]) && (fabs_current < kt_current[i + 1]))
                         {
                            break;
                         }
                    }
                    if (motor_data->qd_hip[0] < 0)
                    {
                        motor_data->tau_hip[0] = current * kt[i + 1] * -1;
                    }
                    else
                    {
                        motor_data->tau_hip[0] = current * kt[i + 1];
                    }
                    motor_data->temp_hip[0] = (float)(temperature_hex - 50) / 2.0;
                    motor_data->alarm_hip[0] = alarm_hex;
                    break;
                case 3:
                    //膝盖电机
                    motor_data->q_knee[0] = (float)(position_hex - 32768) * (12.5 / 32768);
                    motor_data->qd_knee[0] = (float)(speed_hex - 2048) * (18.0 / 2048);
                    
                    current = (float)(current_hex - 2048) * (30.0 / 2048);
                    fabs_current = fabs(current);
                    for (i = 0; i <= 11; i++)
                    {
                         if ((fabs_current >= kt_current[i]) && (fabs_current < kt_current[i + 1]))
                         {
                            break;
                         }
                    }
                    if (motor_data->qd_knee[0] < 0)
                    {
                        motor_data->tau_knee[0] = current * kt[i + 1] * -1;
                    }
                    else
                    {
                        motor_data->tau_knee[0] = current * kt[i + 1];
                    }
                    motor_data->temp_knee[0] = (float)(temperature_hex - 50) / 2.0;
                    motor_data->alarm_knee[0] = alarm_hex;
                    break;    
                default:
                    break;
            }
        }
        else {
            printf("CAN id超过预设值，frame->can_id = %d\n", frame->can_id);
        }

        if (frame->can_id == 1)
        {
        printf("Received motor_id = %d, pos = %.3f, spd = %.3f, tau = %.3f, temp. = %.3f, alarm = %d\n",
                frame->can_id,
                motor_data->q_abad[0],
                motor_data->qd_abad[0],
                motor_data->tau_abad[0],
                motor_data->temp_abad[0],
                motor_data->alarm_abad[0]);
        }
        else if (frame->can_id == 2)
        {
        printf("Received motor_id = %d, pos = %.3f, spd = %.3f, tau = %.3f, temp. = %.3f, alarm = %d\n",
                frame->can_id,
                motor_data->q_hip[0],
                motor_data->qd_hip[0],
                motor_data->tau_hip[0],
                motor_data->temp_hip[0],
                motor_data->alarm_hip[0]);
        }
        else if (frame->can_id == 3)
        {
        printf("Received motor_id = %d, pos = %.3f, spd = %.3f, tau = %.3f, temp. = %.3f, alarm = %d\n",
                frame->can_id,
                motor_data->q_knee[0],
                motor_data->qd_knee[0],
                motor_data->tau_knee[0],
                motor_data->temp_knee[0],
                motor_data->alarm_knee[0]);
        }

    }
    else {
        printf("No data received, nbytes = %d\n", nbytes);
        return -1;
    }

    return 0;
}

void sysUsecTime(float *tv_usec)
{
    struct timeval tv;
    struct timezone tz;

    struct tm *p;

    gettimeofday(&tv, &tz);

    p = localtime(&tv.tv_sec);
    // printf("time_now:%d%d%d%d%d%d.%ld\n", 1900+p->tm_year, 1+p->tm_mon, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec, tv.tv_usec);
    // printf("tv_usec:%ld\n", tv.tv_usec);
    *tv_usec = tv.tv_usec;
}

int set_motor_zero_position(uint16_t can_id)
{
    struct can_frame set_zero_frame;

    struct can_frame read_frame;

    int ret;

    //设置电机零点，初始化数据字段
    set_zero_frame.can_id = 0x7ff;

    set_zero_frame.can_dlc = 4;

    set_zero_frame.data[0] = can_id >> 8;

    set_zero_frame.data[1] = can_id & 0xff;

    set_zero_frame.data[2] = 0;

    set_zero_frame.data[3] = 0x03;

    ret = write(can_sock[0], &set_zero_frame, sizeof(struct can_frame));
    if (ret < 0)
    {
        return ret;
    }

    ret = read(can_sock[0], &read_frame, sizeof(struct can_frame));
    if (ret < 0)
    {
        return ret;
    }

    //需要判断返回值是否正确，才能确定设置零点是否成功
    if (read_frame.data[0] == (can_id >> 8)  && read_frame.data[1] == (can_id & 0xff) && read_frame.data[2] == 0x01 && read_frame.data[3] == 0x03)
    {
        printf("设置零点成功，电机ID=%d\r\n", can_id);
        return 0;
    }
    else
    {
        for(int i = 0; i < read_frame.can_dlc; i++) {
            printf("frame_buf = %#x\n", read_frame.data[i]);
        }
        printf("\n");
        printf("设置零点失败，电机ID=%d\r\n", can_id);
        return -1;
    }
}


int motor_send_receive(struct motor_command_t *command, struct motor_data_t *motor_data)
{
                        // 5    10   15   20   25   30   35   40   45   50   55   60   65   70   75   80   85   90   95  100  105  110  115  120  125  130  135  140
    float motor_kt[50] = {1.8, 1.7, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9, 1.9, 1.7, 1.7, 1.7, 1.7, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5};

    char buf_receive[1024];

    int index, ret;

    int16_t current_hex;

    struct motor_current_t motor_currnt;

    struct can_frame write_frame, read_frame;

    //如果是4条腿，那就循环4次，如果是测试1条腿，那需要把4改成1
    for (int i = 0; i < 1; i++)
    {
        // printf("初始化狗腿子扭矩\n");
        if (command->tau_abad[i] > 140 || command->tau_abad[i] < -140)
        {
            return -1;
        }
        // printf("cmd, legID = %d,内外翻扭矩=%f\r\n", i+1, command->tau_abad[i]);

        if (command->tau_hip[i] > 140 || command->tau_hip[i] < -140)
        {
            return -1;
        }
        // printf("cmd, legID = %d,大腿扭矩=%f\r\n", i+1, command->tau_hip[i]);

        if (command->tau_knee[i] > 140 || command->tau_knee[i] < -140)
        {
            return -1;
        }
        // printf("cmd, legID = %d,膝盖扭矩=%f\r\n", i+1, command->tau_knee[i]);

        //发送内外翻电机的电流模式命令帧，（目前定义CAN ID：1）
        index = fabs(command->tau_abad[i]) / 5;//通过查表法，获取电机实测的KT系数，然后通过KT值，将扭矩转换为电流
        motor_currnt.cur_abad[i] = command->tau_abad[i] / motor_kt[index];
        current_hex = motor_currnt.cur_abad[i] * 100;

        write_frame.data[0] = 0x61;//电流模式，返回报文类型1
        write_frame.data[1] = current_hex >> 8;
        write_frame.data[2] = current_hex;

        write_frame.can_id = 1;
        write_frame.can_dlc = 3;
        // printf("内外翻电机, 计算完成后的电流，current_hex = %d\r\n", current_hex);
        ret = write(can_sock[0], &write_frame, sizeof(struct can_frame));
        if (ret < 0)
        {
            printf("发送内外翻电机的电流模式命令帧， 失败！！！, 第%d腿，ret=%d\r\n", i + 1, ret);
            //return ret;
        }
        else {
            // printf("发送内外翻电机的电流模式命令帧， 成功！, 第%d腿，current_hex=%d\r\n", i + 1, current_hex);
        }

        ret = can_read_analysis(motor_data);
        if (ret < 0)
        {
            printf("接收内外翻电机的电流模式数据帧， 失败！！！, 第%d腿，ret=%d\r\n", i + 1, ret);
            //return ret;
        }

        //发送大腿电机的电流模式命令帧，（目前定义CAN ID：2）
        index = fabs(command->tau_hip[i]) / 5;//通过查表法，获取电机实测的KT系数，然后通过KT值，将扭矩转换为电流
        motor_currnt.cur_hip[i] = command->tau_hip[i] / motor_kt[index];
        current_hex = motor_currnt.cur_hip[i] * 100;

        write_frame.data[0] = 0x61;//电流模式，返回报文类型1
        write_frame.data[1] = current_hex >> 8;
        write_frame.data[2] = current_hex;

        write_frame.can_id = 2;
        write_frame.can_dlc = 3;
        // printf("大腿电机, 计算完成后的电流，current_hex = %d\r\n", current_hex);
        ret = write(can_sock[0], &write_frame, sizeof(struct can_frame));
        if (ret < 0)
        {
            printf("发送大腿电机的电流模式命令帧， 失败！！！, 第%d腿，ret=%d\r\n", i + 1, ret);
            //return ret;
        }
        else {
            // printf("发送大腿电机的电流模式命令帧， 成功！, 第%d腿，current_hex=%d\r\n", i + 1, current_hex);
        }

        ret = can_read_analysis(motor_data);
        if (ret < 0)
        {
            printf("接收大腿电机的电流模式数据帧， 失败！！！, 第%d腿，ret=%d\r\n", i + 1, ret);
            //return ret;
        }

        //发送膝盖电机的电流模式命令帧，（目前定义CAN ID：3）
        index = fabs(command->tau_knee[i]) / 5;//通过查表法，获取电机实测的KT系数，然后通过KT值，将扭矩转换为电流
        motor_currnt.cur_knee[i] = command->tau_knee[i] / motor_kt[index];
        current_hex = motor_currnt.cur_knee[i] * 100;

        write_frame.data[0] = 0x61;//电流模式，返回报文类型1
        write_frame.data[1] = current_hex >> 8;
        write_frame.data[2] = current_hex;

        write_frame.can_id = 3;
        write_frame.can_dlc = 3;
        // printf("膝盖电机, 计算完成后的电流，current_hex = %d\r\n", current_hex);
        ret = write(can_sock[0], &write_frame, sizeof(struct can_frame));
        if (ret < 0)
        {
            printf("发送膝盖电机的电流模式命令帧， 失败！！！, 第%d腿，ret=%d\r\n", i + 1, ret);
            //return ret;
        }
        else {
            // printf("发送膝盖电机的电流模式命令帧， 成功！, 第%d腿，current_hex=%d\r\n", i + 1, current_hex);
        }

        ret = can_read_analysis(motor_data);
        if (ret < 0)
        {
            printf("接收膝盖电机的电流模式数据帧， 失败！！！, 第%d腿，ret=%d\r\n", i + 1, ret);
            //return ret;
        }
    }

    return 0;
}

// int motorTestmain(int argc, char **argv)
// {
//     // struct sockaddr_can addr;
//     // struct ifreq ifr;
//     // const char *ifname = "can0";

//     int s;
//     int read_len;
//     char buf_receive[1024], buf_send[1024];
//     struct can_frame frame_w[3], frame_r;
//     int ret;
//     int i;
//     float time_start, time_end;

//     struct motor_data_t motor_data;
//     struct motor_command_t motor_setting;

//     //初始化好扭矩，一只狗腿循环一次
//     for (i = 0; i < 1; i++)
//     {
//         motor_setting.tau_abad[i] = 5;
//         motor_setting.tau_hip[i] = -5;
//         motor_setting.tau_knee[i] = 5;
//     }

//     //初始化CAN
//     ret = init_can_socket();
//     if(ret == 0) {
//         printf("CAN初始化成功, socket = %d\n", s);
//     }
//     else {
//         printf("CAN初始化失败, socket = %d\n", s);
//     }

//     while(1)
//     {
//         printf("收发任务开始执行！\n");

// 	    sysUsecTime(&time_start);
//         printf("time_start = %.6fs\n", time_start);

//         // for (i = 0; i < 3; i++)
//         // {
//         //     can_send(s, frame_w[i]);
//         // }
//         ret = motor_send_receive(&motor_setting, &motor_data);

//         sysUsecTime(&time_end);
//         printf("time_end = %.6fs\n", time_end);

//         printf("发送完成完成时间相差：%.6fs\n", ((time_end - time_start) / 1000000));

//         printf("\n");
        
//         usleep(1000);
//     }

//     return 0;
// }

//--------------------------------------------------------------------------------------------------------------

TestOneLeg::TestOneLeg() {
    std::cout << "init TestOneLeg" << std::endl;
    // the leg kinematics is relative to body frame, which is the center of the robot
    // leg order: 0-FL  1-FR  2-RL  3-RR
    leg_offset_x[0] = 0.1805;
    leg_offset_x[1] = 0.1805;
    leg_offset_x[2] = -0.1805;
    leg_offset_x[3] = -0.1805;
    leg_offset_y[0] = 0.047;
    leg_offset_y[1] = -0.047;
    leg_offset_y[2] = 0.047;
    leg_offset_y[3] = -0.047;
    motor_offset[0] = 0.0838;
    motor_offset[1] = -0.0838;
    motor_offset[2] = 0.0838;
    motor_offset[3] = -0.0838;
    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = 0.35;
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = 0.39;

    for (int i = 0; i < NUM_LEG; i++) {
        Eigen::VectorXd rho_fix(5);
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
        Eigen::VectorXd rho_opt(3);
        rho_opt << 0.0, 0.0, 0.0;
        rho_fix_list.push_back(rho_fix);
        rho_opt_list.push_back(rho_opt);
    }

    joint_pos.setZero();
    joint_vel.setZero();
    joint_torques.setZero();

}

int main(int, char**) {
    int s;
    int read_len;
    char buf_receive[1024], buf_send[1024];
    struct can_frame frame_w[3], frame_r;
    int ret;
    // int i;
    float time_start, time_end;

    struct motor_data_t motor_data;
    struct motor_command_t motor_setting;

    //初始化扭矩，一只狗腿循环一次
    for (int i = 0; i < 1; i++)
    {
        motor_setting.tau_abad[i] = 0;
        motor_setting.tau_hip[i] = 0;
        motor_setting.tau_knee[i] = 0;
    }


    // //初始化CAN
    // ret = init_can_socket();
    // if(ret == 0) {
    //     printf("CAN初始化成功, socket = %d\n", s);
    // }
    // else {
    //     printf("CAN初始化失败, socket = %d\n", s);
    // }

    // // Get motor data
    // ret = motor_send_receive(&motor_setting, &motor_data);

    // usleep(1000);
    // std::cout<< "2nd r and send"<< std::endl;
    // ret = motor_send_receive(&motor_setting, &motor_data);
    // usleep(1000);
    // std::cout<< "3rd r and send"<< std::endl;
    // ret = motor_send_receive(&motor_setting, &motor_data);

    double q0;
    double q1;
    double q2;
    // q0 = (double) motor_data.q_abad[0];
    // q1 = (double) motor_data.q_hip[0];
    // q2 = (double) motor_data.q_knee[0];
    // FL : 0.001, 0.679 (39deg), -1.625 (-93.1); foot pos: 0.277017, 0.1313, -0.50041; 
    // FL : 0, 0, 0; foot pos: 0.1805, 0.1308, -0.74
    // FR : -0.17, 0.656, -1.559 ; A1 foot pos: 0.27324, -0.217375, -0.497207

    q0 = 0.001;
    q1 = 0.679;
    q2 = -1.625;
    Eigen::Vector3d qMotor(q0,q1,q2);


    TestOneLeg oneleg;
    A1CtrlStates state;
    A1Kinematics a1_kin;
    // std::cout<< "a1_kin, rho_opt_size = "<<a1_kin.RHO_OPT_SIZE <<std::endl;
    // init values
    state.reset(); // 0, 1, 2, 3: FL, FR, RL, RR
    state.robot_mass = 5; //note 1
    state.a1_trunk_inertia << 0.0158533, 0.0, 0.0,
            0.0, 0.0377999, 0.0,
            0.0, 0.0, 0.0456542;  //note 2
    state.root_euler << 0.0, 0.0, 0.0;

    double dt = 0.01; // TODO: update dt = ros::Time::now - pre
    // BezierUtils bezierUtils[NUM_LEG];

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target;    
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel_last_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_last_time;
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;
    foot_pos_target.setZero();
    foot_vel_target.setZero();
    foot_pos_cur.setZero();
    foot_vel_cur.setZero();
    foot_pos_rel_last_time.setZero();
    foot_pos_target_last_time.setZero();
    foot_pos_error.setZero();
    foot_vel_error.setZero();
    foot_forces_kin.setZero();
    joint_torques.setZero();

    Eigen::Vector3d foot_pos_start(0.277017, 0.1313, -0.50041);  // TODO: to begein with, track one certain pose, instead of bezier curve.
    Eigen::Vector3d foot_pos_final(0.227017, 0.1313, -0.50041);

    BezierUtils bs_utils;
    int totalInterpolation = 50;
    Eigen::MatrixXd interp_pos_rst(3,totalInterpolation);
    int ii = 0;
    double sampT = (double) 1/totalInterpolation;
    for (double t=0.0; t<1; t += sampT) {
        interp_pos_rst.col(ii) = bs_utils.get_foot_pos_curve(t,foot_pos_start, 
        foot_pos_final,0);
        ii++;
    }
    // std::cout<<"position interpolation"<<std::endl;
    // std::cout<<interp_pos_rst.col(45)<<std::endl;

    state.foot_pos_rel.block<3, 1>(0, 0) = a1_kin.fk(
        qMotor,
        oneleg.rho_opt_list[0], oneleg.rho_fix_list[0]);  // foot pos in robot frame  ! undefined reference: add A1Kinematics.cpp in cmakelist. 
    foot_pos_cur.block<3, 1>(0, 0) = state.foot_pos_rel.col(0) ;
    std::cout<<"qMotor"<<qMotor<<std::endl;
    std::cout<< "!!!!cur foot pos= " << foot_pos_cur.block<3, 1>(0, 0) << std::endl;

    for (int t = 0; t < totalInterpolation; t++){
        // !!! TODO: READ q FROM MOTOR
        for (int i = 0; i < NUM_LEG; ++i) {
            if (i == 0) {  
                // DO forward kinematics to get foot pos
                // after getting q, calculate current foot pos and update jac     
                state.foot_pos_rel.block<3, 1>(0, i) = a1_kin.fk(
                        qMotor,
                        oneleg.rho_opt_list[i], oneleg.rho_fix_list[i]);  // foot pos in robot frame  ! undefined reference: add A1Kinematics.cpp in cmakelist. 
                foot_pos_cur.block<3, 1>(0, i) = state.foot_pos_rel.col(i) ;
                state.j_foot.block<3, 3>(3 * i, 3 * i) = a1_kin.jac(
                        qMotor,
                        oneleg.rho_opt_list[i], oneleg.rho_fix_list[i]);  // jacobian

                // foot_pos_cur.block<3, 1>(0, i) = state.root_rot_mat_z.transpose() * state.foot_pos_abs.block<3, 1>(0, i); // robot frame 
                std::cout<< "t = " << t << std::endl;
                // std::cout<< ", cur, " << foot_pos_cur.col(0) << std::endl;

                foot_pos_target.block<3, 1>(0, i) = interp_pos_rst.col(t);
                
                foot_vel_cur.block<3, 1>(0, i) = (foot_pos_cur.block<3, 1>(0, i) - foot_pos_rel_last_time.block<3, 1>(0, i)) / dt;
                foot_pos_rel_last_time.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);

                foot_vel_target.block<3, 1>(0, i) = (foot_pos_target.block<3, 1>(0, i) - foot_pos_target_last_time.block<3, 1>(0, i)) / dt;
                foot_pos_target_last_time.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i);

                foot_pos_error.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i) - foot_pos_cur.block<3, 1>(0, i);
                foot_vel_error.block<3, 1>(0, i) = foot_vel_target.block<3, 1>(0, i) - foot_vel_cur.block<3, 1>(0, i);
                foot_forces_kin.block<3, 1>(0, i) = foot_pos_error.block<3, 1>(0, i).cwiseProduct(state.kp_foot.block<3, 1>(0, i)) +
                                                    foot_vel_error.block<3, 1>(0, i).cwiseProduct(state.kd_foot.block<3, 1>(0, i)); // note 3, kp, kd

                state.foot_pos_cur = foot_pos_cur;
                state.foot_forces_kin = foot_forces_kin; // note 4
                std::cout << "pos err= "<< foot_pos_error.block<3, 1>(0, i)[0]<< "," << foot_pos_error.block<3, 1>(0, i)[1]<<", "<<foot_pos_error.block<3, 1>(0, i)[2] << std::endl;
                std::cout << "vel err= "<< foot_vel_error.block<3, 1>(0, i)[0]<< ", "<< foot_vel_error.block<3, 1>(0, i)[1] << ", "<< foot_vel_error.block<3, 1>(0, i)[2] << std::endl;

                // compute torques
                Eigen::Matrix3d jac = state.j_foot.block<3, 3>(3 * i, 3 * i);
                Eigen::Vector3d force_tgt = state.km_foot.cwiseProduct(state.foot_forces_kin.block<3, 1>(0, i));
                joint_torques.segment<3>(i * 3) = jac.lu().solve(force_tgt);   // jac * tau = F
            }
            joint_torques += state.torques_gravity;

            if (i==0){
                std::cout << "cmd tau1,2,3= " << joint_torques[0] <<", " << joint_torques[1] << ", " << joint_torques[2]<< std::endl;
            }
            

            // motor_setting.tau_abad[0] = joint_torques[0]/10;
            // motor_setting.tau_hip[0] = joint_torques[1]/10;
            // motor_setting.tau_knee[0] = joint_torques[2]/10;
            // ret = motor_send_receive(&motor_setting, &motor_data);

            // q0 = (double) motor_data.q_abad[0];
            // q1 = (double) motor_data.q_hip[0];
            // q2 = (double) motor_data.q_knee[0];
            // qMotor(0) = q0;
            // qMotor(1) = q1;
            // qMotor(2) = q2;
            // std::cout<< "received q= " << qMotor(0) << ", " << qMotor(1) << ", "<< qMotor(2)<< std::endl;

            // // prevent nan
            // for (int i = 0; i < 12; ++i) {
            //     if (!isnan(joint_torques[i]))
            //         state.joint_torques[i] = joint_torques[i];
            // }

            usleep(1000);

        }

    }

}

