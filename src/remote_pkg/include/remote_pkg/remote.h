# ifndef _REMOTE_H_
# define _REMOTE_H_


#include <serial/serial.h>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

using std::string;

namespace LXN_ALG
{
class Remote_Handle
{
public:
    /**
     * @brief 无参构造函数
     */
    Remote_Handle(){}

    /**
     * @brief 有参构造函数
     * @param baud_rate 波特率
     * @param USB_Name 串口名称
     */
    Remote_Handle(int baud_rate, string USB_Name);

    /**
     * @brief 析构函数
     */
    ~Remote_Handle();

    /**
     * @brief 循环读取数据
     */
    void run();

private:

    /**
     * @brief 初始化串口
     * @param baud_rate 波特率
     * @param USB_Name 串口名称
     */
    bool _Initlizate(int baud_rate, string USB_Name);

    /**
     * @brief 接收串口数据
     */
    bool _Receive_Data();

    /**
     * @brief 解析数据并发布
     */
    void _Handle_Data();


private:
    serial::Serial USB_Serial_;
    uint8_t Receive_Data_[24];

    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;

    double max_vel_x_; // 最大速度
    double max_vel_y_;
    double max_vel_z_;

}; // class remote_handle

} // end LXN_ALG namespace


# endif
