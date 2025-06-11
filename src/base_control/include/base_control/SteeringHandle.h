/**
 * @file SteeringHandle.h
 * @author LeoXiuNeng (LiWeiran227@163.com)
 * @brief control four steering Motor
 * @version 0.1
 * @date 2025-06-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

# ifndef _STEERINGHANDLE_H_
# define _STEERINGHANDLE_H_

# include <cmath>
# include <CanHandle.h>
# include <ros/ros.h>

namespace LXN_ALG
{
/**
 * @brief 封装转向电机的控制协议，接收位置，调用CAN发送约定协议
 */
class Steering_Handle
{
public:
    /**
     * @brief 舵机无参构造
     */
    Steering_Handle(){}

    /**
     * @brief 舵机有参构函数
     * @param CanHandle_ptr CAN句柄
     */
    Steering_Handle(CanHandle_ptr CanHandle);

    /**
     * @brief 析构函数
     */
    ~Steering_Handle(){}

    /**
     * @brief 发送4个电机角度
     * @return 是否发送成功
     */
    bool SendPose(const double *pos);

    /**
     * @brief 读取当前电机角度
     * @return 是否读取成功
     */
    bool GetPose(double *pos);

    /**
     * @brief 急停
     */
    void EmergencyStop(){}

    /**
     * @brief 设置运行速度
     * @param vel 目标速度
     */
    void SetVel(double vel);

    /**
     * @brief 设置运行加速度
     * @param acc 目标加速度
     */
    void SetAcc(double acc);

    /**
     * @brief 设置运行减速度
     * @param dec 目标减速度
     */
    void SetDec(double dec);

private:
    /**
     * @brief 角度转换成脉冲
     * @param pos 目标角度
     * @param pulse 目标脉冲
     */
    void _Pos2Pulse(const double* pos, int* pulse);

    /**
     * @brief 脉冲转换成角度
     * @param pulse
     * @param pose  角度
     */
    void _Pulse2Pos(const int* pulse, double* pose);

    /**
     * @brief 发送一个电机的目标位置
     * @param pulse 目标脉冲
     * @return 是否发送成功
     */
    bool _SendPulse(const int* pulse);

    /**
     * @brief 将int类型的数按照小端排列，并赋值给数组
     * @param num 传入的int数
     * @param array 写入结果的数组
     */
    void _Int2LEBytes(const int num, unsigned char* array);

    /**
     * @brief 将数组中的小端排序转成int
     * @param array 小段数组
     * @return 结果
     */
    int _LEBttes2Int(const unsigned char* array);

    /**
     * @brief 使能电机
     */
    void _Enable();

private:
    CanHandle_ptr CanHandle_; // CAN句柄

    bool enable_;          // 使能标志位
    int reduce_ratio_;     // 减速比
    int Pulse_per_circle_; // 电机转一圈的脉冲数

    std::array<int, 4> Zero_pulse_; // 零位脉冲
    std::array<int, 4> S_Can_Id_;   // 各电机发送时CAN_ID
    std::array<int, 4> R_Can_Id_;   // 各电机接收时的CAN_ID

    // 舵机控制协议
    unsigned char motion_mode_[8]        = {0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00};    // 修改运动模式
    unsigned char target_position_[8]    = {0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};    // 写目标位置
    unsigned char work_speed_[8]         = {0x23, 0x81, 0x60, 0x00, 0x58, 0x1B, 0x00, 0x00};    // 写工作速度 
    unsigned char acceleration_speed_[8] = {0x23, 0x83, 0x60, 0x00, 0x58, 0x1B, 0x00, 0x00};    // 写加速度 
    unsigned char deceleration_speed_[8] = {0x23, 0x84, 0x60, 0x00, 0x58, 0x1B, 0x00, 0x00};    // 写减速度
    unsigned char motor_ready_[8]        = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};    // 电机准备
    unsigned char wait_for_enable_[8]    = {0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00};    // 电机等待使能
    unsigned char motor_enable_[8]       = {0x2B, 0x40, 0x60, 0x00, 0x2F, 0x00, 0x00, 0x00};    // 电机使能
    unsigned char motor_motion_[8]       = {0x2B, 0x40, 0x60, 0x00, 0x3F, 0x00, 0x00, 0x00};    // 电机运动
    unsigned char read_position_[8]      = {0x40, 0x64, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};    // 读电机位置

}; // class  steering_Handle

typedef  std::shared_ptr<Steering_Handle>  Steering_Handle_ptr;

} // namespace LXN_ALG


# endif 