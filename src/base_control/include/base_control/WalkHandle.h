# ifndef _WALKHANDLE_H_
# define _WALKHANDLE_H_

# include <cmath>
# include <ros/ros.h>
# include <CanHandle.h>

namespace LXN_ALG
{
class Walk_Handle
{
public:
    /**
     * @brief 舵机无参构造
     */
    Walk_Handle(){}

    /**
     * @brief 舵机含参构造
     */
    Walk_Handle(CanHandle_ptr CanHandle);

    /**
     * @brief 析构函数
     */
    ~Walk_Handle(){}

    /**
     * @brief 发送4个电机行走速度 单位：m/s
     * @return 是否发送成功
     */
    bool SendVel(const double *vel);

    /**
     * @brief 读取当前电机运行速度 单位：m/s
     * @return 是否读取成功
     */
    bool GetVel(double *vel);

    /**
     * @brief 急停
     */
    void EmergencyStop();

    /**
     * @brief 设置运行加速度  m/s^2
     * @param acc 目标加速度
     */
    void SetAcc(double acc);

private:
    /**
     * @brief 速度转换成转速 单位：RPM
     * @param vel 速度
     * @param rpm 转速
     */
    void _Vel2RPM(const double* vel, int* rpm);

    /**
     * @brief RPM转换成速度 rpm单位为0.1 
     * @param rpm 转速  单位：0.1RPM
     * @param vel 速度
     */
    void _Rpm2Vel(const int* rpm, double* vel);

    /**
     * @brief 将RPM写入速度指令
     * @param rpm_1 
     * @param rpm_2
     */
    void _RPM2work_speed(int rpm_1, int rpm_2);

    /**
     * @brief 从两段数据中读取转速
     * @param array_1 数据1
     * @param array_2 数据2
     * @param rpm 解析后的转速
     */
    void _work_speed2RPM(const unsigned char* array_1,
                        const unsigned char* array_2,
                        int* rpm );

    /**
     * @brief 发送一个电机的目标脉冲速度
     * @param pulse 目标脉冲
     * @return 是否发送成功
     */
    bool _SendRpm(const int* rpm);

     /**
     * @brief 使能
     */
    void _Enable();


private:
    CanHandle_ptr CanHandle_; // CAN句柄

    std::array<int, 4> S_Can_Id_; // 用于发送的CAN_ID
    std::array<int, 4> R_Can_Id_; // 用于接收的CAN_ID

    double Wheel_radius_; // 轮子半径

    bool enable_; // 使能标志位

    // 轮毂电机控制协议
    unsigned char motion_mode_[8]     = {0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00}; // 写电机运动模式， 默认为速度模式
    unsigned char motor_ready_[8]     = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00}; // 写电机准备
    unsigned char wait_for_enable_[8] = {0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00}; // 写电机准备使能
    unsigned char motor_enable_[8]    = {0x2B, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00}; // 写电机使能
    unsigned char work_speed_[8]      = {0x23, 0xff, 0x60, 0x03, 0x00, 0x00, 0x00, 0x00}; // 写电机运行速度
    unsigned char read_vel_[8]        = {0x40, 0x6c, 0x60, 0x03, 0x00, 0x00, 0x00, 0x00}; // 读电机运行速度
    unsigned char read_Electric_[8]   = {0x40, 0x77, 0x60, 0x03, 0x00, 0x00, 0x00, 0x00}; // 读电机运行电流
    unsigned char stop_enbable[8]     = {0x2B, 0x40, 0x60, 0x00, 0x02, 0x00, 0x00, 0x00}; // 写电机停止并保持使能状态
    unsigned char stop_disabable[8]   = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00}; // 写电机停止并解除使能状态

}; // class  Walk_Handle

typedef  std::shared_ptr<Walk_Handle>  Walk_Handle_ptr;

} // namespace LXN_ALG


# endif 