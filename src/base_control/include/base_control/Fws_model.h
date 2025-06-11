/**
 * @file Fws_model.h
 * @author LeoXiuNeng (LiWeiran227@163.com)
 * @brief  car model, include 4WS and double ACKeman
 * @version 0.1
 * @date 2025-06-11
 * 
 * @copyright Copyright (c) 2025
 * 
 */

# ifndef _FWS_MODEL_H_
# define _FWS_MODEL_H_

# include <ros/ros.h>
# include <geometry_msgs/Twist.h>
# include <cmath>

using std::hypot;

namespace LXN_ALG
{
/**
 * @brief 汽车运动学模型
 */
enum class Carmode : uint8_t 
{
    undetermined = 0, // 未定义，自动选择
    ackm,             // 双阿克曼模型
    Fws               // 四轮四转模型
};

/**
 * @brief 运动学模型类，支持双阿克曼和四轮四转
 */
class FWS_MODEL
{
public:
    /**
     * @brief 无参构造函数
     */
    FWS_MODEL() {}

    /**
     * @brief 有参构造
     * @param car_legth 车辆前后轴的宽度
     * @param car_width 车辆左右轴的宽度
     */
    FWS_MODEL(double car_legth, double car_width);

    /**
     * @brief 无参析构函数
     */
    ~FWS_MODEL() {}

    /**
     * @brief 对外的唯一接口
     * @param current_vel 当前速度
     * @param current_pos 当前角度位置
     * @param target_vel 计算后的速度
     * @param target_pos 计算后的角度位置
     * @param msg 传入的整车速度
     * @param carmode 运动学模型选择，默认是不指定
     */
    bool Calculate(double* current_vel, double* current_pos,
                   double* target_vel, double* target_pos,
                   const geometry_msgs::Twist::ConstPtr &msg,
                   Carmode carmode = Carmode::undetermined);

    /**
     * @brief 对外的唯一接口
     * @param target_vel 计算后的速度
     * @param target_pos 计算后的角度位置
     * @param msg 传入的整车速度
     * @param carmode 运动学模型选择，默认是不指定
     */
    bool Calculate(double *target_vel, double *target_pos,
                   const geometry_msgs::Twist::ConstPtr &msg,
                   Carmode carmode);

private:
    /**
     * @brief 计算双阿克曼模型
     * @param vel 计算后的速度
     * @param pos 计算后的角度位置
     * @param msg 传入的整车速度
     */
    bool _get_ack_vel_pos(double *vel, double *pos,
                          const geometry_msgs::Twist::ConstPtr &msg);

    /**
     * @brief 计算四轮四转速度
     * @param vel 计算后的速度
     * @param pos 计算后的角度位置
     * @param msg 传入的整车速度
     */
    bool _get_4WS_vel_pos(double *vel, double *pos,
                          const geometry_msgs::Twist::ConstPtr &msg);

    /**
     * @brief 模型自动选择
     * @param carmode 返回值，计算后的车辆模型
     * @param msg 传入的整车速度
     */
    void _autochoose_carmode(Carmode &carmode, const geometry_msgs::Twist::ConstPtr &msg);

    /**
     * @brief 限制线速度，当转角偏差过大时
     * @param msg         整车速度
     * @param current_pos 当前角度位置
     * @param target_pos  目标角度位置
     * @param target_vel  目标线速度
     */
    void _limt_linear(const geometry_msgs::Twist::ConstPtr &msg, double* current_pos, 
                        double* target_pos, double* target_vel);

private:
    // Ackman
    double car_legth_; // 前后轮距
    double car_width_; // 左右轮距

    // 4WS
    double Car_R_, Car_Q_, Car_B_;

}; // class FWS_MODEL

typedef  std::shared_ptr<FWS_MODEL>  FWS_MODEL_ptr;

} // namespace LXN_ALG

# endif