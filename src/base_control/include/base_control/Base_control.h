# ifndef _BASE_CONTROL_H_
# define _BASE_CONTROL_H_

# include <SteeringHandle.h>
# include <WalkHandle.h>
# include <Fws_model.h>

# include <ros/ros.h>

namespace LXN_ALG
{
class Base_control
{
public:
    /**
     * @brief 无参构造
     */
    Base_control();

    /**
     * @brief 无参析构
     */
    ~Base_control();

private:
    /**
     * @brief 速度回调函数
     */
    void cmd_vel_CB(const geometry_msgs::Twist::ConstPtr &msg);

private:
    // Handle
    CanHandle_ptr CanHandle_;             // CAN卡控制句柄
    Steering_Handle_ptr Steering_Handle_; // 转向电机控制句柄
    Walk_Handle_ptr Walk_Handle_;         // 行走电机控制句柄
    FWS_MODEL_ptr FWS_Model_;             // 运动模型句柄

    // ROS
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber cmd_vel_sub_;

}; // class LXN_ALG


} // namespace LXN_ALG



# endif