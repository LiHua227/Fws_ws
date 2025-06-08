# ifndef _CANHANDLE_H_
# define _CANHANDLE_H_

# include <iostream>
# include <memory>
# include <array>

# include <controlcan.h>

# include <ros/ros.h>

namespace LXN_ALG
{

class CanHandle
{
public:

    /**
     * @brief 无参构造函数
     */
    CanHandle(){}

    /**
     * @brief 有参构造函数
     * @param baud_rate 波特率 单位：Kbps
     */
    CanHandle(int baud_rate);

    /**
     * @brief 析构函数
     */
    ~CanHandle();

    /**
     * @brief 发送数据
     * @param can_id 目标设备ID
     * @param data 发送的数据内容
     * @return 是否发送成功
     */
    bool SendData(int can_id, const unsigned char* data);

    /**
     * @brief 接收数据
     * @param can_id 接收的目标ID
     * @param data 收到的数据内容
     * @return 是否成功
     */
    bool ReceiveData(int can_id, unsigned char* data);

    /**
     * @brief 清理缓存 在发送过程中从机会返回一些数据作为响应，
     * 但是这些数据并不重要，没必要读取，为了防止排除后续读取过程中这些数据的干扰，
     * 需要清除缓存，在正式读取之前
     */
    void ClearBuffer();

private:

    /**
     * @brief 初始化CAN
     * @param baud_rate 波特率 单位：Kbps
     * @return 是否初始化成功
     */
    bool _Initlizate(int baud_rate);
    
}; // class CanHandle

typedef  std::shared_ptr<CanHandle>  CanHandle_ptr;
} // namespace LXN_ALG


# endif
