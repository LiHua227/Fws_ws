# include <WalkHandle.h>

int main(int argc, char  *argv[])
{
    ros::init(argc, argv, "Test_walk");
    

    ros::NodeHandle nh;
 
    std::cout<< "Begin!"<<std::endl;
    LXN_ALG::CanHandle_ptr CanHandle_;             // CAN卡控制句柄
    LXN_ALG:: Walk_Handle_ptr Walk_Handle_;         // 行走电机控制句柄
   

    CanHandle_   = std::make_shared<LXN_ALG::CanHandle>(500); 
    Walk_Handle_  = std::make_shared<LXN_ALG::Walk_Handle>(CanHandle_); 
    double vel[4]={0.5, 0.3, 0.5, 0.3};
    for(int i = 0;i < 20 ; i++)
    {
        Walk_Handle_->SendVel(vel);
        ros::Duration(0.1).sleep();
    }
  
    std::cout<< "Send successfully!"<<std::endl;
    
    double vel1[4]={ 0.0 , 0.0 , 0.0, 0.0 };

    Walk_Handle_->SendVel(vel1);

     std::cout<< "close!"<<std::endl;
    return 0;
    
}
 