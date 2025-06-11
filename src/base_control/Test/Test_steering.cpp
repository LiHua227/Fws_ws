# include <SteeringHandle.h>

int main(int argc, char  *argv[])
{
    ros::init(argc, argv, "Test_steering");
    

    ros::NodeHandle nh;
 
    std::cout<< "Begin!"<<std::endl;
    LXN_ALG::CanHandle_ptr CanHandle_;             // CAN卡控制句柄
    LXN_ALG::Steering_Handle_ptr Steering_Handle_;         // 旋转电机控制句柄
   

    CanHandle_   = std::make_shared<LXN_ALG::CanHandle>(500); 
    Steering_Handle_  = std::make_shared<LXN_ALG::Steering_Handle>(CanHandle_); 

    double vel[4]={-0.5, -0.3, -0.5, -0.3};
     
        Steering_Handle_->SendPose(vel);
        std::cout<< "Start steering!"<<std::endl;
        ros::Duration(1).sleep();
     

        double vel1[4]={0.5, 0.3, 0.5, 0.3};
     
        Steering_Handle_->SendPose(vel1);
        std::cout<< "Start steering 2!"<<std::endl;
        ros::Duration(5).sleep();




    // Steering_Handle_->GetPose(vel);
    // ros::Duration(0.1).sleep();

    // for(int i = 0 ;i<4 ;i++)
    // {
    //     std::cout<< " vel  "<< i << " =" <<vel[i] <<std::endl;
    // }

    std::cout<< "Send successfully!"<<std::endl;
 
     std::cout<< "close!"<<std::endl;
    return 0;
    
}
 