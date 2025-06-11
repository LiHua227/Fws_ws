# include <Fws_model.h>


void cmd_vel_CB(const geometry_msgs::Twist::ConstPtr &msg)
{ 
    LXN_ALG::FWS_MODEL car_mode(1.29, 0.82);
    double pos[4], vel[4];
    car_mode.Calculate(vel, pos, msg, LXN_ALG::Carmode::undetermined);

    std::cout<<"pos:"<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<" "<<pos[3]<<"\n";
    std::cout<<"vel:"<<vel[0]<<" "<<vel[1]<<" "<<vel[2]<<" "<<vel[3]<<"\n";
}

int main(int argc, char  *argv[])
{
    ros::init(argc, argv, "Test_4WS_mode");
    
    ros::NodeHandle nh;
    ros::Subscriber vel_sub_ = nh.subscribe("/cmd_vel", 1, cmd_vel_CB);
    
    ros::spin();

    return 0;
}
 