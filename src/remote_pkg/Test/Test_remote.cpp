# include<remote.h>

int main(int argc, char  *argv[])
{
    ros::init(argc, argv, "Test_remote_node");
    LXN_ALG::Remote_Handle remote_device(115200, "/dev/ttyUSB0");
    
    return 0;
}
