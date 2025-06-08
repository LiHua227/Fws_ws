# include <Base_control.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_control_node");
    LXN_ALG::Base_control Base_control();
    return 0;
}
