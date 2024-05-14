#include "mirte_controller.h"

int main(int argc, char **argv) 
{
    // Initialize the ROS node
    ros::init(argc, argv, "control_barrel_world_node");
    ros::NodeHandle nh;
    
    // Create an instance of MirteController and run its loop
    MirteController controller(nh);
    controller.runLoop();
    
    return 0;
}
