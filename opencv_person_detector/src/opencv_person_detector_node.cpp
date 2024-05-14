#include "person_detector.h"

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "opencv_person_detector_node");
    ros::NodeHandle nh;

    // Create an instance of PersonDetector and run its loop
    PersonDetector detector(nh);
    detector.runLoop();
    
    return 0;
}
