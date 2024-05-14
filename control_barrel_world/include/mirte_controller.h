#ifndef MIRTE_CONTROLLER_H
#define MIRTE_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/BoundingBox3D.h>
#include <vision_msgs/BoundingBox2D.h>
#include <stdlib.h>
#include <vector>

// This class contains all variables and functions needed to control Mirte 
// on the basis of 3D obstacle detections and 2D pedestrian detections
class MirteController {
    private:
        ros::NodeHandle nh_;

	    // Subscribers and publishers
        ros::Subscriber subPersons_;
        ros::Subscriber subObstacles_;
        ros::Publisher pubControl_;
        
        // Subscriber callback functions
        void callbackObstacles(const vision_msgs::Detection3DArray& msg);
        void callbackPersons(const vision_msgs::Detection2DArray& msg);
        
        // Twist message that will contain Mirte commands
        geometry_msgs::Twist control_;
        
        // This flag becomes true once Mirte gets too close to a person
        // and breaks the control loop to bring her to a permanent stop
        bool emergencyStop_ = false;

        // ROS parameters for controlling Mirte
        double linearVelocityForward_;
        double linearVelocitySteering_;
        double angularVelocitySteering_;
        
        // Vector that contains all detected persons (OpenCV)
        std::vector<vision_msgs::Detection2D> personDetections_;
        
        // Vector that contains all detected obstacles (PCL)
        std::vector<vision_msgs::Detection3D> obstacleDetections_;
        
        // Function that contains main control algorithm for commanding Mirte
        geometry_msgs::Twist getMirteCommand();
        
        // Function to get the closest obstacle to Mirte
        vision_msgs::Detection3D getClosestObstacle(std::vector<vision_msgs::Detection3D> detections);


    public:
    	// Class constructor
    	MirteController(const ros::NodeHandle &nh);
    	
    	// This function contains the while loop and is called in the node
    	void runLoop();
};

#endif