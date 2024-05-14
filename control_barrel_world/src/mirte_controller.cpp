#include "mirte_controller.h"

// Constructor for MirteController class
MirteController::MirteController(const ros::NodeHandle &nh) : nh_(nh) {
    // Create publisher for commanding Mirte
    pubControl_ = nh_.advertise<geometry_msgs::Twist>("mirte/mobile_base_controller/cmd_vel", 20);
    
    // Create subscriber to obstacle detection node (PCL)
    subObstacles_ = nh_.subscribe("pcl_obstacle_detector_node/detections", 20, &MirteController::callbackObstacles, this);
    
    // Create subscriber to person detection node (OpenCV)
    subPersons_ = nh_.subscribe("opencv_person_detector_node/detections", 20, &MirteController::callbackPersons, this); 
    
    // Get params from ROS parameter server, set default values if they are not found
    ros::param::param("linear_velocity_forward", linearVelocityForward_, 0.8);
    ros::param::param("linear_velocity_steering", linearVelocitySteering_, 0.6);
    ros::param::param("angular_velocity_steering", angularVelocitySteering_, 0.4);

    // Check validity of parameters (no negative values etc.)
    if (linearVelocityForward_ <= 0) {
        linearVelocityForward_ = 0.8;
        ROS_WARN_STREAM("ROS parameter 'linear_velocity_forward' was set to an invalid value. Changed to default value " << linearVelocityForward_ << ".");
    }
    if (linearVelocitySteering_ <= 0) {
        linearVelocitySteering_ = 0.6;
        ROS_WARN_STREAM("ROS parameter 'linear_velocity_steering' was set to an invalid value. Changed to default value " << linearVelocitySteering_ << ".");
    }
    if (angularVelocitySteering_ <= 0) {
        angularVelocitySteering_ = 0.4;
        ROS_WARN_STREAM("ROS parameter 'angular_velocity_steering' was set to an invalid value. Changed to default value " << angularVelocitySteering_ << ".");
    }

    // Log parameter values
    ROS_DEBUG_STREAM("ROS parameters: " << linearVelocityForward_ 
    			            << ", " << linearVelocitySteering_
    			            << ", " << angularVelocitySteering_);
}

// This function contains the while loop that ensures Mirte receives control commands at a constant rate. 
void MirteController::runLoop() {
    // The loop executes at a frequency of 10 Hz until the node is killed 
    // or Mirte gets too close to a person
    ros::Rate rate(10);
    while(ros::ok() && !emergencyStop_)
    {
        // Execute pending callbacks first
        ros::spinOnce();
        
        // Determine what control command to send to Mirte based on the current detections
        control_ = getMirteCommand();
        
        // Publish this command to Mirte's command topic
        pubControl_.publish(control_);
        
        // Log Mirte command
        ROS_DEBUG_STREAM("Sending command to Mirte: linear.x=" << control_.linear.x 
                                            << ", angular.z=" << control_.angular.z);
        
        // Wait till next iteration
        rate.sleep();
    }
}

// This function contains the control algorithm for Mirte
geometry_msgs::Twist MirteController::getMirteCommand() {
    // Value to be returned, contains the command message that will be sent to Mirte
    geometry_msgs::Twist cmd_;

    // If no nearby objects detected, drive forward
    if (obstacleDetections_.empty()) {
	    cmd_.linear.x = linearVelocityForward_;
        cmd_.angular.z = 0.0;
        ROS_DEBUG_STREAM("Drive forward");
    } 

    // If one or more nearby objects detected, get the object closest to Mirte
    if (obstacleDetections_.size() >= 1) {
	    vision_msgs::Detection3D closestObstacle = getClosestObstacle(obstacleDetections_);
	    // If this object is to the right of Mirte (positive Y value)
	    // steer left, else steer right
	    if (closestObstacle.bbox.center.position.y > 0) {
	        cmd_.linear.x = linearVelocitySteering_;
	        cmd_.angular.z = -angularVelocitySteering_;
            ROS_DEBUG_STREAM("Steer left");
	    } else {
	        cmd_.linear.x = linearVelocitySteering_;
	        cmd_.angular.z = angularVelocitySteering_;
            ROS_DEBUG_STREAM("Steer right");
	    } 
    }
	
    // If one or more persons detected
    if (!personDetections_.empty()) {
        ROS_DEBUG_STREAM("At least one person detected");
    	for (int i = 0; i < personDetections_.size(); i++) {
            // Get the size and calculate the area of this person detection
            float size_x = personDetections_[i].bbox.size_x;
            float size_y = personDetections_[i].bbox.size_y;
            float area = size_x * size_y;
	        ROS_DEBUG_STREAM("(x,y)-dimensions of rectangle of detected person number " << i << ": (" 
                                << size_x << ", " << size_y << "), area: " << area);

            // If this person is too close, stop immediately!
	        if (area > 20000) {
	            cmd_.linear.x = 0;
		        cmd_.angular.z = 0;
                emergencyStop_ = true;
                ROS_WARN_STREAM("Detected a person who is too close to Mirte! Stopping now.");
	        }
	    }   
    }

    return cmd_;
}

// This function takes the vector of detected obstacles and returns the one closest to Mirte
// (so with the smallest radius)
vision_msgs::Detection3D MirteController::getClosestObstacle(std::vector<vision_msgs::Detection3D> detections) {
    // Value to be returned, will contain the closest obstacle to Mirte
    vision_msgs::Detection3D currentClosestDetection;
    
    // Declaring some variables needed in the for loop below
    float currentClosestRadius = 1.0;
    float xpos;
    float ypos;
    float zpos;
    float radius;
    
    // Iterate over all obstacles, checking their distance to Mirte and updating 
    // currentClosestDetection every time a closer obstacle is found
    for (int i = 0; i < detections.size(); i++) {
    	// Get the x, y and z positions of obstacle i
    	xpos = detections[i].bbox.center.position.x;
    	ypos = detections[i].bbox.center.position.y;
    	zpos = detections[i].bbox.center.position.z;

    	// Calculate the radius (distance to Mirte) of obstacle i
    	radius = std::sqrt(std::pow(xpos, 2) + std::pow(ypos, 2) + std::pow(zpos, 2));

        // If obstacle i is closer, update our currentClosestDetection
    	if (radius < currentClosestRadius) {
    	    currentClosestDetection = detections[i];
    	    currentClosestRadius = radius;
    	}
    }
    
    ROS_DEBUG_STREAM("Radius (distance to Mirte) of closest obstacle: " << currentClosestRadius);
    
    return currentClosestDetection;
}

// Callback function for the subscriber to the 3D obstacle detection topic
void MirteController::callbackObstacles(const vision_msgs::Detection3DArray& msg) {
    // Retrieve all obstacle detections and store them in a (temporary) vector
    std::vector<vision_msgs::Detection3D> detections = msg.detections;
    ROS_DEBUG_STREAM("Retrieved " << detections.size() << " obstacle detections.");
    
    // Filter the object detections to only keep the close ones
    obstacleDetections_.clear();
    for (int i = 0; i < detections.size(); i++) {
    	// Get the x, y and z positions of obstacle i
    	float xpos = msg.detections[i].bbox.center.position.x;
    	float ypos = msg.detections[i].bbox.center.position.y;
    	float zpos = msg.detections[i].bbox.center.position.z;
    	// Get the radius (distance to Mirte) of obstacle i
    	float radius = std::sqrt(std::pow(xpos, 2) + std::pow(ypos, 2) + std::pow(zpos, 2));
    	
    	ROS_DEBUG_STREAM("xyz of detection " << i << ": " << xpos << ", " 
                                          << ypos << ", " << zpos << ", radius: " << radius);
    	
    	// If obstacle i is in front of Mirte and within a radius of 0.7 meters, keep it
    	if (xpos > 0 && radius < 0.7) {
    	    obstacleDetections_.push_back(msg.detections[i]);
            ROS_DEBUG_STREAM("Keeping detection number " << i << ".");
    	}
    	
    }
    
    // Log filtered detections
    ROS_DEBUG_STREAM(obstacleDetections_.size() << " out of " 
                           << detections.size() << " obstacle detections are close to Mirte.");
}

// Callback function for the subscriber to the 2D person detection topic
void MirteController::callbackPersons(const vision_msgs::Detection2DArray& msg) {
    // Retrieve all person detections and store them in a vector
    personDetections_ = msg.detections;
    ROS_DEBUG_STREAM("Retrieved " << personDetections_.size() << " person detections.");
}
