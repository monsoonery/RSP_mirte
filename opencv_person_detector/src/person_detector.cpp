#include "person_detector.h"

// Constructor for PersonDetector class
PersonDetector::PersonDetector(const ros::NodeHandle &nh) : nh_(nh) {
    // Create subscriber to Mirte's image topic
    subImg_ = nh_.subscribe("mirte/image_raw", 20, &PersonDetector::callbackImage, this);
    
    // Create publisher for person detections
    pubDetections_ = nh_.advertise<vision_msgs::Detection2DArray>("opencv_person_detector_node/detections", 20);
    
    // Create publisher for images with green squares
    pubVisuals_ = nh_.advertise<sensor_msgs::Image>("opencv_person_detector_node/visual", 20);
}

// This function contains the while loop that ensures the person detection is executed at a constant rate
void PersonDetector::runLoop() {
    // The loop executes at a frequency of 10 Hz until the node is killed 
    ros::Rate rate(10);
    while(ros::ok())
    {
    	// Execute pending callbacks first
    	ros::spinOnce();
    	        
        // Wait till next iteration
        rate.sleep();
    }
}

// This function converts the vector of detected persons (type cv::Rect) to one Detections2DArray
vision_msgs::Detection2DArray PersonDetector::convertTo2DArray(std::vector<cv::Rect> persons) {
    // Full array of detections to be returned
    vision_msgs::Detection2DArray allDetections; 
    
    // A single detection, multiple of these will be appended to the full array above
    vision_msgs::Detection2D singleDetection; 
    
    // Set values for each detected person
    for (int i = 0; i < persons.size(); i++) {
    	singleDetection.bbox.center.x = persons[i].x + persons[i].width / 2;
    	singleDetection.bbox.center.y = persons[i].y + persons[i].height / 2;
    	singleDetection.bbox.size_x = persons[i].width;
    	singleDetection.bbox.size_y = persons[i].height;

        // Set the header timestamp of this detection to the current time
        singleDetection.header.stamp = ros::Time::now();
    	
    	// Append this detection to the Detections2DArray
    	allDetections.detections.push_back(singleDetection);
    }
    
    // Return the full array
    return allDetections;
}

// Callback function for the subscriber to Mirte's image topic
void PersonDetector::callbackImage(const sensor_msgs::ImageConstPtr& msg) {
    // Convert ROS image format to OpenCV image format
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);    
    
    // Use the default people detector
    hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    
    // Find all persons in the image and store them in the vector foundPersons_
    hog_.detectMultiScale(cv_ptr->image, foundPersons_, hitThreshold_, winStride_);
    ROS_DEBUG_STREAM("Number of detected persons: " << foundPersons_.size());
    
    // Draw a green rectangle around each detected person on the image
    for (int i = 0; i < foundPersons_.size(); i++) {
    	cv::Point topLeft = foundPersons_[i].tl();
    	cv::Point bottomRight = foundPersons_[i].br();
    	cv::rectangle(cv_ptr->image, topLeft, bottomRight, CV_RGB(0,255,0), 2);
    }
    
    // Convert the vector of Rects to a Detections2DArray (for publishing)
    vision_msgs::Detection2DArray detections_ = convertTo2DArray(foundPersons_);
    
    // Set the header timestamp of the message to the current time
    detections_.header.stamp = ros::Time::now();

    // Publish the Detections2DArray message to the detections topic
    pubDetections_.publish(detections_);
    
    // Publish the images with green squares to the visual topic (displayed in Rviz)
    pubVisuals_.publish(cv_ptr->toImageMsg());
}
