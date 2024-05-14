#ifndef PERSON_DETECTOR_H
#define PERSON_DETECTOR_H

#include <ros/ros.h>
#include <vision_msgs/Detection2DArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <vector>

// This class contains all variables and functions needed to detect persons with OpenCV
class PersonDetector {
    private:
        ros::NodeHandle nh_;

	    // Subscribers and publishers
        ros::Subscriber subImg_;
        ros::Publisher pubDetections_;
        ros::Publisher pubVisuals_;
        
        // Subscriber callback function
        void callbackImage(const sensor_msgs::ImageConstPtr& msg);
        
        // Detection2DArray message that will contain the detected persons
        vision_msgs::Detection2DArray detections_;

        // HOG detector and its parameters
        cv::HOGDescriptor hog_;
        std::vector<cv::Rect> foundPersons_;
        cv::Size winStride_ = cv::Size(8,8);
    	int hitThreshold_ = 0;
        
        // Function that converts a vector of cv::Rects to a Detection2DArray
        vision_msgs::Detection2DArray convertTo2DArray(std::vector<cv::Rect> persons);
        
    public:
    	// Class constructor
    	PersonDetector(const ros::NodeHandle &nh);
    	
    	// This function contains the while loop and is called in the node
    	void runLoop();
    		
};

#endif
