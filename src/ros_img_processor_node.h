#ifndef ros_img_processor_node_H
#define ros_img_processor_node_H

//std C++
#include <iostream>

//ROS headers for image I/O
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"

/** \brief Simple Image Processor
 *
 * Simple Image Processor with opencv calls
 *
 */
class RosImgProcessorNode
{
    protected:
        //ros node handle
        ros::NodeHandle nh_;

        //image transport
        image_transport::ImageTransport img_tp_;

        // subscribers to the image and camera info topics
        image_transport::Subscriber image_subs_;
        ros::Subscriber camera_info_subs_;

        //publishers
        image_transport::Publisher image_pub_;

        //pointer to received (in) and published (out) images
        cv_bridge::CvImagePtr cv_img_ptr_in_;
        cv_bridge::CvImage cv_img_out_;

	     	//Camera matrix
		    cv::Mat matrixP_;
        ros::Publisher dir_ball  = nh_.advertise<geometry_msgs::PoseStamped>("position", 1);
        //shows position, orientation and a header
        //image encoding label
        std::string img_encoding_;

        //wished process rate, [hz]
        double rate_;
        // intrinsic calibration matrix K
        cv::Mat matrixK_;
        //inverse K matrixP
        cv::Mat invK_;
        //
        cv::Mat direction = (cv::Mat_<double>(3,1) << 0, 0, 0);
    protected:
        // callbacks
        void imageCallback(const sensor_msgs::ImageConstPtr& _msg);
        void cameraInfoCallback(const sensor_msgs::CameraInfo & _msg);

    public:
        /** \brief Constructor
        *
        * Constructor
        *
        */
        RosImgProcessorNode();

        /** \brief Destructor
        *
        * Destructor
        *
        */
        ~RosImgProcessorNode();

        /** \brief Process input image
        *
        * Process input image
        *
        **/
        void process();

        /** \brief Publish output image
        *
        * Publish output image
        *
        */
        void publish();

        /** \brief Returns rate_
         *
         * Returns rate_
         *
         **/
        double getRate() const;
};
#endif
