#ifndef ros_img_processor_node_H
#define ros_img_processor_node_H

//std C++
#include <iostream>

//ROS headers for image I/O
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

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
        ros::Subscriber camera_info_subs;

        //publishers
        image_transport::Publisher image_pub_;      
        
        //pointer to received (in) and published (out) images
        cv_bridge::CvImagePtr cv_img_ptr_in_;            
        cv_bridge::CvImage cv_img_ptr_out_;
        
        //wished process rate, [hz]
        double rate_;
        
    protected: 
        // callbacks
        void image_callback(const sensor_msgs::ImageConstPtr& _msg);
        void cameraInfo_callback(const sensor_msgs::CameraInfo & _msg);

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
};
#endif