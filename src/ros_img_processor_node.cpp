#include "ros_img_processor_node.h"

RosImgProcessorNode::RosImgProcessorNode() :
    nh_(ros::this_node::getName()),
    img_tp_(nh_)
{
	//loop rate [hz], Could be set from a yaml file
	rate_=10;

	//sets publishers
	image_pub_ = img_tp_.advertise("image_out", 100);

	//sets subscribers
	image_subs_ = img_tp_.subscribe("image_in", 1, &RosImgProcessorNode::imageCallback, this);
	camera_info_subs_ = nh_.subscribe("camera_info_in", 100, &RosImgProcessorNode::cameraInfoCallback, this);
}

RosImgProcessorNode::~RosImgProcessorNode()
{
    //
}
//constants circle detection
const int GAUSSIAN_BLUR_SIZE = 7;
const double GAUSSIAN_BLUR_SIGMA = 2;
const double CANNY_EDGE_TH = 150;
const double HOUGH_ACCUM_RESOLUTION = 2;
const double MIN_CIRCLE_DIST = 40;
const double HOUGH_ACCUM_TH = 70;
const int MIN_RADIUS = 20;
const int MAX_RADIUS = 80;
void RosImgProcessorNode::process()
{

    geometry_msgs::PoseStamped message;
    //message that shows data
    cv::Rect_<int> box;
//find the ball variables Hough transform
    cv::Mat gray_image;
    std::vector<cv::Vec3f> circles;
    cv::Point center;
    cv::Mat previous_center = (cv::Mat_<double>(3,1) << 0, 0, 0);
    cv::Mat centralPoint;
    /*
    cv::Point first = cv::Point (0,0);
    cv::Point second = cv::Point (0,0);
    int drawrow = 0;
    */
    int radius;

    //check if new image is there
    if ( cv_img_ptr_in_ != nullptr )
    {
        //copy the input image to the out one
        cv_img_out_.image = cv_img_ptr_in_->image;
        //previous_center = cv::Point(cv_img_ptr_in_->image.cols/2,cv_img_ptr_in_->image.rows/2 );
		      //find the ball
		        //TODO

        //clear previous circles
		    circles.clear();
        // If input image is RGB, convert it to gray
		    cv::cvtColor(cv_img_out_.image, gray_image, CV_BGR2GRAY);
        //Reduce the noise so we avoid false circle detection filter
        cv::GaussianBlur( gray_image, gray_image, cv::Size(GAUSSIAN_BLUR_SIZE, GAUSSIAN_BLUR_SIZE), GAUSSIAN_BLUR_SIGMA );
        //Apply the Hough Transform to find the circles
	      // third parameter is used to select the Hough method HOUGH_STANDARD,HOUGH_PROBABILISTIC, 		HOUGH_MULTI_SCALE & HOUGH_GRADIENT
        cv::HoughCircles( gray_image, circles, CV_HOUGH_GRADIENT, HOUGH_ACCUM_RESOLUTION, MIN_CIRCLE_DIST, CANNY_EDGE_TH, HOUGH_ACCUM_TH, MIN_RADIUS, MAX_RADIUS );
        for(unsigned int ii = 0; ii < circles.size(); ii++ )
        {
            if ( circles[ii][0] != -1 )
            {
                    //drawrow = 1;
                    center = cv::Point(cvRound(circles[ii][0]), cvRound(circles[ii][1]));
                    radius = cvRound(circles[ii][2]);
                    //circle (img circle drawn, center of the circle, radius, color, thickness -1 for all painted, type of line, nº of fractional bits)
                    //cv::circle(cv_img_out_.image, center, 5, cv::Scalar(0,0,255), 1, 8, 0 );
                    // circle center in green
                    cv::circle(cv_img_out_.image, center, radius, cv::Scalar(0,255,0), 4, 16, 0 );
                    // thickness = 4 & type of line LINE_AA = 16 antialiased line
                    cv::line( cv_img_out_.image, center, cv::Point(direction.at<double>(0, 0), direction.at<double>(1, 0) ), cv::Scalar( 110, 220, 0 ),  2, 8 );
                    // circle center point x and y coordinates.

            }
            /*
            else
            {
              drawrow = 0;
            }

            second = cv::Point (int(cvRound(centralPoint(1,0))), int(cvRound(centralPoint(2,0))));
            */
            centralPoint = (cv::Mat_<double>(3,1) << cvRound(circles[ii][0]), cvRound(circles[ii][1]), 0);
            direction = invK_*centralPoint; //direction d=invK*u
            //drawing line direction of the circle

        }
        /*
        if (drawrow !=0)
        {
          cv::arrowedLine(cv_img_out_.image, previous_center, centralPoint, cv::Scalar(0,0,255), 4, 0 ,0.1);
        }
        previous_center = matrixK_*direction;
        first = cv::Point (int(cvRound(previous_center(1,0))), int(cvRound(previous_center(2,0))));
        */
        message.header.frame_id = "direction d=invK*u";

        //orientation values
      	message.pose.orientation.x = direction.at<double>(0,0);
      	message.pose.orientation.y = direction.at<double>(1,0);
      	message.pose.orientation.z = direction.at<double>(2,0);
        //publish the message
      	dir_ball.publish(message);


        //sets and draw a bounding box around the ball
        box.x = (cv_img_ptr_in_->image.cols/2)-10;
        box.y = (cv_img_ptr_in_->image.rows/2)-10;
        box.width = 20;
        box.height = 20;
        cv::rectangle(cv_img_out_.image, box, cv::Scalar(0,255,255), 3);
    }

    //reset input image
    cv_img_ptr_in_ = nullptr;
}

void RosImgProcessorNode::publish()
{
    //image_raw topic
	if(cv_img_out_.image.data)
	{
	    cv_img_out_.header.seq ++;
	    cv_img_out_.header.stamp = ros::Time::now();
	    cv_img_out_.header.frame_id = "camera";
	    cv_img_out_.encoding = img_encoding_;
	    image_pub_.publish(cv_img_out_.toImageMsg());
	}
  //A Pose with reference coordinate frame and timestamp
  /*geometry_msgs::PoseStamped message;
  // POSE--- A representation of pose in free space, composed of position and orientatio
  message.pose.position.x = 0.0;
  message.pose.position.y = 0.0;
  message.pose.position.z = 0.0;
  message.pose.orientation.w = 0;
	message.pose.orientation.x = a.at<double>(0,0);
	message.pose.orientation.y = a.at<double>(1,0);
	message.pose.orientation.z = a.at<double>(2,0);
	dir_ball.publish(message);*/
}


double RosImgProcessorNode::getRate() const
{
    return rate_;
}

void RosImgProcessorNode::imageCallback(const sensor_msgs::ImageConstPtr& _msg)
{
    try
    {
        img_encoding_ = _msg->encoding;//get image encodings
        cv_img_ptr_in_ = cv_bridge::toCvCopy(_msg, _msg->encoding);//get image
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("RosImgProcessorNode::image_callback(): cv_bridge exception: %s", e.what());
        return;
    }
}

void RosImgProcessorNode::cameraInfoCallback(const sensor_msgs::CameraInfo & _msg)
{
	matrixP_ = (cv::Mat_<double>(3,3) << _msg.P[0],_msg.P[1],_msg.P[2],
                                        _msg.P[3],_msg.P[4],_msg.P[5],
                                        _msg.P[6],_msg.P[7],_msg.P[8]);
  matrixK_ = (cv::Mat_<double>(3,3) << _msg.K[0],_msg.K[1],_msg.K[2],
                                       _msg.K[3],_msg.K[4],_msg.K[5],
                                       _msg.K[6],_msg.K[7],_msg.K[8]);
  cv::invert(matrixK_, invK_, cv::DECOMP_LU) ;
  //Gaussian elimination with the optimal pivot element chosen. returns no-zero value if
  //inverse has been successfully calculated and 0 if input_matrix is singular
  //std::cout << matrixP_ << std::endl;
}
