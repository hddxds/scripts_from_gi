#include "run.h"

using namespace std;
using namespace cv;
 


cv_bridge::CvImagePtr cv2ros(cv::Mat& img)
{
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
 
#if 1
    //NOTE change it to bgr8 if you want to use RGB image
    cv_ptr->encoding = "mono8";

    cv_ptr->header.frame_id = "gi";
#endif
    
    cv_ptr->image = img;
    
    return  cv_ptr;
}

int main(int argc, char** argv){
 
    // Create a VideoCapture object and open the input file
    // If the input is the web camera, pass 0 instead of the video file name
    cv::VideoCapture cap(1); 

    // Check if camera opened successfully
    if(!cap.isOpened())
    {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }
    
    cap.set(3,1280.0);
    cap.set(4,480.0);
    
    ros::init(argc, argv, "gi_custom_camera");
    ros::NodeHandle nh;
    
    ros::Publisher gi_camera_left = nh.advertise<sensor_msgs::Image>("gi/camera_raw/left",10);
    ros::Publisher gi_camera_right = nh.advertise<sensor_msgs::Image>("gi/camera_raw/right",10);
    ros::Publisher gi_camera_stereo = nh.advertise<sensor_msgs::Image>("gi/camera_raw/stereo_image",10);

    while(1)
    {
    
        cv::Mat frame, frame_left, frame_right, gray_frame;
        
        cap >> frame;
        
        // If the frame is empty, break immediately
        if (frame.empty())
            break;
        

        cv::cvtColor(frame, frame, CV_BGR2GRAY);
        
        // Display the resulting frame
        imshow("Frame", frame);
        
        // Press  ESC on keyboard to exit
        char c=(char)waitKey(25);
        if(c==27)
            break;
        
        frame_left  = frame(cv::Range(0, frame.rows - 1), cv::Range(0, frame.cols / 2 - 1));
        frame_right = frame(cv::Range(0, frame.rows - 1), cv::Range(frame.cols / 2 - 1, frame.cols-1));
	
	cv_bridge::CvImagePtr stereo_ptr = cv2ros(frame);
        cv_bridge::CvImagePtr left_ptr = cv2ros(frame_left);
        cv_bridge::CvImagePtr right_ptr = cv2ros(frame_right);

        sensor_msgs::Image msgl = *(left_ptr->toImageMsg());
        sensor_msgs::Image msgr = *(right_ptr->toImageMsg());
        msgr.header.stamp = msgl.header.stamp;

        gi_camera_left.publish(left_ptr->toImageMsg());
        gi_camera_right.publish(right_ptr->toImageMsg());
	gi_camera_stereo.publish(stereo_ptr->toImageMsg());

    }
    
    // When everything done, release the video capture object
    cap.release();
    
    // Closes all the frames
    destroyAllWindows();
        
    return 0;
  
  
}
