#include <cstdio>
#include "tf/transform_listener.h"
#include "ros/ros.h"
#include <fstream>

#define _USE_MATH_DEFINES

//rosrun real_ur5_test ur5_pose_record base tool0_controller 30


class pose_cal
{
public:

  tf::TransformListener tf;
  ros::NodeHandle nh_;
  std::string source_frameid;
  std::string target_frameid;
  double rate_hz;
  int precision=4;
  
  //constructor with name
  pose_cal(ros::NodeHandle n_,std::string source_frameid_,std::string target_frameid_,double rate_hz_=0)
  {
    this->nh_=n_;
    this->source_frameid=source_frameid_;
    this->target_frameid=target_frameid_;
    this->rate_hz=rate_hz_;
  }

  ~pose_cal()
  {

  }


 void prepare_pose_cal()
 {
    //ros::NodeHandle nh("~");

  if(rate_hz==0)
  {
    // read rate parameter
    nh_.param("rate", rate_hz, 1.0);
  }
  
// int precision(3);
  if (nh_.getParam("precision", precision))
  {
    if (precision < 1)
    {
      std::cerr << "Precision must be > 0\n";
    }
    printf("Precision default value was overriden, new value: %d\n", precision);
  }

  // Wait for up to one second for the first transforms to become avaiable. 
 tf.waitForTransform(source_frameid, target_frameid, ros::Time(), ros::Duration(1.0));

 }

 void gain_pose()
 {
   ros::Rate rate(rate_hz);
   std::ofstream outfile;
   while(nh_.ok())
    {
      try
      {
        outfile.open("/home/kinws/src/real_ur5_test/pose_same_sample_quaternion.txt",std::ios::app | std::ios::out);
        tf::StampedTransform echo_transform;
        tf.lookupTransform(source_frameid, target_frameid, ros::Time(), echo_transform);
        std::cout.precision(precision);
        std::cout.setf(std::ios::fixed,std::ios::floatfield);
        std::cout << "At time " << echo_transform.stamp_.toSec() << std::endl;
        double yaw, pitch, roll;
        echo_transform.getBasis().getRPY(roll, pitch, yaw);
        tf::Quaternion q = echo_transform.getRotation();
        tf::Vector3 v = echo_transform.getOrigin();
        std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
        std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", " 
                  << q.getZ() << ", " << q.getW() << "]" << std::endl
                  << "            in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl
                  << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI << ", " << yaw*180.0/M_PI << "]" << std::endl;

        //print transform

        //保存到本地
        
      outfile<<v.getX()<<"\t"<<v.getY()<<"\t"<<v.getZ()<<"\t"<<q.getX()<<"\t"<<q.getY()<<"\t"<<q.getZ()<<"\t"<<q.getW()<<std::endl;

      //outfile<<v.getX()<<"\t"<<v.getY()<<"\t"<<v.getZ()<<"\t"<<roll<<"\t"<<pitch<<"\t"<<yaw<<std::endl;

        outfile.close();
      }
      catch(tf::TransformException& ex)
      {
        std::cout << "Failure at "<< ros::Time::now() << std::endl;
        std::cout << "Exception thrown:" << ex.what()<< std::endl;
        std::cout << "The current list of frames is:" <<std::endl;
        std::cout << tf.allFramesAsString()<<std::endl;
        
      }
      rate.sleep();
    }
 }
};


int main(int argc, char ** argv)
{
  //Initialize ROS
  ros::init(argc, argv, "ur5_pose_record", ros::init_options::AnonymousName);

  // Allow 2 or 3 command line arguments
  if (argc < 3 || argc > 4)
  {
    printf("Usage: tf_echo source_frame target_frame [echo_rate]\n\n");
    printf("This will echo the transform from the coordinate frame of the source_frame\n");
    printf("to the coordinate frame of the target_frame. \n");
    printf("Note: This is the transform to get data from target_frame into the source_frame.\n");
    printf("Default echo rate is 1 if echo_rate is not given.\n");
    return -1;
  }

  
  ros::NodeHandle nh("~");
  std::string source_frameid_=std::string(argv[1]);
  std::string target_frameid_=std::string(argv[2]);
  double rate_hz_=0;
  if (argc==4)
  {
     rate_hz_= atof(argv[3]);
  }
  pose_cal pose(nh,source_frameid_,target_frameid_,rate_hz_);
  pose.prepare_pose_cal();
  pose.gain_pose();
  return 0;
}