#include <sstream>
#include <boost/filesystem.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>

typedef sensor_msgs::PointCloud2 PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

int create_all_directories(std::string rootdir, std::vector<std::string> topics)
{

  for (auto topic: topics)
  {
    auto dir = rootdir + topic;
    boost::filesystem::path outpath(dir);
    if (!boost::filesystem::exists (outpath))
    {
      if (!boost::filesystem::create_directories (outpath))
      {
        std::cerr << "Error creating directory " << dir << std::endl;
        return (-1);
      }
      std::cerr << "Creating directory " << dir << std::endl;
    }
  }

  return 0;
}


int main (int argc, char** argv)
{
  //ros::init (argc, argv, "bag_to_pcd");
  if (argc < 3)
  {
    std::cerr << "Syntax is: " << argv[0] << " <file_in.bag> <output_directory>" << std::endl;
    std::cerr << "Example: " << argv[0] << " data.bag ./dataset" << std::endl;
    return (-1);
  }

  rosbag::Bag bag;
  

  try
  {
    bag.open (argv[1], rosbag::bagmode::Read);
  } 
  catch (rosbag::BagException) 
  { 
    std::cerr << "Error opening file " << argv[1] << std::endl;
    return (-1);
  }


  rosbag::View view(bag);
  

  // read all topics
  std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();
  std::vector<std::string> topics;

  for(auto conn_info: connection_infos)
  {
    std::cout<< conn_info->topic <<std::endl;
    topics.push_back(conn_info->topic);
  }

  //
  create_all_directories(argv[2], topics);

  rosbag::View::iterator view_it;
  view_it = view.begin ();
 

  while (view_it != view.end ())
  {

    auto topic =  view_it->getTopic();
    std::cout<<topic<<std::endl;
    std::cout<<"timestame in view "<<view_it->getTime()<<std::endl;

    sensor_msgs::CompressedImageConstPtr img_compressed = view_it->instantiate<sensor_msgs::CompressedImage> ();
    if (img_compressed != NULL){

       std::cout<<"timestame in pack "<<img_compressed->header.stamp<<std::endl;
       cv::Mat image;
      
        try
        {
          image = cv::imdecode(cv::Mat(img_compressed->data),1);      
        } catch(cv_bridge::Exception)
        {
          std::cerr<<"decode msg failed"<<std::endl;
        }
      
        if (!image.empty()) {

          std::stringstream ss;
          ss << argv[2] + topic <<"/" << img_compressed->header.stamp <<".jpg";
          std::string filename = ss.str();
          cv::imwrite(filename, image);
          std::cout<<"Saved image (" <<img_compressed->format.c_str() <<") " << filename << std::endl;
          //count_++;
        } else {
          std::cerr<<"Couldn't save image, no data!"<<std::endl;
        }
    }

    sensor_msgs::ImageConstPtr img = view_it->instantiate<sensor_msgs::Image> ();
    if (img != NULL)
    {
      std::cout<<"timestame in pack "<<img->header.stamp<<std::endl;
      cv::Mat image;
      try
      {
        //image = cv::imdecode(cv::Mat(msg->data),1);

        image = cv_bridge::toCvShare(img, "bgr8")->image;
      } catch(cv_bridge::Exception)
      {
        std::cerr<<"decode msg failed"<<std::endl;
      }
    
      if (!image.empty()) {

        std::stringstream ss;
        ss << argv[2]+topic <<"/" << img->header.stamp <<".jpg";
        std::string filename = ss.str();
        

        cv::imwrite(filename, image);
        std::cout<<"Saved image  "<< filename <<std::endl;
        
      } else {
        std::cerr<< "Couldn't save image, no data!"<<std::endl;
      }
    }
    
    // Get the PointCloud2 message
    PointCloudConstPtr cloud = view_it->instantiate<PointCloud> ();
    if (cloud != NULL)
    {
      PointCloud cloud_t;
      cloud_t = *cloud;
            
      std::cerr << "Got " << cloud_t.width * cloud_t.height << " data points in frame " << cloud_t.header.frame_id << " with the following fields: " << pcl::getFieldsList (cloud_t) << std::endl;

      std::stringstream ss;
      ss << argv[2] + topic << "/" << cloud_t.header.stamp << ".pcd";
      std::cerr << "Data saved to " << ss.str () << std::endl;
      pcl::io::savePCDFile (ss.str (), cloud_t, Eigen::Vector4f::Zero (),
                            Eigen::Quaternionf::Identity (), true);
    }
   
    
    ++view_it;
  }

  return (0);
}


