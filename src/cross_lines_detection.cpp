
#include <ros/ros.h>
#include <vector>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "tf/LinearMath/Transform.h"
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <pcl/filters/extract_indices.h>


using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//! Class Rcomponent
class CrossLinesDetection
{
protected:
  //!	Saves the name of the component
  string component_name_;
  //! ROS node handle
  ros::NodeHandle nh_;
  //! Private ROS node handle
  ros::NodeHandle pnh_;
  //! ROS publishers  
  ros::Publisher pub_filtered_cloud_ ;
  //! ROS subscribers
  ros::Subscriber sub_scan_;  
  //! Parameters
  string scan_in_topic_name_;
  double row_width_, ransac_threshold_;
  bool publish_filtered_cloud_ , new_cloud_processed_;
  //! Variables
  geometry_msgs::PoseStamped line_pose_, right_line_pose__, look_ahead_pose_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener tfListener_;
  std::vector<pcl_msgs::ModelCoefficients> lines_vector;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_vector;
  int line_numb;
  //pcl::ModelCoefficients line_coefficients;

	


public:
  //! Public constructor
  CrossLinesDetection(ros::NodeHandle h);
  //! Public destructor
  ~CrossLinesDetection();

  //! Configures and initializes the component
  //! @return 0
  //! @return -1
  int setup();
  //! Closes and frees the reserved resources
  //! @return 0
  //! @return -1
  int shutdown();
  //! All core component functionality is contained in this thread.
  //!	All of the CrossLinesDetection component state machine code can be found here.
  void start();

private:
  void cloudInHandler(const sensor_msgs::LaserScan::ConstPtr& scan);
  bool detectLines(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int line_numb);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointIndices inliers);
  //bool getLineEnd(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointIndices inliers, geometry_msgs::Point &line_end_point);
};

/*! \fn CrossLinesDetection::CrossLinesDetection()
 *  \brief Constructor by default
 *	\param h as ros::NodeHandle, ROS node handle
*/
CrossLinesDetection::CrossLinesDetection(ros::NodeHandle h) : nh_(h), pnh_("~")
{
  component_name_.assign("CrossLinesDetection");
}

/*! \fn CrossLinesDetection::~CrossLinesDetection()
 * Destructor by default
*/
CrossLinesDetection::~CrossLinesDetection()
{
}

/*! \fn int CrossLinesDetection::setup()
 * Configures and initializes the component
 * \return 0
 * \return -1
*/
int CrossLinesDetection::setup()
{
  // Ros params
  pnh_.param<string>("scan_in_topic_name", scan_in_topic_name_, "input");
  pnh_.param("ransac_threshold", ransac_threshold_, 0.1);
  pnh_.param("publish_filtered_cloud", publish_filtered_cloud_, true);

  // Topics
  tfListener_.setExtrapolationLimit(ros::Duration(0.1)); 
  sub_scan_ = nh_.subscribe(scan_in_topic_name_, 1, &CrossLinesDetection::cloudInHandler, this);
  //pub_line_ = nh_.advertise<geometry_msgs::PoseStamped>("line_pose", 1);
  pub_filtered_cloud_ = nh_.advertise<PointCloud>("line_cloud", 1);
  //pub_line_end_ = nh_.advertise<geometry_msgs::PointStamped>("line_end", 1);
  
  new_cloud_processed_ = true;


  return -1;
}

/*! \fn int CrossLinesDetection::shutDown()
 * Closes and frees the reserved resources
 * \return 0
 * \return -1
*/
int CrossLinesDetection::shutdown()
{
  return 0;
}

/*!	\fn void CrossLinesDetection::start()
 *	\brief All core component functionality is contained in this thread.
*/
void CrossLinesDetection::start()
{
  ROS_INFO("%s::start(): Init", component_name_.c_str());

  ros::spin();

  ROS_INFO("%s::start(): End", component_name_.c_str());
}

/*!	\fn void CrossLinesDetection::cloudInHandler(const sensor_msgs::PointCloud2ConstPtr& input)
 *	\brief Handler to receive the filtered pointcloud
*/
void CrossLinesDetection::cloudInHandler(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr actual_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	sensor_msgs::PointCloud2 input;
	bool line_detected_;
	
	projector_.transformLaserScanToPointCloud("base_link", *scan, input, tfListener_);
	pcl::fromROSMsg(input, *cloud);

	
	if(new_cloud_processed_)
	{
		actual_cloud = cloud;
		new_cloud_processed_ = false;
		line_numb = 0;
	}
	// Process actual cloud
	line_detected_ = detectLines(actual_cloud, line_numb);
	
	// Calculate angles of rows from line_pose_. TODO filter.
	//line_angle_ = tf::getYaw(line_pose_.pose.orientation); 
	//pub_line_.publish(line_pose_);
}

/*!	\fn bool CrossLinesDetection::detectLines(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int line_numb, geometry_msgs::PoseStamped& line_pose_)
 *	\brief gets the rows of the left and right vineyard, and publishes the end point of each row
*/
bool CrossLinesDetection::detectLines(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int line_numb)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	//geometry_msgs::PointStamped line_end_point;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    //seg.setMaxIterations(1000);
    //seg.setDistanceThreshold(ransac_threshold_);
    seg.setDistanceThreshold(0.005);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) cloud->points.size ();
     // While 10% of the original cloud is still there
     while (cloud->points.size () > 0.1 * nr_points){
         // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
              ROS_INFO("Could not estimate a planar model for the given dataset.");
              break;
            }

            // Extract the inliers
            extract.setInputCloud (cloud);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_line);
            ROS_INFO("PointCloud representing the line component: %d data points", cloud_line->width * cloud_line->height);

            //SAVE LINE IN VECTOR OF LINES

            // Create the filtering object
            extract.setNegative (true);
            extract.filter (*cloud_filter);
            cloud.swap (cloud_filter);
            i++;
     }
  
//	/*if (cloud->points.size() > 2)
//	{
//        seg.segment(*inliers, *coefficients);
//		/* Coefficients for a model of type pcl::SACMODEL_LINE are: [x, y, z, dir_x, dir_y, dir_z] */
//		cloud_filtered = cropCloud(cloud, inliers);
//		clouds_vector.push_back(cloud_filtered);
//        lines_vector.push_back(*coefficients);
//		line_numb++;
//		//pcl_conversions::fromPCL(coefficients, ros_coefficients);
//		// Publish filtered cloud for debug
//		if(publish_filtered_cloud_)
//		{
//			pub_filtered_cloud_.publish(cloud_filtered);
//		}
//	}
//	else
//	{
//		ROS_INFO("Cloud Processed");
//		return 0;
//	}
	/*
	// Pose fill
	line_pose_.header = ros_coefficients.header;
	line_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(ros_coefficients.values[4] / ros_coefficients.values[3]);
	line_pose_.pose.position.x = 0.0;
	line_pose_.pose.position.y = ros_coefficients.values[4] / ros_coefficients.values[3] + ros_coefficients.values[1] - ros_coefficients.values[4] / ros_coefficients.values[3] * ros_coefficients.values[0];
	line_pose_.pose.position.z = 0.0;

	// Get and publish row end
	if(getLineEnd(cloud_filtered, inliers, line_end_point.point))
	{
	  line_end_point.header = ros_coefficients.header;
	  pub_line_end_.publish(line_end_point);
	}

	// EXCEPTIONS 
	if (line_pose_.pose.position.y * line_numb <= 0)  // If a row is detected on the oposite line_numb.
	{
	  ROS_DEBUG("[CrossLinesDetection::detectLines] %s : Oposite row is detected. %5.2f", (line_numb < 0 ? "right" : "left"), line_pose_.pose.position.y);
	  if (line_numb < 0)
		rows_status_msgs_.status_right = 1;
	  else
		rows_status_msgs_.status_left = 2;
	  return 0;
	}
	*/
}

/*!	\fn geometry_msgs::Pose CrossLinesDetection::cropCloud()
 *	\brief returns pointcloud with the line in cloud_filtered and extract those from the original pcl
*/
//pcl::PointCloud<pcl::PointXYZ>::Ptr CrossLinesDetection::cropCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointIndicesPtr& inliers)
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//	// Copies all inliers of the model computed to another PointCloud
//	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_filtered);
//	// Extract fInliers from the input cloud
//	pcl::ExtractIndices<pcl::PointXYZ> extract ;
//	extract.setInputCloud (cloud);
//	extract.setIndices (inliers);
//	//extract.setNegative (false); //Removes part_of_cloud but retain the original full_cloud
//	extract.setNegative (true); // Removes part_of_cloud from full cloud  and keep the rest
//	extract.filter (*cloud);

//	return cloud_filtered;
//}

/*!	\fn bool CrossLinesDetection::getLineEnd(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointIndices inliers, geometry_msgs::PointStamped line_end_point)
 *	\brief gets the end point of the row defined by the input cloud and the RANSAC inliers

bool CrossLinesDetection::getLineEnd(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointIndices inliers, geometry_msgs::Point &line_end_point)
{
  float max_distance = -1;
  int end_point_index = -1;
  for(int i=0; i<inliers.indices.size(); i++)
  {
    float distance = cloud->at(inliers.indices[i]).x;
    
    if(distance > max_distance)
    {
      end_point_index = i;
      max_distance = distance;
    }
  }
  
  if(end_point_index == -1)
  {
    line_end_point.x = line_end_point.y = line_end_point.z = -1; // undefined
    return false;
  }
  else
  {
    line_end_point.x = cloud->at(end_point_index).x;
    line_end_point.y = cloud->at(end_point_index).y;
    line_end_point.z = cloud->at(end_point_index).z;
    //ROS_INFO("CrossLinesDetection::getLineEnd: further point at distance: %f", max_distance);
    return true;
  }
}
*/
// MAIN
int main(int argc, char** argv)
{
  ros::init(argc, argv, "row_detection_node");

  ros::NodeHandle n;
  CrossLinesDetection row_detection(n);

  row_detection.setup();

  row_detection.start();

  row_detection.shutdown();

  return (0);
}
