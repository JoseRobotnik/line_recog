
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
#include <pcl/common/geometry.h>

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
#include <pcl/common/intersections.h>


using namespace std;

struct segment_t{
   pcl::PointCloud<pcl::PointXYZ> line_cloud;
   pcl::ModelCoefficients coeffs;
};

struct intersection_t{
    pcl::PointCloud<pcl::PointXYZ> lines[2];
    pcl::PointXYZ point;
    float angle;
};



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
  ros::Publisher line_cloud_pub_;
  //! ROS subscribers
  ros::Subscriber sub_scan_;  
  //! Parameters
  string scan_in_topic_name_;
  double row_width_, ransac_threshold_;
  bool publish_filtered_cloud_ , new_cloud_processed_;
  //! Variables
  geometry_msgs::PoseStamped line_pose_, right_line_pose__, look_ahead_pose_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
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
  pnh_.param<string>("scan_in_topic_name", scan_in_topic_name_, "hokuyo/scan");
  pnh_.param("ransac_threshold", ransac_threshold_, 0.005);
  pnh_.param("publish_filtered_cloud", publish_filtered_cloud_, true);

  // Topics
  //tfListener_.setExtrapolationLimit(ros::Duration(0.1));
  sub_scan_ = nh_.subscribe(scan_in_topic_name_, 1, &CrossLinesDetection::cloudInHandler, this);
  //pub_line_ = nh_.advertise<geometry_msgs::PoseStamped>("line_pose", 1);
  pub_filtered_cloud_ = nh_.advertise<PointCloud>("line_cloud", 1);
  //pub_line_end_ = nh_.advertise<geometry_msgs::PointStamped>("line_end", 1);
  line_cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("line_cloud_publisher", 10);
  
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
	
    //waitForTransform blocks until the transform is possible or time out.
    if(!listener_.waitForTransform(scan->header.frame_id, "/base_link", scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
            ros::Duration(1.0))){
        ROS_WARN("Transform from %s to /base_link not exist!", scan->header.frame_id.c_str());
        return;
    } //TODO TEST IT
    projector_.transformLaserScanToPointCloud("base_link", *scan, input, listener_);
    //projector_.transformLaserScanToPointCloud("base_link", *scan, input, tfListener_);
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
    Eigen::Vector4f point;
    std::vector<segment_t> lines;
    std::vector<Eigen::Vector4f> intersect_points;
    std::vector<intersection_t> intersections;

    segment_t segment;
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
    seg.setDistanceThreshold(0.03);

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
              ROS_INFO("Could not estimate a line model for the given dataset.");
              break;
            }

            // Extract the inliers
            extract.setInputCloud (cloud);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_line);
            ROS_INFO("ANCHURA: %d", cloud_line->width);
            ROS_INFO("ALTURA: %d", cloud_line->height);

            ROS_INFO("PointCloud representing the line component: %d data points", cloud_line->width * cloud_line->height);

            //line_cloud_pub_.publish(cloud_line);

            //Store segments in vector
            segment.coeffs = *coefficients;
            segment.line_cloud = *cloud_line;
            lines.push_back(segment);

            // Create the filtering object
            extract.setNegative (true);
            extract.filter (*cloud_filter);
            cloud.swap (cloud_filter);
            i++;
     }

//     for(std::vector<segment_t>::iterator it = lines.begin(); it != lines.end(); ++it) {

//     }

     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_lines_extracted(new pcl::PointCloud<pcl::PointXYZI>);
     cloud_lines_extracted->header.frame_id = "hokuyo_laser_link";
     for (int i=0; i < lines.size(); i++) {
         int current_size=cloud_lines_extracted->points.size();
         cloud_lines_extracted->points.resize(current_size + lines[i].line_cloud.points.size());
         for (int j = 0; j < lines[i].line_cloud.points.size(); j++) {
             cloud_lines_extracted->points[j+current_size].x = lines[i].line_cloud.points[j].x;
             cloud_lines_extracted->points[j+current_size].y = lines[i].line_cloud.points[j].y;
             cloud_lines_extracted->points[j+current_size].z = lines[i].line_cloud.points[j].z;
             cloud_lines_extracted->points[j+current_size].intensity = 255*(i)/(lines.size() + 5) ;
         }
     }
     cloud_lines_extracted->width= cloud_lines_extracted->points.size();
     cloud_lines_extracted->height = 1;


     for(int i=0;i<lines.size();++i){
         for(int j=i+1;j<lines.size();++j){
             float min_distance_between_lines = 0.01;
                bool intersect = pcl::lineWithLineIntersection (lines[i].coeffs, lines[j].coeffs, point, min_distance_between_lines);
                if(intersect){

                    //ROS_INFO("Intersection point: x: %f y: %f z: %f w: %f", point[0], point[1], point[2], point[3]);
                    intersect_points.push_back(point);

                    float min_distance_to_segment_i= 10000; //TODO: poner std::limits::float o algo asi
                    for (int l = 0; l < lines[i].line_cloud.points.size(); l++) {
                        Eigen::Vector3f p;
                        p[0] = point[0] - lines[i].line_cloud.points[l].x;
                        p[1] = point[1] - lines[i].line_cloud.points[l].y;
                        p[2] = point[2] - lines[i].line_cloud.points[l].z;

                        float distance = p[0]*p[0] + p[1]*p[1] + p[2]*p[2];
                        if (distance < min_distance_to_segment_i)
                            min_distance_to_segment_i = distance;
                    }
                    float min_distance_to_segment_j= 10000; //TODO: poner std::limits::float o algo asi
                    for (int l = 0; l < lines[j].line_cloud.points.size(); l++) {
                        Eigen::Vector3f p;
                        p[0] = point[0] - lines[j].line_cloud.points[l].x;
                        p[1] = point[1] - lines[j].line_cloud.points[l].y;
                        p[2] = point[2] - lines[j].line_cloud.points[l].z;

                        float distance = p[0]*p[0] + p[1]*p[1] + p[2]*p[2];
                        if (distance < min_distance_to_segment_j)
                            min_distance_to_segment_j = distance;
                    }
                    float distance_to_segment_threshold = 0.05;
                    if (min_distance_to_segment_i < distance_to_segment_threshold && min_distance_to_segment_j < distance_to_segment_threshold )
                    {
                         intersection_t current_intersection;
                         current_intersection.lines[0] = lines[i].line_cloud;
                         current_intersection.lines[1] = lines[j].line_cloud;
                         current_intersection.point.x = point[0];
                         current_intersection.point.y = point[1];
                         current_intersection.point.z = point[2];

                         Eigen::Vector4f directions[2];
                         directions[0][0] = lines[i].coeffs.values[3];
                         directions[0][1] = lines[i].coeffs.values[4];
                         directions[0][2] = lines[i].coeffs.values[5];
                         directions[0][3] = 0;

                         directions[1][0] = lines[j].coeffs.values[3];
                         directions[1][1] = lines[j].coeffs.values[4];
                         directions[1][2] = lines[j].coeffs.values[5];
                         directions[1][3] = 0;

                         current_intersection.angle = pcl::getAngle3D(directions[1], directions[0]);


                         intersections.push_back(current_intersection);

                         ROS_INFO("The segments intersect! Angle: %f", current_intersection.angle);
                         pcl::PointXYZI segment_intersection;
                         segment_intersection.x = point[0];
                         segment_intersection.y = point[1];
                         segment_intersection.z = point[2];
                         segment_intersection.intensity = 255;
                         cloud_lines_extracted->points.push_back(segment_intersection);
                         cloud_lines_extracted->width = cloud_lines_extracted->points.size();
                    }
                }
        }
     }



     line_cloud_pub_.publish(cloud_lines_extracted);

     //TODO: agrupar las interseccions de 2 en 2, en funcion de si comparten un segmento o no.
     //Si una interseccion no comparte segmento con otra, se descarta

     for (int i = 0; i < intersections.size(); i++) {

     }

     //ITERATOR?
     //ITER VECTOR
  
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
