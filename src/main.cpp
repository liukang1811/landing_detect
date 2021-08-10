#include <fstream>
#include <iostream>
#include <cstring>
#include <stdlib.h>
#include <stdio.h>
#include "ros/time.h"
#include "ros/ros.h"
#include "motion_state.h"
#include "arrowtorviz.h"
#include <algorithm>
#include "pointcloud_common.h"
#include "ros_publish_common.h"
#include <boost/thread/mutex.hpp>

#include <boost/thread/thread.hpp>
#define numberet 257

using namespace std;

class SubscribeAndPublish  
{  
public:

  SubscribeAndPublish(ros::NodeHandle& ns, std::string params_path): grid3d(Eigen::Vector3f(200, 100, 100), Eigen::Vector3f(0.5, 0.5, 0.5)),
                                            feature_map(Eigen::Vector3f(0.2, 0.2, 0.2))
  {
    sub2 = ns.subscribe("/xs_points", 1000, &SubscribeAndPublish::chatterCallback2, this);
    sub = ns.subscribe("/zvision_lidar_points", 1000, &SubscribeAndPublish::chatterCallback, this);
    sub3 = ns.subscribe("/imu/data", 1000, &SubscribeAndPublish::chatterCallback3, this);
    pub = ns.advertise<sensor_msgs::PointCloud2>("raw_points2", 1);
    pub2 = ns.advertise<sensor_msgs::PointCloud2>("detected_result", 10);
    pub4 = ns.advertise<geometry_msgs::PoseArray>("pose_", 10);
    pub5 = ns.advertise<sensor_msgs::PointCloud2>("boundingbox_points", 10);
    markerArrayPub = ns.advertise<visualization_msgs::MarkerArray>("MarkerId", 10);
    ArrayIdPub = ns.advertise<visualization_msgs::MarkerArray>("maker_id2", 10);
    posearray.header.frame_id = "daa_link";
    posearray.header.stamp = ros::Time::now();
    posearray.header.seq = 0;

  }
  void chatterCallback2(const sensor_msgs::PointCloud2::ConstPtr &s)
  {
      double start = ros::Time::now().toSec();
      //pub5.publish(pointcloud);
      double end = ros::Time::now().toSec();
      cout << "i heared xspoints  " << std::setprecision(20)<<s->header.stamp.toSec() << endl;

  }

  void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr &s)
  {
    Ros_receive_publish::range_info range(-6, 6, -50, 50, -10, 10);
    double start = ros::Time::now().toSec();
    pointcloud_common::pointcloud_t points = Ros_receive_publish::ros_reveice_common::Rospointcloud2pointcloud_t(*s, 1, range);
    pointcloud_common::pointcloud_common::Rotationpoints(pose, points);
    pointcloud_common::occupied_grid3d grid(Eigen::Vector3f(20, 12, 300), Eigen::Vector3f(0.2, 0.2, 0.2));
    grid.CreateMap(points);
    pointcloud_common::keymap feat;
    std::unordered_map<int, pointcloud_common::pointfeature > ::iterator  itt0 = grid.result_map.begin();
    while(itt0 != grid.result_map.end())
    {
        Eigen::Vector3f position = grid.GetPositionByindex(itt0->first);
        pointcloud_common::KEY key(floor(position[0] / grid.resolution[0]), floor(position[1] / grid.resolution[1]),
                                   floor(itt0->second.totalpoints.rbegin()->first / grid.resolution[2]) );
        pointcloud_common::grid_feature feature;
        feature.minheight = 0;
        feature.maxheight = itt0->second.totalpoints.rbegin()->first;
        feature.total_points = itt0->second.totalpoints.size();
        feature.x = position[0];
        feature.y = position[1];
        ++itt0;
        feat.insert(std::pair<pointcloud_common::KEY, pointcloud_common::grid_feature>(key, feature));
    }
//    pointcloud_common::ClusterRegionExpand::grid_grouping(Eigen::Vector3i(2,2,2), 3, feat, Eigen::Vector3f(24,200,40),grid.result_map, grid.resolution );
//    pointcloud_common::Feature_map feat = grid.GetFeatuerMap();
    std::unordered_map<int, pointcloud_common::pointfeature > ::iterator  itt = grid.result_map.begin();
    double end = ros::Time::now().toSec();
    pointcloud_common::keymap::iterator itt2 = feat.begin();
    pointcloud_common::pointcloud_t pointclouds;
    std::cout << grid.result_map.size() << "pointclouds.points.size()"<< std::endl;
    while(itt2 != feat.end())
    {
       pointcloud_common::point_type point(itt2->first.x* 0.2,itt2->first.y * 0.2, itt2->second.maxheight, itt2->second.cluster_index);
       pointclouds.points.push_back(point);
       ++itt2;
    }
    sensor_msgs::PointCloud2 pointsc;
    Ros_receive_publish::ros_publish_common::pubcloud_fill(pointsc, "daa_link", 16, (float*)pointclouds.points.data(), pointclouds.points.size());
    pub2.publish(pointsc);
    /////////////////////////////////////////////////////////////////////////////



////
//    pointcloud_common::keymap::iterator itt = grid3d.cluster_map.begin();
//    while(itt != grid3d.cluster_map.end())
//    {
//        points2.points.push_back(pointcloud_common::point_type (itt->first.x * 0.5, itt->first.y * 0.5,  itt->first.z * 0.5, itt->second.cluster_index));
//        ++itt;
//    }

    sensor_msgs::PointCloud2 pub_points;
//    Ros_receive_publish::ros_publish_common::pubcloud_fill(pub_points, "daa_link", 16, (float*)points2.points.data(), points2.width);
//    sensor_msgs::PointCloud2 pub_boundingbox_points;
//    Ros_receive_publish::ros_publish_common::pubcloud_fill(pub_boundingbox_points, "daa_link", 16, bboxdata.data(), bboxdata.size() / 4.0);
//    pub5.publish(pub_boundingbox_points);
//    pub.publish(pub_points);
    cout << "i heared rspoints  "<< end - start <<" size " <<pose[0] / 3.14 * 180.0 << endl;
    ///////////////////////////////////////////////////////////////
    
  }
  void chatterCallback3(const sensor_msgs::Imu s)
  {
    double start = ros::Time::now().toSec();
    posearray.poses.clear();
    tf::Quaternion quat;
    tf::quaternionMsgToTF(s.orientation, quat);
    double roll, pitch, yaw;//定义存储r\p\y的容器
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换
    Eigen::Vector3d angle(roll, pitch, yaw);
    if (roll < 0)
    {
        pose[1] = (-3.141592654 - roll);
    }
    else
    {
        pose[1] = (3.141592654 - roll);
    }
    pose[0] = pitch - 3.1415926 / 2;
    pose[2] = 0;
//    ArrowToRviz arrow( angle, Eigen::Vector3d(0,0,0));
//    posearray.poses.push_back(arrow.pose);
//    pub4.publish(posearray);
//    cout << "i heared imu data   "<< angle[1] / 3.14 * 180.0 << "    "<< std::setprecision(20)<<s.header.stamp.toSec() << endl;

  }
  void timecallback(const ros::TimerEvent& event)
  {
      visualization_msgs::MarkerArray pubmarkers;
      count1 = 0;
      mutex.lock();
      pointcloud_common::keymap::iterator itg = feature_map.static_map.begin();
      while (itg != feature_map.static_map.end())
      {
          count1 ++;
          if (count1 >= 10000) count1 = 0;
          visualization_msgs::Marker item;
          {
              item = Ros_receive_publish::ros_publish_common::CudeMarkerCreate(Eigen::Vector3f(itg->first.x* 0.5,itg->first.y* 0.5,(itg->second.minheight + itg->second.maxheight) / 2.0),
                                                                               Eigen::Vector3f(0.5,0.5, max(float(0.1),(itg->second.maxheight - itg->second.minheight))), count1, "daa_link",
                                                                               Eigen::Quaternionf::Identity(),
                                                                               Eigen::Vector4f(1, 1, itg->second.cluster_index, 0.5) );
              pubmarkers.markers.push_back(item);
          }
          itg++;
      }
      mutex.unlock();
      ArrayIdPub.publish(pubmarkers);

   // markerArrayPub.publish(aei.planes_dis);
   // pub4.publish(aei.nomal_arrow);
  }


private:  
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::Publisher pub2;
  ros::Publisher pub3;
  ros::Publisher pub4;
  ros::Publisher pub5;
  ros::Subscriber sub2;
  ros::Subscriber sub3;
  ros::Publisher markerArrayPub;
  ros::Publisher ArrayIdPub;
  geometry_msgs::PoseArray posearray;
  pointcloud_common::occupied_grid3d grid3d;
  pointcloud_common::Feature_map feature_map;
  Eigen::Vector3d pose;
  std::vector<std::vector<int> > result_cluster;
  boost::mutex mutex;
  double timestamp_;
  int count1 = 0, count2 = 0;
  bool l_flag1_bool;

};//End of class SubscribeAndPublish  

int main(int argc, char **argv)  
{  
  //Initiate ROS  
  ros::init(argc, argv, "lidarrevicve");  
  ros::NodeHandle ns;
  ros::MultiThreadedSpinner s(2);

  SubscribeAndPublish test(ns, "../track_algo/params/lidar_perception.yaml");

  ros::Timer timer = ns.createTimer(ros::Duration(0.1), &SubscribeAndPublish::timecallback, &test);
  double anotherv = 0.0;
  ros::spin(s);
  return 0;  
}




