//
// Created by liuk on 2021/7/22.
//

#ifndef LIDAR_DETECTION_ROS_ROS_PUBLISH_COMMON_H
#define LIDAR_DETECTION_ROS_ROS_PUBLISH_COMMON_H
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
namespace Ros_receive_publish {
    struct range_info
    {
        float xmin;
        float ymin;
        float xmax;
        float ymax;
        float zmax;
        float zmin;
        range_info(){};
        range_info(float xmin, float xmax, float ymin, float ymax, float zmin, float zmax)
        : xmin(xmin), xmax(xmax), ymin(ymin),ymax(ymax), zmin(zmin), zmax(zmax){}
    };
    struct center_size
    {
        Eigen::Vector3f center;
        Eigen::Vector3f size;
        int number = 0;
    };
    class ros_publish_common {
    public:
        ros_publish_common(){;}
        static void pubcloud_fill(sensor_msgs::PointCloud2& pubcloud, string a1, int point_step, float* intput_cloud, int point_width)
        {
            pubcloud.fields.resize(4);
            pubcloud.fields[0].name = "x";
            pubcloud.fields[0].offset = 0;
            pubcloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
            pubcloud.fields[0].count = 1;
            pubcloud.fields[1].name = "y";
            pubcloud.fields[1].offset = 4;
            pubcloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
            pubcloud.fields[1].count = 1;
            pubcloud.fields[2].name = "z";
            pubcloud.fields[2].offset = 8;
            pubcloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
            pubcloud.fields[2].count = 1;
            pubcloud.fields[3].name = "intense";
            pubcloud.fields[3].offset = 12;
            pubcloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
            pubcloud.fields[3].count = 1;
            pubcloud.height = 1;//pubcloud.data
            pubcloud.point_step = point_step;
            pubcloud.row_step = point_width  * point_step;
            pubcloud.width = point_width;
            pubcloud.header.frame_id =a1 ;
            pubcloud.header.stamp = ros::Time::now();
            pubcloud.data.resize(pubcloud.row_step);
            memcpy(&pubcloud.data[0], intput_cloud, pubcloud.row_step);
        }
        static visualization_msgs::Marker CudeMarkerCreate(Eigen::Vector3f position, Eigen::Vector3f scale, int id , string frame_id
                                                          ,Eigen::Quaternionf quertion = Eigen::Quaternionf::Identity(),
                                                          Eigen::Vector4f color = Eigen::Vector4f(0.1, 1, 0.2, 1))
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id=frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = std::to_string(id);
            marker.id =  id ;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(0.1);
            marker.scale.x = scale[0];
            marker.scale.y = scale[1];
            marker.scale.z = scale[2];
            marker.color.r = color[0];
            marker.color.g = color[1];
            marker.color.b = color[2];
            marker.color.a = color[3];
            marker.frame_locked = false;

            geometry_msgs::Pose pose;
            pose.position.x =  position[0];
            pose.position.y =  position[1];
            pose.position.z =  position[2];
            pose.orientation.w = quertion.w();
            pose.orientation.x = quertion.x();
            pose.orientation.y = quertion.y();
            pose.orientation.z = quertion.z();
//            ostringstream str;
//            str<<" angle ";
//            marker.text=str.str();
            marker.pose=pose;
            return marker;
        }
        static visualization_msgs::Marker TEXTMarkerCreate(Eigen::Vector3f position, Eigen::Vector3f scale, int id , string frame_id,
                                                           std::vector<std::pair<string, float> > value,
                                                           Eigen::Vector4f color = Eigen::Vector4f(0.1, 1, 0.2, 1))
        {
            Eigen::Quaternionf quertion = Eigen::Quaternionf::Identity();
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id;
            marker.header.stamp = ros::Time::now();
            marker.ns = std::to_string(id);
            marker.id =  id ;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(0.1);
            marker.scale.x = scale[0];
            marker.scale.y = scale[1];
            marker.scale.z = scale[2];
            marker.color.r = color[0];
            marker.color.g = color[1];
            marker.color.b = color[2];
            marker.color.a = color[3];
            marker.frame_locked = false;

            geometry_msgs::Pose pose;
            pose.position.x =  position[0];
            pose.position.y =  position[1];
            pose.position.z =  position[2];
            pose.orientation.w = quertion.w();
            pose.orientation.x = quertion.x();
            pose.orientation.y = quertion.y();
            pose.orientation.z = quertion.z();
            ostringstream str;
            for(int i = 0; i < value.size(); i++)
            {
                str<<value[i].first;
                str<<value[i].second;
                str<< '\n';
            }
            marker.text=str.str();
            marker.pose=pose;
            return marker;
        }
        static vector<float> boundingbox2data(const vector<center_size> boxs, Eigen::Vector4f roi)
        {
            vector<float> datae;
            float size = 0.1;
            float probilty_t = 0.8;
            float xs[4] = {roi[0], roi[1], roi[1], roi[0]};
            float ys[4] = {roi[3], roi[3], roi[2], roi[2]};

            float dx = xs[1] - xs[0];
            float dy = ys[1] - ys[0];
            float distance = sqrt(dx*dx + dy*dy);
            int number = distance / 0.1;
            dx = xs[1] - xs[0];
            dy = ys[1] - ys[0];
            distance = sqrt(dx*dx + dy*dy);
            number = distance / size;
            for(int j= 0; j < number; j++)
            {
                float x = xs[0] + float(j) * (dx / number);
                datae.push_back(x);
                float y = ys[0] + float(j) * (dy / number);
                datae.push_back(y);
                float z = 0;
                datae.push_back(z);
                float intensity = 0;
                datae.push_back(intensity);
            }

            dx = xs[2] - xs[1];
            dy = ys[2] - ys[1];
            distance = sqrt(dx*dx + dy*dy);
            number = distance / size;
            for(int j= 0; j < number; j++)
            {
                float x = xs[1] + float(j) * (dx / number);
                datae.push_back(x);
                float y = ys[1] + float(j) * (dy / number);
                datae.push_back(y);
                float z = 0;
                datae.push_back(z);
                float intensity = 0;
                datae.push_back(intensity);
            }

            dx = xs[3] - xs[2];
            dy = ys[3] - ys[2];
            distance = sqrt(dx*dx + dy*dy);
            number = distance / size;
            for(int j= 0; j < number; j++)
            {
                float x = xs[2] + float(j) * (dx / number);
                datae.push_back(x);
                float y = ys[2] + float(j) * (dy / number);
                datae.push_back(y);
                float z = 0;
                datae.push_back(z);
                float intensity = 0;
                datae.push_back(intensity);
            }

            dx = xs[0] - xs[3];
            dy = ys[0] - ys[3];
            distance = sqrt(dx*dx + dy*dy);
            number = distance / size;
            for(int j= 0; j < number; j++)
            {
                float x = xs[3] + float(j) * (dx / number);
                datae.push_back(x);
                float y = ys[3] + float(j) * (dy / number);
                datae.push_back(y);
                float z = 0;
                datae.push_back(z);
                float intensity = 0;
                datae.push_back(intensity);
            }
            int classnumber = 0;
            ////////////////////////////////////////
            for (int i = 0; i < boxs.size(); i ++)
            {
                if (1 > probilty_t)
                {
                    classnumber = boxs[i].number;
                    float minx = boxs[i].center[0] - boxs[i].size[0] / 2.0;
                    float maxx = boxs[i].center[0] + boxs[i].size[0] / 2.0;
                    float miny = boxs[i].center[1] - boxs[i].size[1] / 2.0;
                    float maxy = boxs[i].center[1] + boxs[i].size[1] / 2.0;
                    float minz = boxs[i].center[2] - boxs[i].size[2] / 2.0;
                    float maxz = boxs[i].center[2] + boxs[i].size[2] / 2.0;
                    float xs[8] = {minx, minx, maxx, maxx, minx, minx, maxx, maxx};
                    float ys[8] = {miny, maxy, maxy, miny, miny, maxy, maxy, miny};
                    float zs[8] =  {minz, minz, minz, minz, maxz, maxz, maxz, maxz};
                    float x_center = boxs[i].center[0];
                    float y_center = boxs[i].center[1];
                    float z_center = boxs[i].center[2];
                    datae.push_back(x_center);
                    datae.push_back(y_center);
                    datae.push_back(z_center);
                    datae.push_back(0);
                    float dx = xs[1] - xs[0];
                    float dy = ys[1] - ys[0];
                    float distance = sqrt(dx*dx + dy*dy);
                    int number = distance / size;
                    //classnumber = classnumber + 1;
                    float x, y, z,intensity;
                    for(int j = 0; j <int(number); j++)
                    {
                        x = xs[0] + float(j) * (dx / number);
                        datae.push_back(x);
                        y = ys[0] + float(j) * (dy / number);
                        datae.push_back(y);
                        z = minz;
                        datae.push_back(z);
                        intensity = classnumber;
                        datae.push_back(intensity);
                    }

                    for(int j = 0; j <int(number); j++)
                    {
                        x = xs[0] + float(j) * (dx / number);
                        datae.push_back(x);
                        y = ys[0] + float(j) * (dy / number);
                        datae.push_back(y);
                        z = maxz;
                        datae.push_back(z);
                        intensity = classnumber;
                        datae.push_back(intensity);
                    }


                    dx = xs[2] - xs[1];
                    dy = ys[2] - ys[1];
                    distance = sqrt(dx*dx + dy*dy);
                    number = distance / size;
                    for(int j = 0; j <int(number); j++)
                    {
                        x = xs[1] + float(j) * (dx / number);
                        datae.push_back(x);
                        y = ys[1] + float(j) * (dy / number);
                        datae.push_back(y);
                        z = minz;
                        datae.push_back(z);
                        intensity = classnumber;
                        datae.push_back(intensity);
                    }

                    for(int j = 0; j <int(number); j++)
                    {
                        x = xs[1] + float(j) * (dx / number);
                        datae.push_back(x);
                        y = ys[1] + float(j) * (dy / number);
                        datae.push_back(y);
                        z = maxz;
                        datae.push_back(z);
                        intensity = classnumber;
                        datae.push_back(intensity);
                    }

                    dx = xs[3] - xs[2];
                    dy = ys[3] - ys[2];
                    distance = sqrt(dx*dx + dy*dy);
                    number = distance / size;
                    for(int j = 0; j <int(number); j++)
                    {
                        x = xs[2] + float(j) * (dx / number);
                        datae.push_back(x);
                        y = ys[2] + float(j) * (dy / number);
                        datae.push_back(y);
                        z = minz;
                        datae.push_back(z);
                        intensity = classnumber;
                        datae.push_back(intensity);
                    }

                    for(int j = 0; j <int(number); j++)
                    {
                        x = xs[2] + float(j) * (dx / number);
                        datae.push_back(x);
                        y = ys[2] + float(j) * (dy / number);
                        datae.push_back(y);
                        z = maxz;
                        datae.push_back(z);
                        intensity = classnumber;
                        datae.push_back(intensity);
                    }

                    dx = xs[0] - xs[3];
                    dy = ys[0] - ys[3];
                    distance = sqrt(dx*dx + dy*dy);
                    number = distance / size;
                    for(int j = 0; j <int(number); j++)
                    {
                        x = xs[3] + float(j) * (dx / number);
                        datae.push_back(x);
                        y = ys[3] + float(j) * (dy / number);
                        datae.push_back(y);
                        z = minz;
                        datae.push_back(z);
                        intensity = classnumber;
                        datae.push_back(intensity);
                    }

                    for(int j = 0; j <int(number); j++)
                    {
                        x = xs[3] + float(j) * (dx / number);
                        datae.push_back(x);
                        y = ys[3] + float(j) * (dy / number);
                        datae.push_back(y);
                        z = minz + boxs[i].size[2];
                        datae.push_back(z);
                        intensity = classnumber;
                        datae.push_back(intensity);
                    }
                    for(int l =0 ; l < 4; l++)
                    {
                        number = boxs[i].size[2] / size;
                        for(int j = 0; j < number; j ++)
                        {
                            x = xs[l];
                            datae.push_back(x);
                            y = ys[l];
                            datae.push_back(y);
                            z = minz + j * (boxs[i].size[2] / number);
                            datae.push_back(z);
                            intensity = classnumber;
                            datae.push_back(intensity);
                        }
                    }
                }


            }
            return  datae;
        }
    };
    class ros_reveice_common {

    public:

        ros_reveice_common()
        {
            ;
        }
        static pointcloud_common::pointcloud_t  Rospointcloud2pointcloud_t(const sensor_msgs::PointCloud2 s, uint8_t pointstype, range_info range) // 1 rspoints 2 livox 3 velodyne 4....
        {
            pointcloud_common::pointcloud_t points;
            if (pointstype == 1)
            {
                int k= 0 ;
                points.points.clear();
                float *dataptr;
                dataptr = (float*)(&s.data[0]);
                int unt = s.point_step/4;
                float xMin, yMin, xMax, yMax;
                int k_number = 0;
                for(int i=0; i<unt* s.width * s.height;i += unt)
                {
                    if ( (dataptr[i+1] >= range.ymin) && (dataptr[i+1] < range.ymax)&&
                         (dataptr[i] >= range.xmin) && (dataptr[i] < range.xmax)&&
                         (dataptr[i+2] >= range.zmin) && (dataptr[i+2] < range.zmax))
                    {
                        bool flag = ((dataptr[i+1] >= -0.5) && (dataptr[i+1] < 0.5)&&
                                    (dataptr[i] >=  - 0.5) && (dataptr[i] <  0.5)&&
                                    (dataptr[i+2] >=  - 0.5) && (dataptr[i+2] < 0.5));
                        if (!flag)
                        {
                            k_number++;
                            pointcloud_common::point_type singlepoint(dataptr[i], dataptr[i + 1], dataptr[i + 2], dataptr[i + 4]);
                            points.points.push_back(singlepoint);
                        }
                    }
                }
            }
            points.width = points.points.size();
            return points;
        }

    };
}


#endif //LIDAR_DETECTION_ROS_ROS_PUBLISH_COMMON_H
