#ifndef ARROWTORVIZ_H
#define ARROWTORVIZ_H
#include <math.h>
#include <thread>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <algorithm>
#include <sensor_msgs/PointField.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Eigen>
#include <ros/time.h>
using namespace std;
class ArrowToRviz
{
   public:
    geometry_msgs::Quaternion quaternion;
    geometry_msgs::Point position;
    geometry_msgs::Pose pose;
    ArrowToRviz(Eigen::Vector3d nomals,  Eigen::Vector3d position)// positon 0 :x 1: y 2: z
    {
        if(nomals.rows() != 3)  cout<<"\033[31m the size of normal is not 3 \033[0m" << endl;
        else
        {
            if ((nomals[0] == 0)&&(nomals[1] == 0))
            {
              this->position.x = position[0];
              this->position.y = position[1];
              this->position.z = position[2];
              quaternion.x = 0;
              quaternion.y = 0;
              quaternion.z = 0;
              quaternion.w = 1;
              pose.orientation = quaternion;
              pose.position = this->position;

            }
            else
            {
              geometry_msgs::Quaternion fa2 =  vector2Quaternion_100(nomals);
              this->position.x = position[0];
              this->position.y = position[1];
              this->position.z = position[2];
              quaternion.x = fa2.x;
              quaternion.y = fa2.y;
              quaternion.z = fa2.z;
              quaternion.w = fa2.w;
              pose.orientation = quaternion;
              pose.position = this->position;
            }

        }
    }
    ArrowToRviz(float angle, Eigen::Vector3d position)// angle:0- 2pi
    {
        geometry_msgs::Quaternion fa2 =  vector2Quaternion_100( angle2orienation(angle));
        this->position.x = position[0];
        this->position.y = position[1];
        this->position.z = position[2];
        quaternion.x = fa2.x;
        quaternion.y = fa2.y;
        quaternion.z = fa2.z;
        quaternion.w = fa2.w;
        pose.orientation = quaternion;
        pose.position = this->position;
    }
    static Eigen::Vector3f rotationMatrixToEulerAngles_eigen(Eigen::Matrix3d &R)
    {

        float sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0) );


        bool singular = sy < 1e-6; // If


        float x, y, z;
        if (!singular) {
            x = atan2(R(2,1) , R(2,2));
            y = atan2(-R(2,0), sy);
            z = atan2(R(1,0), R(0,0));
        } else {
            x = atan2(-R(1,2), R(1,1));
            y = atan2(-R(2,0), sy);
            z = 0;
        }
        return Eigen::Vector3f(x, y, z);
    }
    static Eigen::Vector3d angle2orienation(float angle)
    {
       angle = fix_angle(angle);
       Eigen::Vector3d te;
       if (angle < 3.14159265 / 2.0)
       {
         te[0] = 1;
         te[1] = tan(angle);
         te[2] = 0;
       }
       else if (angle < 3.14159265)
       {
         te[0] = -1;
         te[1] = -tan(angle);
         te[2] = 0;
       }
       else if (angle < 3.14159265 * 3 / 2)
       {
         te[0] = -1;
         te[1] = -tan(angle);
         te[2] = 0;
       }
       else
       {
         te[0] = 1;
         te[1] = tan(angle);
         te[2] = 0;
       }
       return te;
    }
    static float fix_angle(float anglecentor)
    {
        float precise_ang;
        if(anglecentor > 0)
        {
            int fei = floor(anglecentor / (3.14159265 * 2.0));
            anglecentor = anglecentor - fei * (3.14159265 * 2.0);
        }
        else
        {
            int fei = floor(-anglecentor / (3.14159265 * 2.0));
            anglecentor = anglecentor + fei * (3.14159265 * 2.0);
        }
        precise_ang = (anglecentor > 0) ? anglecentor:( 3.14159265 * 2 + anglecentor);
        return precise_ang;
    }
   protected:
    geometry_msgs::Quaternion vector2Quaternion_100(Eigen::Vector3d vec)
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d v1{1,0,0};
        R = Eigen::Quaterniond::FromTwoVectors(v1, vec);
        Eigen::Vector3f vec2 = rotationMatrixToEulerAngles_eigen(R);
        geometry_msgs::Quaternion rf = tf::createQuaternionMsgFromRollPitchYaw(vec2[0], vec2[1], vec2[2]);
        return rf;
    }

};
#endif // ARROWTORVIZ_H
