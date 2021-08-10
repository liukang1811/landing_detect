#ifndef MOTION_STATE_H
#define MOTION_STATE_H
#include "iostream"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "vector"
#include "math.h"
#define WGS84vaild 1
#define enuvaild 2
using namespace Eigen;
namespace Motion {
    const int x_axis = 1;
    const int y_axis = 2;
    const int z_axis = 3;
    typedef struct
    {
      Vector3d lon0lat1at2;
      Vector3d position; // 0:x 1:y 2:z
      Vector3d velcity;
      Vector3d acceleration;
      Vector3d direction;
      Vector3d angular; // 0:by x(roll) 1:by y(pitch) 2:by z(yaw)
      Vector3d angular_velocity; // 0:by x 1:by y 2:by z(yaw)
      uint8_t wgs84ORenu;
    } state_t; //ENU coord
    class motion_state
    {
    public:
        motion_state(Vector3d lon0lat1_origin, Vector3d direction, int history_size);
        motion_state();
        void StateUpdate(state_t state);
        void set_reference_state(Vector3d position, Vector3d direction);
        Vector3f get_translation();// aginst reference_state
        Matrix3f get_rotation_matrix();// aginst reference_state
        Matrix4f get_transformation();// aginst reference_state
        Vector3d get_AbsoluteAzimuth();////aginst north (1, 0, 0) return direction vector
        state_t current_state;
        std::vector<state_t> histort_state;
        static Vector3d fromAngleToDirection(Vector3d ENUangle, uint8_t aixs_aginst);
        static Vector3d fromDirectionToAngle(Vector3d ENUvector, uint8_t aixs_aginst);
        static Eigen::Quaterniond fromDirectionToQuaternion(Vector3d ENUdirection, uint8_t aixs_aginst);
        static Vector3d fromQuaternionToDirection(Eigen::Quaterniond ENUQuaterniond, uint8_t aixs_aginst);
        static Eigen::Quaterniond fromAngleToQuaternion(Vector3d ENUangle, uint8_t aixs_aginst);
        static Vector3d fromQuaternionToAngle(Eigen::Quaterniond ENUQuaterniond, uint8_t aixs_aginst);

    private:
        int his_size;
        Vector3d lon0lat1_origin;
        Vector3d direction_origin;
        Vector3d cur_lon0lat1at2;
        Vector3d cur_ENUposition;
        Vector3d cur_direction;
        Vector3d Direction_Global;
        Vector3d fromWGS84ToENU(Vector3d position);
        Vector3d fromWGS84Toeceu(Vector3d position);
        void WGS84_update(state_t state);
        void ENU_update(state_t state);
    };


}

//// about ENU coordinate /////////
//  north 0y
//  east 0x
// 0 rad 0x anti-clockwise

#endif // MOTION_STATE_H
