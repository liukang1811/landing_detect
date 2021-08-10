#include "motion_state.h"

Motion::motion_state::motion_state()
{
    this->his_size = 15;
    Direction_Global = Vector3d(1, 0, 0);
}
Motion::motion_state::motion_state(Vector3d lon0lat1_origin, Vector3d direction, int history_size)
{
    this->his_size = history_size;
    this->lon0lat1_origin = lon0lat1_origin;
    this->direction_origin = direction;
    Direction_Global = Vector3d(1, 0, 0);
}
Vector3d Motion::motion_state::fromWGS84Toeceu(Vector3d position)
{
    Vector3d result;

    double a = 6378137;
    double b = 6356752.3142;
    double f = (a - b) / a;
    double e_sq = f * (2 - f);
    double lamb = position[1] / 180.0 * 3.14159265;
    double phi =  (position[0] / 180.0) * 3.14159265;
    double s = sin(lamb);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lamb);
    double cos_lambda = cos(lamb);
    double sin_phi = sin(phi);
    double cos_phi = cos(phi);

    double x = (position[2] + N) * cos_lambda * cos_phi;
    double y = (position[2] + N) * cos_lambda * sin_phi;
    double z = (position[2] + (1 - e_sq) * N) * sin_lambda;


    result[0] = x;
    result[1] = y;
    result[2] = z;

    return  result;
}
Vector3d Motion::motion_state::fromWGS84ToENU(Vector3d position)
{
    Vector3d enuresult;
    Vector3d startPos   = fromWGS84Toeceu(this->lon0lat1_origin);
    Vector3d endPos     = fromWGS84Toeceu(position);
    Vector3d diff = endPos - startPos;
    double clat = cos(position[1] / 180.0 * 3.14159265);
    double slat = sin(position[1] / 180.0 * 3.14159265);
    double clong = cos(position[0] / 180.0 * 3.14159265);
    double slong = sin(position[0] / 180.0 * 3.14159265);
    enuresult[0] = diff[1]*clong - diff[0]*slong;
    enuresult[1] = diff[2]*clat - slat*(diff[0]*clong + diff[1]*slong);
    enuresult[2] = clat*(diff[0]*clong + diff[1]*slong) + diff[2]*slat;
    return enuresult;
}
void Motion::motion_state::set_reference_state(Vector3d position, Vector3d direction)
{
    this->lon0lat1_origin = position;
    this->direction_origin = direction;
}
void Motion::motion_state::StateUpdate(state_t state)
{
    if (state.wgs84ORenu == WGS84vaild) WGS84_update(state);
    else ENU_update(state);
}
Vector3f Motion::motion_state::get_translation()
{
    return this->cur_ENUposition.cast<float>();

}
Vector3d Motion::motion_state::get_AbsoluteAzimuth()
{
    Eigen::Matrix3d R;
    Eigen::Vector3d v1(1,0,0);
    R = Eigen::Quaterniond::FromTwoVectors(v1 , this->cur_direction).toRotationMatrix();
    Vector3d angle = R.eulerAngles(0,1,2);
    return angle;
}
Matrix3f Motion::motion_state::get_rotation_matrix()
{
    Eigen::Matrix3d R;
    R = Eigen::Quaterniond::FromTwoVectors(this->direction_origin ,this->cur_direction).toRotationMatrix();
    return R.cast<float>();
}
void Motion::motion_state::WGS84_update(state_t state)
{
    this->cur_lon0lat1at2 = state.lon0lat1at2;
    this->cur_ENUposition = fromWGS84ToENU(this->cur_lon0lat1at2);
    this->cur_direction = state.direction;
}
void Motion::motion_state::ENU_update(state_t state)
{
    this->cur_direction = state.direction;
    this->cur_ENUposition = state.position;
}
Vector3d Motion::motion_state::fromAngleToDirection(Vector3d ENUangle, uint8_t aixs_aginst)
{
    Eigen::AngleAxisd rollAngle(AngleAxisd(ENUangle(2),Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(AngleAxisd(ENUangle(1),Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(AngleAxisd(ENUangle(0),Vector3d::UnitZ()));
    Eigen::AngleAxisd rotation_vector;
    Eigen::Vector3d v1;
    if (aixs_aginst == x_axis) v1 = Eigen::Vector3d(1,0,0);
    else if (aixs_aginst == y_axis) v1 = Eigen::Vector3d(0,1,0);
    else v1 = Eigen::Vector3d(0,0,1);
    rotation_vector=yawAngle*pitchAngle*rollAngle;
    Eigen::Vector3d rotated_v1 = rotation_vector.matrix() * v1;
    return rotated_v1;
}
Vector3d Motion::motion_state::fromDirectionToAngle(Vector3d ENUvector, uint8_t aixs_aginst)
{
    Eigen::Matrix3d R;
    Eigen::Vector3d v1;
    if (aixs_aginst == x_axis) v1 = Eigen::Vector3d(1,0,0);
    else if (aixs_aginst == y_axis) v1 = Eigen::Vector3d(0,1,0);
    else v1 = Eigen::Vector3d(0,0,1);
    R = Eigen::Quaterniond::FromTwoVectors(v1 , ENUvector).toRotationMatrix();
    Vector3d angle = R.eulerAngles(0,1,2);
    return angle;
}
Eigen::Quaterniond Motion::motion_state::fromDirectionToQuaternion(Vector3d ENUdirection, uint8_t aixs_aginst)
{
    Eigen::Vector3d v1;
    if (aixs_aginst == x_axis) v1 = Eigen::Vector3d(1,0,0);
    else if (aixs_aginst == y_axis) v1 = Eigen::Vector3d(0,1,0);
    else v1 = Eigen::Vector3d(0,0,1);
    return (Eigen::Quaterniond::FromTwoVectors(v1 , ENUdirection));
}
Vector3d Motion::motion_state::fromQuaternionToDirection(Eigen::Quaterniond ENUQuaterniond, uint8_t aixs_aginst)
{
   Eigen::Matrix3d r = ENUQuaterniond.toRotationMatrix();
   Eigen::Vector3d v1;
   if (aixs_aginst == x_axis) v1 = Eigen::Vector3d(1,0,0);
   else if (aixs_aginst == y_axis) v1 = Eigen::Vector3d(0,1,0);
   else v1 = Eigen::Vector3d(0,0,1);
   Eigen::Vector3d rotated_v1 = r * v1;
   return  rotated_v1;
}
Eigen::Quaterniond Motion::motion_state::fromAngleToQuaternion(Vector3d ENUangle, uint8_t aixs_aginst)// aginst (1, 0, 0)
{
    Eigen::Vector3d eulervector= fromAngleToDirection(ENUangle, aixs_aginst);
    Eigen::Quaterniond quaterniond = fromDirectionToQuaternion(eulervector, aixs_aginst);
    return  quaterniond;

//    return quaternion;
}
Vector3d Motion::motion_state::fromQuaternionToAngle(Eigen::Quaterniond ENUQuaterniond, uint8_t aixs_aginst)// aginst (1, 0, 0)
{
    Eigen::Vector3d eulervector= fromQuaternionToDirection(ENUQuaterniond, aixs_aginst);
    Eigen::Vector3d eulerangle = fromDirectionToAngle(eulervector, aixs_aginst);
//    Eigen::Quaterniond quaternion_(Eigen::Vector4d(s.orientation.x, s.orientation.y, s.orientation.z, s.orientation.w));
//    Eigen::Vector3d eulerAngle4 = quaternion_.matrix().eulerAngles(0,1,2);
//    Eigen::Vector3d angle(roll, pitch, yaw);
    return  eulerangle;
}


