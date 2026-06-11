#include <iostream>
#include <vector>

using namespace std;

float RadToAngle(float rad)
{
    float angle = rad * 360 / (2 * 3.1415926);
    return angle;
}


vector<float> toEulerAngle(float x, float y, float z, float w)  // input: x, y, z, w  output: Euler vector: roll, pitch, yaw
{
    vector<float> output_rpy;
    float roll, pitch, yaw;

    // roll (x-axis rotation)
    float sinr_cosp = 2.0 * (w * x + y * z);
    float cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2.0 * (w * z + x * y);
    float cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
    // return yaw;

    output_rpy.push_back(roll);
    output_rpy.push_back(pitch);
    output_rpy.push_back(yaw);

    return output_rpy;
}

vector<float> toQuaernions(float roll, float pitch, float yaw)  // input: roll, pitch, yaw  output: x, y, z, w
{
    vector<float> output_qua;
    float x, y, z, w;
    x = sin(pitch / 2) * sin(yaw / 2) * cos(roll / 2) + cos(pitch / 2) * cos(yaw / 2) * sin(roll / 2);
    y = sin(pitch / 2) * cos(yaw / 2) * cos(roll / 2) + cos(pitch / 2) * sin(yaw / 2) * sin(roll / 2);
    z = cos(pitch / 2) * sin(yaw / 2) * cos(roll / 2) - sin(pitch / 2) * cos(yaw / 2) * sin(roll / 2);
    w = cos(pitch / 2) * cos(yaw / 2) * cos(roll / 2) - sin(pitch / 2) * sin(yaw / 2) * sin(roll / 2);

    output_qua.push_back(x);
    output_qua.push_back(y);
    output_qua.push_back(z);
    output_qua.push_back(w);

    return output_qua;
}
