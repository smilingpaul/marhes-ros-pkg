#include "odom_ekf/ins.h"

// state: px, py, pz, vx, vy, vz, yaw, pitch, roll
Ins::Ins():state_(NUM_STATES_), Cnb(3, 3)
{
  state_ = 0;
  imu_rxed_ = false;
  CalcDCM(state(7), state(8), state(9));
}

void Ins::Update(sensor_msgs::Imu msg)
{
  if (imu_rxed_)
  {
    // Get elapsed time to do integration
    double dt = (msg.header.stamp - imu_msg_last_.header.stamp).toSec();
  
    // Get the accelerations
    ColumnVector fb(3), fn(3), g(3), angle_change(3);
    fb(1) = msg.linear_acceleration.x;
    fb(2) = msg.linear_acceleration.y;
    fb(3) = msg.linear_acceleration.z;
    g(1) = 0; g(2) = 0; g(3) = 9.80665;
    
    // Update Cnb
    angle_change(1) = msg.angular_acceleration.x * dt;
    angle_change(2) = msg.angular_acceleration.y * dt;
    angle_change(3) = msg.angular_acceleration.z * dt;
    double angle_change_mag = std.sqrt(angle_change(1) * angle_change(1) + 
                                       angle_change(2) * angle_change(2) + 
                                       angle_change(3) * angle_change(3));
    double alpha = sin(angle_change_mag) / angle_change_mag;
    double beta = (1 - cos(angle_change_mag)) / (angle_change_mag * angle_change_mag);
    Matrix angle_skew(3, 3);
    angle_skew(1, 1) = 0; angle_skew(1, 2) = -angle_change(3); angle_skew(1, 3) = angle_change(2);
    angle_skew(2, 1) = angle_change(3); angle_skew(2, 2) = 0; angle_skew(2, 3) = -angle_change(1);
    angle_skew(3, 1) = -angle_change(2); angle_skew(3, 2) = angle_change(1); angle_skew(3, 3) = 0;    
    Matrix I(3, 3);
    I = 0; I(1, 1) = 1; I(2, 2) = 1; I(3, 3) = 1;
    Cnb = Cnb * (I + alpha * angle_skew + beta * angle_skew * angle_skew);
    
    // Get the accelerations in the N frame
    fn = Cnb * fb - g;
    
  }
  
  imu_rxed_ = true;
  imu_msg_last_ = msg;
}

void Eif::CalcDCM(double yaw, double pitch, double roll)
{
  double cy = cos(yaw);
  double sy = sin(yaw);
  double cp = cos(pitch);
  double sp = sin(pitch);
  double cr = cos(roll);
  double sr = sin(roll);
  
  Cnb(1,1) = cp * cy;
  Cnb(1,2) = -cr * sy + sr * sp * cy;
  Cnb(1,3) = sr * sy + cr * sp * cy;
  Cnb(2,1) = cp * sy;
  Cnb(2,2) = cr * cy + sr * sp * sy;
  Cnb(2,3) = -sr * cy + cr * sp * sr;
  Cnb(3,1) = -sp;
  Cnb(3,2) = sr * cp;
  Cnb(3,3) = cr * cp;  
}
