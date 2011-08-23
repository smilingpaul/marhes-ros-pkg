#include "odom_ekf/eif.h"

Eif::Eif():y_k_k_1(STATES_), y_k_k(STATES_), Y_k_k_1(STATES_, STATES_),
           Y_k_k(STATES_, STATES_), dF(STATES_, STATES_), F(STATES_), 
           x_k_k(STATES_), x_k_k_1(STATES_), H_enc(STATES_, STATES_),
           H_gps(2, STATES_), Cnb(3,3)         
{
  y_k_k_1 = 0;
  y_k_k = 0;
  Y_k_k_1 = 0;
  Y_k_k = 0;
  dF = 0;
  F = 0;
  x_k_k = 0;
  x_k_k_1 = 0;
  H_enc = 0;
    
  for (int i = 1; i <= STATES_; i++)
  {
    for (int j = 1; j <= STATES_; j++)
    {
      if (i == j)
      {
        dF(i, j) = 1;
        H_enc(i, j) = 1;
        Y_k_k(i, j) = 1;
      }
    }
  }
  
  H_gps = 0;
  H_gps(1,1) = 1;
  H_gps(2,2) = 1;
  
  calcDCM(0, 0, 0);
  
  first_msg_ = true;
}

void Eif::Predict(sensor_msgs::Imu msg) //double ax, double wz, Matrix Q)
{
  /*dF(1, 3) = -sin(x_k_k(4));
  dF(2, 3) = cos(x_k_k(4));
  Y_k_k_1 = (dF * Y_k_k.inverse() * dF.transpose() + Q).inverse();
  F(1) = x_k_k(3) * cos(x_k_k(4));
  F(2) = x_k_k(3) * sin(x_k_k(4));
  F(3) = ax;
  F(4) = wz;
  y_k_k_1 = Y_k_k_1 * F;
  x_k_k_1 = Y_k_k_1.inverse() * y_k_k_1;
  */
  
  // Get the accelerations in the N-Frame
  ColumnVector fb(3), fn(3), g(3);
  fb(1) = msg.linear_acceleration.x;
  fb(2) = msg.linear_acceleration.y;
  fb(3) = msg.linear_acceleration.z;
  g(1) = 0; g(2) = 0; g(3) = -9.805;
  CalcDCM();
  
  double dt = 0;
  if (first_msg_)
  {
    dt = 0;
    first_msg_ = false;
  }
  else
    dt = (msg.header.stamp - last_msg_.header.stamp).toSec();

  fn = Cnb * fb - g;  
  //x_k_k_1(1) += x_k_k(4);
  //x//_k_k_1(2) += x_k_k(5);
  //x_k_k_1(3) += x_k_k(6);
  //x_k_k_1(4) += fn(1);
  //x_k_k_1(5) += fn(2);
  //x_k_k_1(6) += fn(3);
  x_k_k_1(7) += (msg.angular_velocity.y * sin(x_k_k(9)) + msg.angular_velocity.z * cos(x_k_k(9))) / cos(x_k_k(8)) * dt;
  x_k_k_1(8) += msg.angular_velocity.y * cos(x_k_k(9)) - msg.angular_velocity.z * sin(x_k_k(9)) * dt;
  x_k_k_1(9) += msg.angular_velocity.x + (msg.angular_velocity.y * sin(x_k_k(9)) + msg.angular_velocity.z * cos(x_k_k(9))) * tan(x_k_k(8)) * dt;
  
  x_k_k = x_k_k_1;        
}

void Eif::UpdateGPS(ColumnVector meas, Matrix R)
{
  ColumnVector z(2);
  
  z = x_k_k.sub(1, 2) - meas;
  Y_k_k = Y_k_k_1 + H_gps.transpose() * R.inverse() * H_gps;
  y_k_k = y_k_k_1 + H_gps.transpose() * R.inverse() * z;
  x_k_k = Y_k_k.inverse() * y_k_k;
}

void Eif::UpdateEncoder(ColumnVector meas, Matrix R)
{
  ColumnVector z(4);
  
  z = x_k_k - meas;
  Y_k_k = Y_k_k_1 + H_gps.transpose() * R.inverse() * H_gps;
  y_k_k = y_k_k_1 + H_gps.transpose() * R.inverse() * z;
  x_k_k = Y_k_k.inverse() * y_k_k;
}

void Eif::UpdateVicon(ColumnVector meas, Matrix R)
{
  ColumnVector z(2);
  
  z = x_k_k.sub(1, 2) - meas;
  Y_k_k = Y_k_k_1 + H_gps.transpose() * R.inverse() * H_gps;
  y_k_k = y_k_k_1 + H_gps.transpose() * R.inverse() * z;
  x_k_k = Y_k_k.inverse() * y_k_k;
}

ColumnVector Eif::GetEstimate()
{
  return x_k_k;
}

void Eif::CalcDCM()
{
  double cy = cos(x_k_k(7));
  double sy = sin(x_k_k(7));
  double cp = cos(x_k_k(8));
  double sp = sin(x_k_k(8));
  double cr = cos(x_k_k(9));
  double sr = sin(x_k_k(9));
  
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
