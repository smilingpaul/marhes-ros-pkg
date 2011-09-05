#include "odom_ekf/eif.h"

Eif::Eif():y_k_k_1(STATES_), y_k_k(STATES_), Y_k_k_1(STATES_, STATES_),
           Y_k_k(STATES_, STATES_), F(STATES_, STATES_), 
           x_k_k(STATES_), x_k_k_1(STATES_), H_enc(STATES_, STATES_),
           H_gps(3, STATES_), Qc(STATES_, STATES_), G(STATES_, STATES_),
           Q(STATES_, STATES_), Y(STATES_, STATES_), y(STATES_)
{
  y_k_k_1 = 0;
  y_k_k = 0;
  Y_k_k_1 = 0;
  Y_k_k = 0;
  y = 0.05;
  Y = 0;
  F = 0;
  F(1, 4) = 1; F(2, 5) = 1; F(3, 6) = 1;
  x_k_k = 0;
  x_k_k_1 = 0;
  H_enc = 0;
  Qc = 0;
  Qc(1, 1) = 0.05; Qc(2, 2) = 0.05; Qc(3, 3) = 0.05;
  Qc(4, 4) = 0.01; Qc(5, 5) = 0.01; Qc(6, 6) = 0.01;
  Qc(7, 7) = 0.005; Qc(8, 8) = 0.005; Qc(9, 9) = 0.005;
  G = 0;
  G(1, 1) = 1; G(2, 2) = 1; G(3, 3) = 1;
  Q = 0;
    
  for (int i = 1; i <= STATES_; i++)
  {
    for (int j = 1; j <= STATES_; j++)
    {
      if (i == j)
      {
        H_enc(i, j) = 1;
        Y_k_k(i, j) = 1;
        Y(i, j) = 0.1;
      }
    }
  }
  
  H_gps = 0;
  H_gps(1,1) = 1;
  H_gps(2,2) = 1;
  H_gps(3,3) = 1;
  
  predict_ = false;
}

void Eif::Predict(ColumnVector fn, Matrix Cnb, double dt)
{
  if (predict_)
  {
    // Update F
    F(4, 8) = -fn(3); F(4, 9) = fn(2);
    F(5, 7) = fn(3); F(5, 9) = -fn(1);
    F(6, 7) = -fn(2); F(6, 8) = fn(1);
    //ROS_INFO("FN: %f, %f, %f", fn(1), fn(2), fn(3));
   
    // Update G
    G(4, 4) = Cnb(1, 1); G(4, 5) = Cnb(1, 2); G(4, 6) = Cnb(1, 3);
    G(5, 4) = Cnb(2, 1); G(5, 5) = Cnb(2, 2); G(5, 6) = Cnb(2, 3);
    G(6, 4) = Cnb(3, 1); G(6, 5) = Cnb(3, 2); G(6, 6) = Cnb(3, 3);
    G(7, 7) = -Cnb(1, 1); G(7, 8) = -Cnb(1, 2); G(7, 9) = -Cnb(1, 3);
    G(8, 7) = -Cnb(2, 1); G(8, 8) = -Cnb(2, 2); G(8, 9) = -Cnb(2, 3);
    G(9, 7) = -Cnb(3, 1); G(9, 8) = -Cnb(3, 2); G(9, 9) = -Cnb(3, 3); 
    
    // Update Q
    Q = F * G * Qc * G.transpose() * F.transpose() + G * Qc * G.transpose();
    Q = Q * 0.5 * dt;
    
    // Predict Y
    Y = (F * Y.inverse() * F.transpose() + Q).inverse();
    
    // Predict y
    y = Y * F * Y.inverse() * y;
  }
  
  predict_ = true;
}

void Eif::UpdateGPS(ColumnVector ins, ColumnVector meas, Matrix R)
{
  ColumnVector z(3);
  z = ins - meas;
  Y = Y + H_gps.transpose() * R.inverse() * H_gps;
  y = y + H_gps.transpose() * R.inverse() * z;
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

ColumnVector Eif::GetState()
{
  return Y.inverse() * y;
}
