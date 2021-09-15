//Kalman filter implementation file

#include <iostream>
#include <stdexcept>

#include <kf.h>

//intialize kalmanfilter constructor with initalization list
KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& F,
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P)
  : F(F), H(H), Q(Q), R(R), P0(P),
    m(H.rows()), n(F.rows()), dt(dt), initialized(false),
    I(n, n), x_hat(n), x_hat_new(n)
{
    I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
}

void KalmanFilter::init() {
  x_hat.setZero();
  P = P0;
  t0 = 0;
  t = t0;
  initialized = true;
}

void KalmanFilter::update(const Eigen::VectorXd& y) {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");

  x_hat_new = F * x_hat;
  P = F*P*F.transpose() + Q;
  K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
  x_hat_new += K * (y - H*x_hat_new);
  P = (I - K*H)*P;
  x_hat = x_hat_new;

  t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) 
{
  this->F = F;
  this->dt = dt;
  update(y);
}