/***********************************************************
 *                                                         *
 * Error-state covariance for DLIO's state (odom/geo/estimator: iekf | iekf_output)
 *                                                         *
 * The nominal state stays DLIO's own (position, velocity, attitude, gyro and accelerometer bias); this class
 * carries the 15x15 covariance of its error, ordered
 *   [ dtheta (3, attitude, body frame, q (+) exp(dtheta)) | dp (3, world) | dv (3, world) | dbg (3) | dba (3) ]
 * It is propagated on every IMU sample from the sensor's noise densities (Sola, "Quaternion kinematics for the
 * error-state Kalman filter", 2017). Two ways to bring a pose measurement in: conditionOnPose() for a
 * registration that already fused the filter's prior inside its solver (mode iekf), updatePose() for the plain
 * Kalman update from a pose innovation (mode iekf_output).
 *                                                         *
 ***********************************************************/

#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace dlio {

class ErrorStateFilter {
public:
  using Mat15 = Eigen::Matrix<double, 15, 15>;
  using Mat6 = Eigen::Matrix<double, 6, 6>;
  using Vec15 = Eigen::Matrix<double, 15, 1>;
  using Vec6 = Eigen::Matrix<double, 6, 1>;

  // Noise densities: gyro rad/s/sqrt(Hz), accel m/s^2/sqrt(Hz), bias random walks rad/s^2/sqrt(Hz), m/s^3/sqrt(Hz)
  struct Noise {
    double gyro {2e-3};
    double accel {2e-2};
    double gyro_bias {1e-5};
    double accel_bias {1e-4};
  };

  ErrorStateFilter() { P_.setIdentity(); }

  void reset(double att_std, double pos_std, double vel_std, double gyro_bias_std, double accel_bias_std) {
    P_.setZero();
    P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * att_std * att_std;
    P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * pos_std * pos_std;
    P_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * vel_std * vel_std;
    P_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * gyro_bias_std * gyro_bias_std;
    P_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * accel_bias_std * accel_bias_std;
  }

  Noise& noise() { return noise_; }
  const Mat15& P() const { return P_; }

  // Keep the pose block from collapsing below the floors (applied after each conditioning)
  void floorPose(const Vec6& floor_variance) {
    for (int i = 0; i < 6; ++i) {
      P_(i, i) = std::max(P_(i, i), floor_variance(i));
    }
  }

  static Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return m;
  }

  // One IMU step of the covariance. accel and gyro are the bias-corrected body measurements the nominal state was
  // propagated with, R the body-to-world rotation of the nominal attitude before the step.
  void propagate(double dt, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, const Eigen::Matrix3d& R) {
    if (!(dt > 0.0) || dt > 0.5) {
      return;
    }
    Mat15 F = Mat15::Identity();
    F.block<3, 3>(0, 0) -= skew(gyro) * dt;                  // attitude error rotates with the body rate
    F.block<3, 3>(0, 9) = -Eigen::Matrix3d::Identity() * dt;  // ... and integrates the gyro bias error
    F.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;   // position error integrates the velocity error
    F.block<3, 3>(6, 0) = -R * skew(accel) * dt;             // velocity error from the attitude error (specific force)
    F.block<3, 3>(6, 12) = -R * dt;                           // ... and from the accelerometer bias error
    Mat15 Q = Mat15::Zero();
    Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (noise_.gyro * noise_.gyro * dt);
    Q.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * (noise_.accel * noise_.accel * dt);
    Q.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (noise_.gyro_bias * noise_.gyro_bias * dt);
    Q.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (noise_.accel_bias * noise_.accel_bias * dt);
    P_ = F * P_ * F.transpose() + Q;
    P_ = (0.5 * (P_ + P_.transpose())).eval();
  }

  // Covariance of the pose block (dtheta, dp) and its inverse, the information the registration gets as its prior.
  Mat6 poseCovariance() const { return P_.block<6, 6>(0, 0); }

  Mat6 poseInformation(const Vec6& floor_variance) const {
    Mat6 c = poseCovariance();
    c.diagonal() += floor_variance;  // never more certain than the IMU model allows, never singular
    return c.ldlt().solve(Mat6::Identity());
  }

  // Condition every error state on the pose posterior the registration produced: dpose is the shift of the pose
  // mean (fused minus prior, [dtheta body; dp world]) and ppost its posterior covariance. Returns the correction of
  // all 15 error states (the pose part is dpose itself) and updates P accordingly (Gaussian conditioning).
  Vec15 conditionOnPose(const Vec6& dpose, const Mat6& ppost) {
    const Mat6 ppp = P_.block<6, 6>(0, 0);
    const Eigen::Matrix<double, 9, 6> pop = P_.block<9, 6>(6, 0);
    Mat6 reg = ppp;
    reg.diagonal().array() += 1e-12;
    const Eigen::Matrix<double, 9, 6> gain = pop * reg.ldlt().solve(Mat6::Identity());   // P_op P_pp^-1
    Vec15 dx = Vec15::Zero();
    dx.head<6>() = dpose;
    dx.tail<9>() = gain * dpose;
    Mat15 P = P_;
    P.block<6, 6>(0, 0) = ppost;
    P.block<9, 6>(6, 0) = gain * ppost;
    P.block<6, 9>(0, 6) = (gain * ppost).transpose();
    P.block<9, 9>(6, 6) = P_.block<9, 9>(6, 6) - gain * (ppp - ppost) * gain.transpose();
    P_ = (0.5 * (P + P.transpose())).eval();
    return dx;
  }

  // The plain Kalman update from a pose measurement: nu is the innovation (measured pose minus predicted pose,
  // [dtheta body; dp world]), R the measurement covariance; the measurement is the pose block itself (H = [I 0]).
  // Returns the correction of all 15 error states, updates P (Joseph form) and writes the normalised innovation
  // squared to nis (chi-square with 6 degrees of freedom: about 6 when the filter is consistent).
  Vec15 updatePose(const Vec6& nu, const Mat6& R, double& nis) {
    Mat6 S = P_.block<6, 6>(0, 0) + R;
    S = (0.5 * (S + S.transpose())).eval();
    const Mat6 S_inv = S.ldlt().solve(Mat6::Identity());
    nis = nu.dot(S_inv * nu);
    const Eigen::Matrix<double, 15, 6> K = P_.block<15, 6>(0, 0) * S_inv;   // P H^T S^-1
    const Vec15 dx = K * nu;
    Mat15 IKH = Mat15::Identity();
    IKH.block<15, 6>(0, 0) -= K;
    Mat15 P = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    P_ = (0.5 * (P + P.transpose())).eval();
    return dx;
  }

private:
  Mat15 P_;
  Noise noise_;
};

}  // namespace dlio
