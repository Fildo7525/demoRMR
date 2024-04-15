#include "KalmanFilter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
						MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
	m_x = x_in;
	m_P = P_in;
	m_F = F_in;
	m_H = H_in;
	m_R = R_in;
	m_Q = Q_in;
}

void KalmanFilter::Predict()
{
	/**
	* predict the state
	*/
	m_x = m_F * m_x;
	MatrixXd Ft = m_F.transpose();
	m_P = m_F * m_P * Ft + m_Q;
}

void KalmanFilter::Update(const VectorXd &z)
{
	/**
	* update the state by using Kalman Filter equations
	*/
	VectorXd z_pred = m_H * m_x;
	VectorXd y = z - z_pred;
	MatrixXd Ht = m_H.transpose();
	MatrixXd S = m_H * m_P * Ht + m_R;
	MatrixXd Si = S.inverse();

	// Compute Kalman gain
	MatrixXd K = m_P * Ht * Si;

	// Update estimate
	m_x = m_x + K * y;
	long x_size = m_x.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	m_P = (I - K * m_H) * m_P;
}

void KalmanFilter::UpdateEKF(const VectorXd &z)
{
	/**
	* update the state by using Extended Kalman Filter equations
	*/
	float px = m_x(0);
	float py = m_x(1);
	float vx = m_x(2);
	float vy = m_x(3);

	// Map predicted state into measurement space
	double rho	 = sqrt(px * px + py * py);
	double phi	   = atan2(py, px);
	double rho_dot = (px * vx + py * vy) / std::max(rho, 0.0001);

	VectorXd z_pred(3);
	z_pred << rho, phi, rho_dot;

	VectorXd y = z - z_pred;

	// Normalize angle
	while (y(1) > M_PI) y(1) -= 2 * M_PI;
	while (y(1) < -M_PI) y(1) += 2 * M_PI;

	MatrixXd Ht = m_H.transpose();
	MatrixXd S = m_H * m_P * Ht + m_R;
	MatrixXd Si = S.inverse();

	// Compute Kalman gain
	MatrixXd K = m_P * Ht * Si;

	// Update estimate
	m_x = m_x + K * y;
	long x_size = m_x.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	m_P = (I - K * m_H) * m_P;
}

