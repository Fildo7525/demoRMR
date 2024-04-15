#pragma once

#include <eigen3/Eigen/Dense>

class KalmanFilter
{
public:
	/**
	 * Constructor
	 */
	KalmanFilter();

	/**
	 * Destructor
	 */
	virtual ~KalmanFilter();

	/**
	 * Init Initializes Kalman filter
	 * @param x_in Initial state
	 * @param P_in Initial state covariance
	 * @param F_in Transition matrix
	 * @param H_in Measurement matrix
	 * @param R_in Measurement covariance matrix
	 * @param Q_in Process covariance matrix
	 */
	void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
			  Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);

	/**
	 * Prediction Predicts the state and the state covariance
	 * using the process model
	 * @param delta_T Time between k and k+1 in s
	 */
	void Predict();

	/**
	 * Updates the state by using standard Kalman Filter equations
	 * @param z The measurement at k+1
	 */
	void Update(const Eigen::VectorXd &z);

	/**
	 * Updates the state by using Extended Kalman Filter equations
	 * @param z The measurement at k+1
	 */
	void UpdateEKF(const Eigen::VectorXd &z);

private:
	// state vector
	Eigen::VectorXd m_x;

	// state covariance matrix
	Eigen::MatrixXd m_P;

	// state transition matrix
	Eigen::MatrixXd m_F;

	// process covariance matrix
	Eigen::MatrixXd m_Q;

	// measurement matrix
	Eigen::MatrixXd m_H;

	// measurement covariance matrix
	Eigen::MatrixXd m_R;
};

