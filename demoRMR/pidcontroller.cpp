#include "pidcontroller.h"

PIDController::PIDController(double p, double i, double d, double target)
	: m_kp(p)
	, m_ki(i)
	, m_kd(d)
	, m_target(target)
	, m_integral(0)
	, m_prev_error(0)
{
}

double PIDController::compute(double current)
{
	double error = m_target - current;
	return computeFromError(error);
}

double PIDController::computeFromError(double error)
{
	m_integral += error;
	double derivative = error - m_prev_error;
	m_prev_error = error;

	return m_kp * error + m_ki * m_integral + m_kd * derivative;
}
