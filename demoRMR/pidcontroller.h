#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <QObject>

// PID Controller class
class PIDController
	: public QObject
{
	Q_OBJECT
private:
	double m_kp; // Proportional gain
	double m_ki; // Integral gain
	double m_kd; // Derivative gain
	double m_target; // Target position
	double m_integral; // Integral term
	double m_prev_error; // Previous error

public:
	PIDController(double p, double i, double d, double target = 0);
	double compute(double current);
	void setTarget(double newTarget) { m_target = newTarget; }
	double target() const { return m_target; }
};

#endif // PIDCONTROLLER_H
