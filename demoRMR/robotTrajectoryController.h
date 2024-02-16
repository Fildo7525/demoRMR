#ifndef ROBOTTRAJECTORYCONTROLLER_H
#define ROBOTTRAJECTORYCONTROLLER_H

#include "pidcontroller.h"
#include "qtimer.h"
#include "robot.h"
#include <QObject>
#include <QWidget>
#include <QThread>
#include <memory>

class RobotTrajectoryController
	: public QObject
{
	Q_OBJECT
public:
	RobotTrajectoryController(QObject *parrent, Robot *robot, double timerInterval = 100);

	void setTranslationSpeed(double velocity, bool stopPositionTimer = false, double accelerationRate = 50);
	void setRotationSpeed(double omega, bool stopPositionTimer = false, double accelerationRate = 0.1);

	void rotateRobotTo(double rotation);
	void moveForwardBy(double distance);

private:
	bool isNear(double currentVelocity);
	double distanceError();
	double rotationError();

public slots:
	void controlTrajectory();
	void stop();
	void control();
	void onTimeoutChangePosition();

private:
	Robot *m_robot;

	QThread m_speedControlThread;

	QTimer m_accelerationTimer;
	QTimer m_stoppingTimer;
	QTimer m_positionTimer;

	std::shared_ptr<PIDController> m_controller;

	bool isRotating;

	double m_forwardSpeed;
	double m_rotationSpeed;

	double m_targetVelocity;
	double m_accelerationRate;

	double m_targetPosition;
	double m_targetRotation;
};

#endif // ROBOTTRAJECTORYCONTROLLER_H
