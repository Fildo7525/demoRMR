#ifndef ROBOTTRAJECTORYCONTROLLER_H
#define ROBOTTRAJECTORYCONTROLLER_H

#include "qtimer.h"
#include "robot.h"
#include <QObject>
#include <QWidget>
#include <QThread>

class RobotTrajectoryController
	: public QObject
{
	Q_OBJECT
public:
	RobotTrajectoryController(QObject *parrent, Robot *robot, double timerInterval = 100);

	void setTranslationSpeed(double velocity, double accelerationRate = 50);
	void setRotationSpeed(double omega, double accelerationRate = 0.1);

	void rotateRobotTo(double rotation);
	void moveForwardBy(double distance);

private:
	bool isNear(double currentVelocity);

public slots:
	void controlTrajectory();
	void stop();
	void control();

private:
	Robot *m_robot;

	QThread m_speedControlThread;
	QTimer m_accelerationTimer;
	QTimer m_stoppingTimer;
	QTimer m_positionTimer;

	bool isRotating;

	double m_forwardSpeed;
	double m_rotationSpeed;

	double m_targetVelocity;
	double m_accelerationRate;

	double m_targetPosition;
	double m_targetRotation;
};

#endif // ROBOTTRAJECTORYCONTROLLER_H
