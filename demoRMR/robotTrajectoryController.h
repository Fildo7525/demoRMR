#ifndef ROBOTTRAJECTORYCONTROLLER_H
#define ROBOTTRAJECTORYCONTROLLER_H

#include "pidcontroller.h"
#include "qtimer.h"
#include "robot.h"
#include <QObject>
#include <QWidget>
#include <QThread>
#include <condition_variable>
#include <memory>

class RobotTrajectoryController
	: public QObject
{
	Q_OBJECT
public:
	RobotTrajectoryController(Robot *robot, QObject *window, double timerInterval = 100);

	void setTranslationSpeed(double velocity, bool stopPositionTimer = false, double accelerationRate = 50);
	void setRotationSpeed(double omega, bool stopPositionTimer = false, double accelerationRate = 0.1);

	void rotateRobotTo(double rotation);
	void moveForwardBy(double distance);

private:
	bool isNear(double currentVelocity);
	double finalDistanceError();
	double localDistanceError();
	double finalRotationError();
	double localRotationError();

private: signals:
	void requestMovement(double distance);
	void requestRotation(double rotation);

public slots:
	void stop();
	void control();
	void onTimeoutChangePosition();

	void onMoveForwardMove(double speed);
	void onChangeRotationRotate(double speed);
	void handleResults(double distance, double rotation, QVector<QPointF> points);

	void onRequestMovementMove(double distance);
	void onRequestRotationMove(double rotation);

private:
	Robot *m_robot;
	QObject *m_mainWindow;

	QTimer m_accelerationTimer;
	QTimer m_positionTimer;
	QTimer m_stoppingTimer;

	std::shared_ptr<PIDController> m_controller;
	QVector<QPointF> m_points;

	bool isRotating;
	bool m_stopped;

	double m_forwardSpeed;
	double m_rotationSpeed;

	double m_targetVelocity;
	double m_accelerationRate;

	double m_targetPosition;
	double m_targetRotation;
};

#endif // ROBOTTRAJECTORYCONTROLLER_H
