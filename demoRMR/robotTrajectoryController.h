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

class RobotTrajectoryController : public QObject
{
	Q_OBJECT

	enum class MovementType {
		Forward,
		Rotation,
		Arc
	};

public:
	RobotTrajectoryController(Robot *robot, QObject *window, double timerInterval = 100);

	void setTranslationSpeed(double velocity, bool stopPositionTimer = false, double accelerationRate = 50);
	void setRotationSpeed(double omega, bool stopPositionTimer = false, double accelerationRate = 0.1);
	void setArcSpeed(double velocity, double omega, bool stopPositionTimer = false, double accelerationRate = 50, double omegaRate = 0.1);

	void rotateRobotTo(double rotation);
	void moveForwardBy(double distance);
	void moveByArcTo(double distance, double rotation);

private:
	bool isNear(double currentVelocity);
	double finalDistanceError();
	double localDistanceError();
	double finalRotationError();
	double localRotationError();

public slots:
	void on_stoppingTimerTimeout_stop();
	void on_accelerationTimerTimeout_control();
	void on_positionTimerTimeout_changePosition();
	void on_arcTimerTimeout_changePosition();

	void onMoveForwardMove(double speed);
	void onChangeRotationRotate(double speed);
	void handleLinResults(double distance, double rotation, QVector<QPointF> points);
	void handleArcResults(double distance, double rotation);

	void on_requestMovement_move(double distance);
	void on_requestRotation_move(double rotation);

private:
signals:
	void requestMovement(double distance);
	void requestRotation(double rotation);

private:
	Robot *m_robot;
	QObject *m_mainWindow;

	QTimer m_accelerationTimer;
	QTimer m_positionTimer;
	QTimer m_arcTimer;
	QTimer m_stoppingTimer;

	std::shared_ptr<PIDController> m_controller;
	std::shared_ptr<PIDController> m_rotationController;

	QVector<QPointF> m_points;

	MovementType m_movementType;
	bool m_stopped;

	double m_forwardSpeed;
	double m_rotationSpeed;

	double m_targetVelocity;
	double m_accelerationRate;
	double m_omegaRate;

	double m_targetPosition;
	double m_targetOmega;
};

#endif // ROBOTTRAJECTORYCONTROLLER_H
