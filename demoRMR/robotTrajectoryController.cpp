#include "robotTrajectoryController.h"
#include "mainwindow.h"
#include "qtimer.h"

RobotTrajectoryController::RobotTrajectoryController(QObject *parrent, Robot *robot, double timerInterval)
	: QObject(parrent)
	, m_robot(robot)
	, m_speedControlThread(parrent)
	, m_timer(parrent)
	, m_forwardSpeed(0)
	, m_rotationSpeed(0)
{
	m_timer.setInterval(timerInterval);
	m_timer.setSingleShot(false);

	m_stoppingTimer.setInterval(3'000);
	m_stoppingTimer.setSingleShot(true);

	connect(&m_stoppingTimer, &QTimer::timeout, this, &RobotTrajectoryController::stop);
	connect(&m_timer, &QTimer::timeout, this, &RobotTrajectoryController::control);
}

void RobotTrajectoryController::setTranslationSpeed(double velocity, double accelerationRate)
{
	m_rotationSpeed = 0;
	isRotating = false;
	m_stoppingTimer.stop();

	m_targetVelocity = velocity;
	m_accelerationRate = accelerationRate;

	if (velocity == 0) {
		m_robot->setTranslationSpeed(0);
		return;
	}

	if (m_targetVelocity < m_forwardSpeed) {
		m_accelerationRate = - m_accelerationRate;
	}

	m_timer.start();
}

void RobotTrajectoryController::setRotationSpeed(double omega, double accelerationRate)
{
	m_forwardSpeed = 0;
	isRotating = true;
	m_stoppingTimer.stop();

	m_targetVelocity = omega;
	m_accelerationRate = accelerationRate;

	if (m_targetVelocity < m_rotationSpeed) {
		m_accelerationRate = - m_accelerationRate;
	}

	m_timer.start();
}

bool RobotTrajectoryController::isNear(double currentVelocity)
{
	return std::abs(currentVelocity - m_targetVelocity) <= std::abs(m_accelerationRate/2.);
}

void RobotTrajectoryController::controlTrajectory()
{
	MainWindow *win = qobject_cast<MainWindow*>(parent());
	auto directions = win->calculateTrajectory();

	setTranslationSpeed(500);
}

void RobotTrajectoryController::stop()
{
	m_forwardSpeed = 0;
	m_rotationSpeed = 0;
	m_robot->setTranslationSpeed(0);
}

void RobotTrajectoryController::control()
{
	if (isNear(m_forwardSpeed) || isNear(m_rotationSpeed)) {
		m_timer.stop();
		m_stoppingTimer.start();
		return;
	}

	if (isRotating) {
		m_rotationSpeed += m_accelerationRate;
		m_robot->setRotationSpeed(m_rotationSpeed);
	}
	else {
		m_forwardSpeed += m_accelerationRate;
		m_robot->setTranslationSpeed(m_forwardSpeed);
	}
}
