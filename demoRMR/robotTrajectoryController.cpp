#include "robotTrajectoryController.h"
#include "mainwindow.h"
#include "pidcontroller.h"
#include "qtimer.h"
#include <chrono>
#include <ios>
#include <memory>
#include <thread>

RobotTrajectoryController::RobotTrajectoryController(QObject *parrent, Robot *robot, double timerInterval)
	: QObject(parrent)
	, m_robot(robot)
	, m_speedControlThread(parrent)
	, m_accelerationTimer(parrent)
	, m_forwardSpeed(0)
	, m_rotationSpeed(0)
{
	m_accelerationTimer.setInterval(timerInterval/2);
	m_accelerationTimer.setSingleShot(false);

	m_positionTimer.setInterval(timerInterval);
	m_positionTimer.setSingleShot(false);

	m_stoppingTimer.setInterval(3'000);
	m_stoppingTimer.setSingleShot(true);

	connect(&m_stoppingTimer, &QTimer::timeout, this, &RobotTrajectoryController::stop);
	connect(&m_accelerationTimer, &QTimer::timeout, this, &RobotTrajectoryController::control);
	connect(&m_positionTimer, &QTimer::timeout, this, &RobotTrajectoryController::onTimeoutChangePosition);
}

void RobotTrajectoryController::setTranslationSpeed(double velocity, bool stopPositionTimer, double accelerationRate)
{
	m_rotationSpeed = 0;
	isRotating = false;
	m_stoppingTimer.stop();
	if (stopPositionTimer)
		m_positionTimer.stop();

	m_targetVelocity = velocity;
	m_accelerationRate = accelerationRate;

	if (velocity == 0) {
		m_forwardSpeed = 0;
		m_robot->setTranslationSpeed(0);
		return;
	}

	if (m_targetVelocity < m_forwardSpeed) {
		m_accelerationRate = - m_accelerationRate;
	}

	m_accelerationTimer.start();
}

void RobotTrajectoryController::setRotationSpeed(double omega, bool stopPositionTimer, double accelerationRate)
{
	m_forwardSpeed = 0;
	isRotating = true;
	m_stoppingTimer.stop();
	if (stopPositionTimer)
		m_positionTimer.stop();

	m_targetVelocity = omega;
	m_accelerationRate = accelerationRate;

	if (omega == 0) {
		m_rotationSpeed = 0;
		m_robot->setRotationSpeed(0);
		return;
	}

	if (m_targetVelocity < m_rotationSpeed) {
		m_accelerationRate = - m_accelerationRate;
	}

	m_accelerationTimer.start();
}

void RobotTrajectoryController::rotateRobotTo(double rotation)
{
	isRotating = true;
	m_accelerationTimer.stop();
	m_stoppingTimer.stop();

	m_controller = std::make_shared<PIDController>(1, 0, 0, rotation);
	std::cout << "Starting position timer" << std::endl;
	m_positionTimer.start();
}

void RobotTrajectoryController::moveForwardBy(double distance)
{
	isRotating = false;
	m_accelerationTimer.stop();
	m_stoppingTimer.stop();

	m_controller = std::make_shared<PIDController>(100, 5, 0, distance);
	std::cout << "Starting position timer" << std::endl;
	m_positionTimer.start();
}

bool RobotTrajectoryController::isNear(double currentVelocity)
{
	return std::abs(currentVelocity - m_targetVelocity) <= std::abs(m_accelerationRate/2.);
}

double RobotTrajectoryController::distanceError()
{
	MainWindow *win = qobject_cast<MainWindow*>(parent());
	auto [distance, rotation] = win->calculateTrajectory();
	return distance;
}

double RobotTrajectoryController::rotationError()
{
	MainWindow *win = qobject_cast<MainWindow*>(parent());
	return win->rotationError();
}

void RobotTrajectoryController::controlTrajectory()
{
	MainWindow *win = qobject_cast<MainWindow*>(parent());
	auto [distance, rotation] = win->calculateTrajectory();

	std::cout << "Moving to rotation " << rotation << std::endl;
	moveForwardBy(distance);
	//rotateRobotTo(rotation);
	std::cout << "Is position timer active: " << std::boolalpha << m_positionTimer.isActive() << std::noboolalpha << std::endl;
}

void RobotTrajectoryController::stop()
{
	m_forwardSpeed = 0;
	m_rotationSpeed = 0;

	m_accelerationTimer.stop();
	m_positionTimer.stop();
	m_stoppingTimer.stop();

	m_stoppingTimer.setInterval(3'000);
	m_robot->setTranslationSpeed(0);
}

void RobotTrajectoryController::control()
{
	if (isNear(m_forwardSpeed) || isNear(m_rotationSpeed)) {
		m_accelerationTimer.stop();
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

void RobotTrajectoryController::onTimeoutChangePosition()
{
	double error;

	if (isRotating) {
		error = rotationError();
	}
	else {
		error = distanceError();
	}

	if (std::abs(error) < 0.2) {
		stop();
		return;
	}

	double u = m_controller->computeFromError(error);
	std::cout << "Akcny zasah: " << u << std::endl;

	if (isRotating) {
		setRotationSpeed(u);
	}
	else {
		setTranslationSpeed(u);
	}
}
