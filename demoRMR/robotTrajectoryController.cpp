#include "robotTrajectoryController.h"
#include "mainwindow.h"
#include "pidcontroller.h"
#include "qdebug.h"
#include "qnamespace.h"
#include "qtimer.h"
#include <memory>
#include <thread>

RobotTrajectoryController::RobotTrajectoryController(Robot *robot, QObject *window, double timerInterval)
	: QObject()
	, m_robot(robot)
	, m_mainWindow(window)
	, m_accelerationTimer(this)
	, m_positionTimer(this)
	, m_stoppingTimer(this)
	, m_forwardSpeed(0)
	, m_rotationSpeed(0)
{
	m_accelerationTimer.setInterval(timerInterval / 2);
	m_accelerationTimer.setSingleShot(false);

	m_positionTimer.setInterval(timerInterval);
	m_positionTimer.setSingleShot(false);

	m_stoppingTimer.setInterval(3'000);
	m_stoppingTimer.setSingleShot(true);

	connect(&m_stoppingTimer, &QTimer::timeout, this, &RobotTrajectoryController::on_stoppingTimerTimeout_stop, Qt::QueuedConnection);
	connect(&m_accelerationTimer, &QTimer::timeout, this, &RobotTrajectoryController::on_accelerationTimerTimeout_control, Qt::QueuedConnection);
	connect(&m_positionTimer, &QTimer::timeout, this, &RobotTrajectoryController::on_positionTimerTimeout_changePosition, Qt::QueuedConnection);

	connect(this, &RobotTrajectoryController::requestMovement, this, &RobotTrajectoryController::on_requestMovement_move, Qt::QueuedConnection);
	connect(this, &RobotTrajectoryController::requestRotation, this, &RobotTrajectoryController::on_requestRotation_move, Qt::QueuedConnection);
}

void RobotTrajectoryController::setTranslationSpeed(double velocity, bool stopPositionTimer, double accelerationRate)
{
	std::cout << __FUNCTION__ << " " << std::this_thread::get_id() << std::endl;
	m_rotationSpeed = 0;
	m_movementType = MovementType::Forward;
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
		m_accelerationRate = -m_accelerationRate;
	}

	m_accelerationTimer.start();
}

void RobotTrajectoryController::setRotationSpeed(double omega, bool stopPositionTimer, double accelerationRate)
{
	// std::cout << "setting rotation Speed\n";
	m_forwardSpeed = 0;
	m_movementType = MovementType::Rotation;
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
		m_accelerationRate = -m_accelerationRate;
	}

	//std::cout << "Rotation Speed was set\n";
	m_accelerationTimer.start();
}

void RobotTrajectoryController::rotateRobotTo(double rotation)
{
	m_movementType = MovementType::Rotation;
	m_stopped = false;

	m_accelerationTimer.stop();
	m_stoppingTimer.stop();

	m_controller = std::make_shared<PIDController>(1, 0, 0, rotation);
	m_positionTimer.start();
}

void RobotTrajectoryController::moveForwardBy(double distance)
{
	m_movementType = MovementType::Forward;
	m_stopped = false;

	m_accelerationTimer.stop();
	m_stoppingTimer.stop();

	m_controller = std::make_shared<PIDController>(100, 5, 0, distance);
	std::cout << "Starting position timer" << std::endl;
	m_positionTimer.start();
}

void RobotTrajectoryController::moveByArcTo(QPointF point)
{
}

bool RobotTrajectoryController::isNear(double currentVelocity)
{
	return std::abs(currentVelocity - m_targetVelocity) <= std::abs(m_accelerationRate / 2.);
}

double RobotTrajectoryController::finalDistanceError()
{
	MainWindow *win = qobject_cast<MainWindow *>(m_mainWindow);
	auto [distance, rotation] = win->calculateTrajectory();
	return distance;
}

double RobotTrajectoryController::localDistanceError()
{
	QPointF point = m_points.first();

	MainWindow *win = qobject_cast<MainWindow *>(m_mainWindow);
	auto [distance, rotation] = win->calculateTrajectoryTo({ point.x(), point.y() });

	return distance;
}

double RobotTrajectoryController::finalRotationError()
{
	MainWindow *win = qobject_cast<MainWindow *>(m_mainWindow);
	std::cout << "Rotation error detection\n";
	return win->finalRotationError();
}

double RobotTrajectoryController::localRotationError()
{
	QPointF point = m_points.first();

	MainWindow *win = qobject_cast<MainWindow *>(m_mainWindow);
	return win->localRotationError({ point.x(), point.y() });
}

void RobotTrajectoryController::on_stoppingTimerTimeout_stop()
{
	m_forwardSpeed = 0;
	m_rotationSpeed = 0;

	m_accelerationTimer.stop();
	m_positionTimer.stop();
	m_stoppingTimer.stop();

	m_stoppingTimer.setInterval(3'000);
	m_robot->setTranslationSpeed(0);
	m_stopped = true;
}

void RobotTrajectoryController::on_accelerationTimerTimeout_control()
{
	if (isNear(m_forwardSpeed) || isNear(m_rotationSpeed)) {
		m_accelerationTimer.stop();
		m_stoppingTimer.start();
		return;
	}

	if (m_movementType == MovementType::Rotation) {
		m_rotationSpeed += m_accelerationRate;
		std::cout << "setting Robot rotation speed to " << m_rotationSpeed << std::endl;
		m_robot->setRotationSpeed(m_rotationSpeed);
	}
	else if (m_movementType == MovementType::Forward) {
		m_forwardSpeed += m_accelerationRate;
		m_robot->setTranslationSpeed(m_forwardSpeed);
	}
}

void RobotTrajectoryController::on_positionTimerTimeout_changePosition()
{
	double error;

	if (m_movementType == MovementType::Rotation) {
		error = localRotationError();
	}
	else if (m_movementType == MovementType::Forward) {
		error = localDistanceError();
	}

	std::cout << "Error: " << error << "\n";
	if (std::abs(error) < 0.1) {
		qDebug() << "Error is less than 0.1. It's " << error;

		on_stoppingTimerTimeout_stop();
		qDebug() << m_points;
		if (m_movementType == MovementType::Rotation) {
			emit requestMovement(localDistanceError());
		}
		else if (m_movementType == MovementType::Forward && finalDistanceError() > 0.1) {
			if (m_points.size() > 1)
				m_points.removeFirst();
			qDebug() << m_points;
			emit requestRotation(localRotationError());
		}
		return;
	}

	double u = m_controller->computeFromError(error);
	std::cout << "Akcny zasah: " << u << std::endl;

	if (m_movementType == MovementType::Rotation) {
		setRotationSpeed(u);
	}
	else if (m_movementType == MovementType::Forward) {
		setTranslationSpeed(u);
	}
	else {
		qDebug() << "Arc movement type";
	}
}

void RobotTrajectoryController::onMoveForwardMove(double speed)
{
	setTranslationSpeed(speed, true);
}

void RobotTrajectoryController::onChangeRotationRotate(double speed)
{
	setRotationSpeed(speed, true);
}

void RobotTrajectoryController::handleLinResults(double distance, double rotation, QVector<QPointF> points)
{
	m_points = points;
	rotateRobotTo(rotation);
}

void RobotTrajectoryController::handleArcResults(double distance, double rotation)
{
	
}

void RobotTrajectoryController::on_requestMovement_move(double distance)
{
	moveForwardBy(distance);
}

void RobotTrajectoryController::on_requestRotation_move(double rotation)
{
	rotateRobotTo(rotation);
}
