#include "mainwindow.h"
#include "pidcontroller.h"
#include "qnamespace.h"
#include "robotTrajectoryController.h"
#include <QDebug>
#include <QTimer>
#include <fstream>
#include <memory>

RobotTrajectoryController::RobotTrajectoryController(Robot *robot, QObject *window, double timerInterval)
	: QObject()
	, m_robot(robot)
	, m_mainWindow(window)
	, m_accelerationTimer(this)
	, m_positionTimer(this)
	, m_stoppingTimer(this)
	, m_controller(nullptr)
	, m_rotationController(nullptr)
	, m_forwardSpeed(0)
	, m_rotationSpeed(0)
	, m_map(300, std::vector<bool>(300, false))
	, m_fileWriteCounter(0)
{
	m_accelerationTimer.setInterval(timerInterval);
	m_accelerationTimer.setSingleShot(false);

	m_positionTimer.setInterval(timerInterval*2);
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
	// std::cout << __FUNCTION__ << " velocity: " << velocity << std::endl;
	m_rotationSpeed = 0;
	m_movementType = MovementType::Forward;
	m_stoppingTimer.stop();
	if (stopPositionTimer) {
		m_positionTimer.stop();
	}

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

	if (stopPositionTimer) {
		m_positionTimer.stop();
	}

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

void RobotTrajectoryController::setArcSpeed(double velocity, double omega, bool stopPositionTimer, double accelerationRate, double omegaRate)
{
	m_movementType = MovementType::Arc;
	m_stoppingTimer.stop();

	if (stopPositionTimer) {
		m_positionTimer.stop();
	}

	m_targetVelocity = velocity;
	m_targetOmega = omega;
	m_accelerationRate = accelerationRate;
	m_omegaRate = omegaRate;

	if (m_targetVelocity < m_forwardSpeed) {
		m_accelerationRate = -m_accelerationRate;
	}

	if (m_targetOmega < m_rotationSpeed) {
		m_omegaRate = -m_omegaRate;
	}

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

	m_controller = std::make_shared<PIDController>(1000, 0, 0, distance);
	m_positionTimer.start();
}

void RobotTrajectoryController::moveByArcTo(double distance, double rotation)
{
	m_movementType = MovementType::Arc;
	m_stopped = false;

	m_accelerationTimer.stop();
	m_stoppingTimer.stop();

	m_controller = std::make_shared<PIDController>(100, 0, 0, distance);
	m_rotationController = std::make_shared<PIDController>(0.5, 0, 0, rotation);

	m_positionTimer.start();
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
	// std::cout << "Rotation error detection\n";
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
	m_robot->setTranslationSpeed(0);
	m_forwardSpeed = 0;
	m_rotationSpeed = 0;

	m_accelerationTimer.stop();
	m_positionTimer.stop();
	m_stoppingTimer.stop();

	// m_stoppingTimer.setInterval(3'000);
	m_stopped = true;
	m_movementType = MovementType::None;
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
		// qDebug() << "setting Robot rotation speed to " << m_rotationSpeed;
		m_robot->setRotationSpeed(m_rotationSpeed);
	}
	else if (m_movementType == MovementType::Forward) {
		m_forwardSpeed += m_accelerationRate;
		// qDebug() << "setting Robot forward speed to " << m_forwardSpeed;
		m_robot->setTranslationSpeed(m_forwardSpeed);
	}
	else if (m_movementType == MovementType::Arc) {
		m_forwardSpeed += m_accelerationRate;
		m_rotationSpeed += m_omegaRate;
		m_robot->setArcSpeed(m_forwardSpeed, m_rotationSpeed);
	}
}

void RobotTrajectoryController::on_positionTimerTimeout_changePosition()
{
	static double error, maxCorrection;

	if (m_movementType == MovementType::Rotation) {
		error = localRotationError();
		maxCorrection = 0.1;
	}
	else if (m_movementType == MovementType::Forward) {
		error = localDistanceError();
		auto rotError = localRotationError();
		maxCorrection = std::abs((std::sin(rotError) * error) * 1.1);
	}
	else if (m_movementType == MovementType::Arc) {
		error = localDistanceError();
		maxCorrection = 0.5;
	}

	// qDebug() << "Error: " << error << " maxCorrection: " << maxCorrection;
	if (std::abs(error) < maxCorrection) {
		// qDebug() << "Error is less than " << maxCorrection << " It's " << maxCorrection;

		on_stoppingTimerTimeout_stop();
		// qDebug() << m_points;
		if (m_movementType == MovementType::Rotation) {
			qDebug() << "Rotation finished, requesting movement";
			emit requestMovement(localDistanceError());
		}
		else if (m_movementType == MovementType::Forward && finalDistanceError() > 0.05) {
			if (m_points.size() > 1)
				m_points.removeFirst();

			emit requestRotation(localRotationError());
		}
		// qDebug() << "Final destination reached final distance with movement: " << movementTypeToString(m_movementType) << " error: " << finalDistanceError();
		return;
	}

	double u;
	if (m_movementType == MovementType::Rotation) {
		u = m_controller->computeFromError(error);
		// qDebug() << "Akcny zasah: " << u;
		setRotationSpeed(u);
	}
	else if (m_movementType == MovementType::Forward) {
		u = m_controller->computeFromError(error, true);
		// qDebug() << "Akcny zasah: " << u;
		setTranslationSpeed(u);
	}
	else if (m_movementType == MovementType::Arc) {
		u = m_controller->computeFromError(error, true);
		double o = m_rotationController->computeFromError(error);
		qDebug() << "Akcny zasah u: " << u << " o: " << o;
		setArcSpeed(u, o);
	}

	// qDebug() << "Akcny zasah: " << u;
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

void RobotTrajectoryController::handleArcResults(double distance, double rotation, QVector<QPointF> points)
{
	m_points = points;
	moveByArcTo(distance, rotation);
}

void RobotTrajectoryController::on_requestMovement_move(double distance)
{
	moveForwardBy(distance);
}

void RobotTrajectoryController::on_requestRotation_move(double rotation)
{
	rotateRobotTo(rotation);
}

void RobotTrajectoryController::on_lidarDataReady_map(LaserMeasurement laserData)
{
	MainWindow *win = qobject_cast<MainWindow *>(m_mainWindow);
	double robotAngle = win->m_fi - win->m_fiCorrection;
	double robotX = win->m_x;
	double robotY = win->m_y;

	if (m_movementType == MovementType::Rotation) {
		return;
	}

	if (m_fileWriteCounter % 5 == 0) {
		m_laserMapFile.open("laserData.txt", std::ios::out);
	}

	for (size_t i = 0; i < laserData.numberOfScans; i += 5) {
		double distance = laserData.Data[i].scanDistance / 60.;
		if (distance < m_robot->b / 2.) {
			// The laser did not reach the wall.
			continue;
		}

		double scanAngle = laserData.Data[i].scanAngle * TO_RADIANS;
		qDebug() << "Robot X: " << robotX << " Y: " << robotY << " Angle: " << robotAngle;
		double x = robotX / 60. + distance * std::cos(scanAngle + robotAngle);
		double y = robotY / 60. + distance * std::sin(scanAngle + robotAngle);

		x += m_map[0].size() / 2;
		y += m_map.size() / 2;

		// qDebug() << "x: " << (int) (x + int(m_map[0].size() / 2)) << " y: " << (int) (y + int(m_map.size() / 2));
		int mapX = static_cast<int>(x);
		int mapY = static_cast<int>(y);

		// Check if within map bounds
		if (mapX >= 0 && mapX < m_map[0].size() && mapY >= 0 && mapY < m_map.size()) {
			m_map[mapY][mapX] = true;
		}
	}

	if (m_fileWriteCounter % 5 == 0) {
		qDebug() << "Laser data ready";
		m_laserMapFile << m_map;
		m_laserMapFile.close();
	}

	m_fileWriteCounter++;
	qDebug() << "Counter: " << m_fileWriteCounter;
}

std::ostream &operator<<(std::ostream &os, const RobotTrajectoryController::Map &map)
{
	for (size_t i = 0; i < map.size(); i++) {
		for (size_t j = 0; j < map[i].size(); j++) {
			os << (map[i][j] ? '#' : ' ');
		}
		os << std::endl;
	}
	return os;
}

