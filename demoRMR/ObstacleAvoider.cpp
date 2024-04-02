#include "ObstacleAvoider.h"

#include <iostream>

#include <QDebug>
#include <qdebug.h>

#include "mainwindow.h"
#include "robotTrajectoryController.h"

ObstacleAvoider::ObstacleAvoider(QObject *parent)
	: QObject(nullptr)
	, m_isInAutoMode(false)
	, finalTransportStarted(false)
	, cornersAvailable(0)
	, checkCorners(false)
	, m_mainWindow(qobject_cast<MainWindow*>(parent))
{

}

void ObstacleAvoider::on_requestAvoidance_handle(double xTarget, double yTarget)
{
	qDebug() << "Requesting avoidance to: " << xTarget << ", " << yTarget;
	obstacleAvoidanceTrajectoryInit(xTarget, yTarget);
}

void ObstacleAvoider::on_handleTrajectory_handle(const LaserMeasurement &laserData, double actual_X, double actual_Y, double actual_Fi)
{
	if (m_isInAutoMode) {
		qDebug() << "Handling trajectory";
		obstacleAvoidanceTrajectoryHandle(laserData, actual_X, actual_Y, actual_Fi);
	}
}

void ObstacleAvoider::obstacleAvoidanceTrajectoryInit(double X_target, double Y_target)
{
	autoModeTarget_X = X_target;
	autoModeTarget_Y = Y_target;
	m_isInAutoMode = true;
	std::cout << "To do: " << autoModeTarget_X << " " << autoModeTarget_Y << std::endl;
	finalTransportStarted = false;
	checkCorners = true;
}

void ObstacleAvoider::obstacleAvoidanceTrajectoryHandle(const LaserMeasurement &laserData, double actual_X, double actual_Y, double actual_Fi)
{
//	std::cout << "Input values for computeDistance:" << std::endl;
//	std::cout << "actual_X: " << actual_X << std::endl;
//	std::cout << "actual_Y: " << actual_Y << std::endl;
//	std::cout << "autoModeTarget_X: " << autoModeTarget_X << std::endl;
//	std::cout << "autoModeTarget_Y: " << autoModeTarget_Y << std::endl;
	double distanceToTarget = computeDistance(actual_X,actual_Y,autoModeTarget_X,autoModeTarget_Y);

//	std::cout << "\nInput values for computeAngle:" << std::endl;
//	std::cout << "actual_X: " << actual_X << std::endl;
//	std::cout << "actual_Y: " << actual_Y << std::endl;
//	std::cout << "autoModeTarget_X: " << autoModeTarget_X << std::endl;
//	std::cout << "autoModeTarget_Y: " << autoModeTarget_Y << std::endl;
//	std::cout << "actual_Fi: " << actual_Fi << std::endl;
	double angleToTarget =  computeAngle(actual_X,actual_Y,autoModeTarget_X,autoModeTarget_Y, actual_Fi);

//	std::cout << "Distance to target: " << distanceToTarget << std::endl;
//	std::cout << "Angle to target: " << angleToTarget << std::endl;

	if (doISeeTheTarget(laserData,angleToTarget,distanceToTarget)) {
		if(!finalTransportStarted) {
			std::cout << "target visible at: " << angleToTarget << std::endl;
			finalTransportStarted = true;
			doFinalTransport();
		}
	}
	else {
		if(checkCorners) {
			checkCorners = false;
			analyseCorners(laserData, actual_X, actual_Y);
			if(cornersAvailable > 0) {
				auto x = obstacleCorners[0].cornerPos.x();
				auto y = obstacleCorners[0].cornerPos.y();
				qDebug() << "Requesting trajectory to: " << x << ", " << y << " with type: Arc";
				emit requestTrajectory(x, y, RobotTrajectoryController::MovementType::Arc);
			}
		}
	}
}

void ObstacleAvoider::analyseCorners(const LaserMeasurement& laserData, double actual_X, double actual_Y)
{
	cornersAvailable = 0;
	// Compute differentiation
	laserDataDiff.numberOfScans = laserData.numberOfScans - 1;
	for (size_t i = 0; i < laserData.numberOfScans - 1; ++i) {
		laserDataDiff.Data[i].scanDistance = laserData.Data[i+1].scanDistance - laserData.Data[i].scanDistance;
		laserDataDiff.Data[i].scanAngle = laserData.Data[i].scanAngle;

		if(abs(laserDataDiff.Data[i].scanDistance/1000.0) > 0.5) {

			obstacleCorner thisObstacleCorner;
			thisObstacleCorner.direction = laserDataDiff.Data[i].scanDistance > 0;

			if(laserData.Data[i].scanDistance < laserData.Data[i+1].scanDistance )
				thisObstacleCorner.cornerPos = computeTargetPosition(actual_X, actual_Y, laserData.Data[i].scanAngle, laserData.Data[i].scanDistance/1000.0, thisObstacleCorner.direction);
			else
				thisObstacleCorner.cornerPos = computeTargetPosition(actual_X, actual_Y, laserData.Data[i+1].scanAngle, laserData.Data[i+1].scanDistance/1000.0, thisObstacleCorner.direction);

			thisObstacleCorner.firstPathLen = computeDistance(actual_X, actual_Y, thisObstacleCorner.cornerPos.x(), thisObstacleCorner.cornerPos.y());
			thisObstacleCorner.secondPathLen = computeDistance(thisObstacleCorner.cornerPos.x(), thisObstacleCorner.cornerPos.y(), autoModeTarget_X,autoModeTarget_Y);
			thisObstacleCorner.totalPathLen =  thisObstacleCorner.firstPathLen + thisObstacleCorner.secondPathLen;

			if(0) {
				std::cout << "Corner at angle: " << laserDataDiff.Data[i].scanAngle;
				std::cout << "at pos: (" << thisObstacleCorner.cornerPos.x() << ", " << thisObstacleCorner.cornerPos.y() << ")";
				std::cout << "at dir: " << thisObstacleCorner.direction;
				std::cout << "first path: " << thisObstacleCorner.firstPathLen;
				std::cout << "second path: " <<  thisObstacleCorner.secondPathLen;
				std::cout << "sum: " <<   thisObstacleCorner.totalPathLen << std::endl;
			}

			obstacleCorners[cornersAvailable] = thisObstacleCorner;

			cornersAvailable++;
		}
	}
	std::cout << cornersAvailable << std::endl;

}

QPointF ObstacleAvoider::computeTargetPosition(double actual_X, double actual_Y, double angleToTarget_deg, double distanceToTarget, bool dir)
{
	// Convert the angle to radians
	double angleRad = angleToTarget_deg * M_PI / 180.0;
	angleRad = angleRad - (m_mainWindow->m_x - M_PI/2.0);

	// Compute the target position using trigonometry
	double target_X = actual_X + distanceToTarget * sin(angleRad);
	double target_Y = actual_Y + distanceToTarget * cos(angleRad);

	if(0) {
		std::cout << " myX:" << actual_X;
		std::cout << " myY:" << actual_Y;
		std::cout << " angle:" << angleToTarget_deg;
		std::cout << " dist:" << distanceToTarget;
		std::cout << " dir:" << dir;
		std::cout << " target_X:" << target_X;
		std::cout << " target_Y:" << target_Y << std::endl;
	}

	return QPointF(target_X, target_Y);
}

void ObstacleAvoider::doFinalTransport()
{
	std::cout << "Final transport started" << std::endl;
	emit requestTrajectory(autoModeTarget_X, autoModeTarget_Y, RobotTrajectoryController::MovementType::Arc);
	m_isInAutoMode = false;
}

bool ObstacleAvoider::doISeeTheTarget(const LaserMeasurement &laserData, double angleToTarget, double distanceToTarget)
{
	for(int i = 0; i<laserData.numberOfScans; i++) {
		if(laserData.Data[i].scanAngle < angleToTarget+0.1 && laserData.Data[i].scanAngle > angleToTarget-0.1) {
			if((laserData.Data[i].scanDistance/1000.0) > distanceToTarget) {
				return true;
			}
		}
	}
	return false;
}

double ObstacleAvoider::computeDistance(double x1, double y1, double x2, double y2)
{
	double deltaX = x2 - x1;
	double deltaY = y2 - y1;
	return sqrt(deltaX * deltaX + deltaY * deltaY);
}

double  ObstacleAvoider::computeAngle(double x1, double y1, double x2, double y2, double actual_Fi)
{
	double deltaY = y2 - y1;
	double deltaX = x2 - x1;

	// Compute the angle in radians using atan2
	double angle_rad = atan2(deltaY, deltaX);

	// Convert radians to degrees
	double angle_deg = angle_rad * 180.0 / PI;

	angle_deg = angle_deg + (-1)*(actual_Fi * 180.0 / PI);

	// Ensure angle is in the range [0, 360)
	if (angle_deg < 0)
		angle_deg += 360.0;

	return angle_deg;
}

