#pragma once

#include <QObject>
#include <QPointF>

#include "rplidar.h"
#include "robotTrajectoryController.h"

#define MAX_OBSTACLE_CORNERS 10

class MainWindow;

class ObstacleAvoider
	: public QObject
{
	Q_OBJECT;

	struct obstacleCorner {
		QPointF cornerPos;
		double firstPathLen;
		double secondPathLen;
		double totalPathLen;
		bool direction; // true-right, false-left
		// Add other members as needed
	};

public:
	ObstacleAvoider(QObject *parent = 0);

	void on_requestAvoidance_handle(double xTarget, double yTarget);
	void on_handleTrajectory_handle(const LaserMeasurement &laserData, double actual_X, double actual_Y, double actual_Fi);

public: signals:
	void requestTrajectory(double x, double y, RobotTrajectoryController::MovementType type);

private:
	double computeDistance(double x1, double y1, double x2, double y2);
	double computeAngle(double x1, double y1, double x2, double y2, double acutal_Fi);
	QPointF computeTargetPosition(double actual_X, double actual_Y, double angleToTarget_deg, double distanceToTarget, bool dir);

	void onLiveAvoidObstaclesButton_clicked(bool clicked);
	void obstacleAvoidanceTrajectoryInit(double X_target, double Y_target);
	void obstacleAvoidanceTrajectoryHandle(const LaserMeasurement &laserData, double actual_X, double actual_Y, double actual_Fi);
	bool doISeeTheTarget(const LaserMeasurement &laserData, double angleToTarget, double distanceToTarget);
	void doFinalTransport();
	void analyseCorners(const LaserMeasurement& laserData, double actual_X, double actual_Y);


private:
	bool m_isInAutoMode;
	double autoModeTarget_X;
	double autoModeTarget_Y;
	double autoModeInit_X;
	double autoModeInit_Y;
	bool finalTransportStarted;
	LaserMeasurement laserDataDiff;
	obstacleCorner obstacleCorners[MAX_OBSTACLE_CORNERS];
	int cornersAvailable;
	bool checkCorners;
	MainWindow *m_mainWindow;
};
