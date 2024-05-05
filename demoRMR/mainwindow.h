#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "CKobuki.h"
#include "floodPlanner.h"
#include <QMainWindow>
#include <QTimer>
#ifdef _WIN32
#include <windows.h>
#endif
#include "qlineedit.h"
#include "qthread.h"
#include "robot.h"
#include "robotTrajectoryController.h"
#include "floodPlanner.h"
#include <QMutex>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <sys/types.h>
#include "lidarMapper.h"

#define TILE_SIZE 280. // 280 mm => 28cm
#define MAX_OBSTACLE_CORNERS 10
#define MAX_VISITED_CORNERS 25
#define CORNER_APPROACH_GAP 0.4
#define DISTANCE_PER_DT_STEADY_THRESHOLD 0.00003
#define CORNER_VISITED_TOLERANCE 0.4
#define LASER_DIFF_CORNER_THRESHOLD 0.5
#define MAX_CORNER_DISTANCE 3.0
#define COLISION_THRESHOLD 0.25

struct obstacleCorner
{
	QPointF cornerPos;
	QPointF neighbourPoints[3];
	QPointF cornerApproachPoint;
	QPointF cornerBypassPoint;
	double firstPathLen;
	double secondPathLen;
	double totalPathLen;
	double firstPathLenReal;
	double secondPathLenReal;
	double totalPathLenReal;
	bool direction; // true-right, false-left
	// Add other members as needed
};

namespace Ui {
class MainWindow;
}

///toto je trieda s oknom.. ktora sa spusti ked sa spusti aplikacia.. su tu vsetky gombiky a spustania...
class MainWindow : public QMainWindow
{
	Q_OBJECT

	friend class RobotTrajectoryController;
	friend class LidarMapper;

public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

	int processThisLidar(LaserMeasurement laserData);

	int processThisRobot(TKobukiData robotdata);

	int processThisCamera(cv::Mat cameraData);

	double finalRotationError();
	double localRotationError(QPair<double, double> point);
	QPair<double, double> calculateTrajectory();
	QPair<double, double> calculateTrajectoryTo(QPair<double, double> point);

private:
	void paintEvent(QPaintEvent *event); // Q_DECL_OVERRIDE;
	void calculateOdometry(const TKobukiData &robotdata);
	void _calculateTrajectory(RobotTrajectoryController::MovementType type);
	double computeDistance(double x1, double y1, double x2, double y2);
	double computeAngle(double x1, double y1, double x2, double y2, double acutal_Fi);
	QPointF computeTargetPosition(double actual_X, double actual_Y, double angleToTarget_deg, double distanceToTarget, bool dir);

private slots:
	void on_pushButton_8_clicked();

	void on_pushButton_9_clicked();

	void on_pushButton_2_clicked();

	void on_pushButton_3_clicked();

	void on_pushButton_6_clicked();

	void on_pushButton_5_clicked();

	void on_pushButton_4_clicked();

	void on_pushButton_clicked();

	void on_showMapButton_clicked();

	void on_startScanButton_clicked();

	void on_pathPlannerButton_clicked();

	bool updateTarget(QLineEdit *lineEdit, double &controller);
	void onLinSubmitButtonClicked(bool clicked);
	void onArcSubmitButtonClicked(bool clicked);
	void onLiveAvoidObstaclesButton_clicked(bool clicked);
	void obstacleAvoidanceTrajectoryInit(double X_target, double Y_target, double actual_X, double actual_Y, double actual_Fi);
	void obstacleAvoidanceTrajectoryHandle();
	bool doISeeTheTarget(LaserMeasurement laserData, double angleToTarget, double distanceToTarget);
	void doFinalTransport();
	QPointF analyseCorners(LaserMeasurement &laserData, const QPointF &actual, int idx, const QPointF &target);
	void analyseCorners(LaserMeasurement &laserData, double actual_X, double actual_Y);
	void findCornerWithShortestPath();
	bool wasCornerVisited(obstacleCorner thisCorner);
	double computeDistancePoints(QPointF A, QPointF B);
	void doCheckCorners();
	void onStartCheckCornersTimer();
	void checkColision();
	void obstacleAvoidanceAbort();

public slots:
	void setUiValues(double robotX, double robotY, double robotFi);
	void timeout();
	void handlePath(QVector<QPointF> path);
	void obstacleAvoidanceOnce(const QPointF &target);
	void obstacleAvoidanceOnceI(const QPointF &target, int idx);

private:
signals:
	void uiValuesChanged(double newrobotX, double newrobotY, double newrobotFi); ///toto nema telo
	void startGuiding();

	void moveForward(double speed);
	void changeRotation(double rotation);

	void requestPath(const QPointF &start, const QPointF &end);
	void startCheckCornersTimer();

public:
signals:
	void linResultsReady(double distance, double rotaiton, QVector<QPointF> points);
	void arcResultsReady(double distance, double rotaiton, QVector<QPointF> points);
	void lidarDataReady(LaserMeasurement laserData);
	void appendTransitionPoints(const QVector<QPointF> &points);

private:
	bool useCamera1;
	//  cv::VideoCapture cap;
	int actIndex;
	//	cv::Mat frame[3];
	cv::Mat frame[3];

	//--skuste tu nic nevymazat... pridavajte co chcete, ale pri odoberani by sa mohol stat nejaky drobny problem, co bude vyhadzovat chyby
	Ui::MainWindow *ui;
	LidarMapper *m_lidarMapper;
	QMetaObject::Connection m_connection;

	int updateLaserPicture;
	LaserMeasurement copyOfLaserData;
	std::string ipaddress;
	Robot robot;

	std::shared_ptr<RobotTrajectoryController> m_trajectoryController;
	std::shared_ptr<FloodPlanner> m_floodPlanner;

	TKobukiData robotdata;
	int datacounter;

	QTimer *timer;

	int lastLeftEncoder;
	int lastRightEncoder;
	std::atomic<double> m_fi;
	std::atomic<double> m_x;
	std::atomic<double> m_y;

	double m_xTarget;
	double m_yTarget;

	QThread *m_controllerThread;
	QThread *m_plannerThread;
	QThread *obstacleAvoidanceThread;
	QMutex m_mutex;

	double forwardspeed;  // mm/s
	double rotationspeed; // omega/s

	bool m_robotStartupLocation;
	double m_fiCorrection;

	bool m_isInAutoMode;
	double autoModeTarget_X;
	double autoModeTarget_Y;
	double autoModeInit_X;
	double autoModeInit_Y;
	bool finalTransportStarted;
	LaserMeasurement laserDataDiff;
	obstacleCorner obstacleCorners[MAX_OBSTACLE_CORNERS];
	obstacleCorner cornerWithShortestPath;
	obstacleCorner visitedCorners[MAX_VISITED_CORNERS];
	int visitedCornersCount;
	int cornersAvailable;
	bool checkCorners;
	double distancePerDT;
	QTimer *checkCornersTimer;
	bool timerStarted;
	bool isInitialCornerCheck;
	bool checkingColision;
	QPointF m_cornerOne;
	QPointF m_cornerTwo;
	double m_cornerOneAngle;
	double m_cornerTwoAngle;
};

#endif // MAINWINDOW_H
