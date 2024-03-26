#ifndef MAINWINDOW_H
#define MAINWINDOW_H

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

#define TILE_SIZE 13.

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
    void obstacleAvoidanceTrajectoryHandle(LaserMeasurement laserData, double actual_X, double actual_Y, double actual_Fi);
    bool doISeeTheTarget(LaserMeasurement laserData, double angleToTarget, double distanceToTarget);
    void doFinalTransport();

public slots:
	void setUiValues(double robotX, double robotY, double robotFi);
	void timeout();
	void handlePath(QVector<QPointF> path);

private:
signals:
	void uiValuesChanged(double newrobotX, double newrobotY, double newrobotFi); ///toto nema telo
	void startGuiding();

	void moveForward(double speed);
	void changeRotation(double rotation);

	void requestPath(const QPoint &start, const QPoint &end);

public:
signals:
	void linResultsReady(double distance, double rotaiton, QVector<QPointF> points);
	void arcResultsReady(double distance, double rotaiton, QVector<QPointF> points);
	void lidarDataReady(LaserMeasurement laserData);

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
};

#endif // MAINWINDOW_H
