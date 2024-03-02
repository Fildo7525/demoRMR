#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#ifdef _WIN32
#include <windows.h>
#endif
#include "pidcontroller.h"
#include "qlineedit.h"
#include "qthread.h"
#include "robot.h"
#include "robotTrajectoryController.h"
#include <QJoysticks.h>
#include <QMutex>
#include <chrono>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <vector>
//#include "ckobuki.h"
//#include "ckobuki.h"
//#include "rplidar.h"
//#include "rplidar.h"
//#include<arpa/inet.h>
//#include<arpa/inet.h>
//#include<sys/socket.h>
//#include<sys/socket.h>
//#include<unistd.h>
//#include<unistd.h>

namespace Ui {
class MainWindow;
}

///toto je trieda s oknom.. ktora sa spusti ked sa spusti aplikacia.. su tu vsetky gombiky a spustania...
class MainWindow : public QMainWindow
{
	Q_OBJECT

	friend class RobotTrajectoryController;

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
	void _calculateTrajectory();

private slots:
	void on_pushButton_9_clicked();

	void on_pushButton_2_clicked();

	void on_pushButton_3_clicked();

	void on_pushButton_6_clicked();

	void on_pushButton_5_clicked();

	void on_pushButton_4_clicked();

	void on_pushButton_clicked();
	bool updateTarget(QLineEdit *lineEdit, double &controller);
	void onLinSubmitButtonClicked(bool clicked);
	void onArcSubmitButtonClicked(bool clicked);

public slots:
	void setUiValues(double robotX, double robotY, double robotFi);
	void timeout();

private:
signals:
	void uiValuesChanged(double newrobotX, double newrobotY, double newrobotFi); ///toto nema telo
	void startGuiding();

	void moveForward(double speed);
	void changeRotation(double rotation);

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
	int updateLaserPicture;
	LaserMeasurement copyOfLaserData;
	std::string ipaddress;
	Robot robot;

	RobotTrajectoryController *m_trajectoryController;

	TKobukiData robotdata;
	int datacounter;

	QTimer *timer;

	QJoysticks *instance;

	int lastLeftEncoder;
	int lastRightEncoder;
	std::atomic<double> m_fi;
	std::atomic<double> m_x;
	std::atomic<double> m_y;

	double m_xTarget;
	double m_yTarget;

	QThread *m_trajectoryThread;
	QThread *m_controllerThread;
	QMutex m_mutex;

	double forwardspeed;  // mm/s
	double rotationspeed; // omega/s

	bool m_robotStartupLocation;
	double m_fiCorrection;
};

#endif // MAINWINDOW_H
