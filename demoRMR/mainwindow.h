#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "pidcontroller.h"
#include "qlineedit.h"
#include "qthread.h"
#include "robotTrajectoryController.h"
#include <QMainWindow>
#include <QTimer>
#include <QMutex>
#ifdef _WIN32
#include<windows.h>
#endif
#include<iostream>
//#include<arpa/inet.h>
//#include<unistd.h>
//#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
//#include "ckobuki.h"
//#include "rplidar.h"
#include <chrono>

#include "robot.h"

using namespace std::chrono;

namespace Ui {
class MainWindow;
}

/// toto je trieda s oknom.. ktora sa spusti ked sa spusti aplikacia.. su tu vsetky gombiky a spustania...
class MainWindow : public QMainWindow
{
	Q_OBJECT
public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

	int processThisLidar(LaserMeasurement laserData);

	int processThisRobot(TKobukiData robotdata);
	QPair<double, double> calculateTrajectory();

private:
	void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
	void calculateOdometry(const TKobukiData &robotdata);

private slots:
	void on_pushButton_9_clicked();

	void on_pushButton_2_clicked();

	void on_pushButton_3_clicked();

	void on_pushButton_6_clicked();

	void on_pushButton_5_clicked();

	void on_pushButton_4_clicked();

	void on_pushButton_clicked();
	bool updateTarget(QLineEdit *lineEdit, PIDController *controller);
	void onSubmitButtonClicked(bool clicked);

public slots:
	void setUiValues(double robotX,double robotY,double robotFi);
	void timeout();

private: signals:
	void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo
	void startGuiding();

private:

	//--skuste tu nic nevymazat... pridavajte co chcete, ale pri odoberani by sa mohol stat nejaky drobny problem, co bude vyhadzovat chyby
	Ui::MainWindow *ui;
	int updateLaserPicture;
	LaserMeasurement copyOfLaserData;
	std::string ipaddress;
	Robot robot;
	RobotTrajectoryController *m_trajectoryController;
	TKobukiData robotdata;
	int datacounter;

	int lastLeftEncoder;
	int lastRightEncoder;
	double m_fi;
	double m_x;
	double m_y;

	PIDController m_xControl;
	PIDController m_yControl;

	time_point<steady_clock> m_time;
	double m_timeDiff;

	QTimer m_trajectoryTimer;

	QThread *m_trajectoryThread;
	QThread *m_controllerThread;
	QMutex m_mutex;

	double forwardspeed; //mm/s
	double rotationspeed; //omega/s

};

#endif // MAINWINDOW_H
