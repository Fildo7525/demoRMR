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
	void onSubmitButtonClicked(bool clicked);

public slots:
	void setUiValues(double robotX,double robotY,double robotFi);
	void timeout();

private: signals:
	void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo
	void startGuiding();

	void moveForward(double speed);
	void changeRotation(double rotation);

public: signals:
	void resultsReady(double distance, double rotaiton, QVector<QPointF> points);

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

	double m_xTarget;
	double m_yTarget;

	QThread *m_trajectoryThread;
	QThread *m_controllerThread;
	QMutex m_mutex;

	double forwardspeed; // mm/s
	double rotationspeed; // omega/s
};

#endif // MAINWINDOW_H
