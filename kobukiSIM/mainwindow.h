#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMutex>
//#include<windows.h>
#include <iostream>
//#include<arpa/inet.h>
//#include<unistd.h>
//#include<sys/socket.h>
#include <QUdpSocket>
#include <QTcpServer>
#include <QTcpSocket>
#include <QThread>
//#include<sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <thread>
#include "CKobuki.h"

#include "rplidar.h"
#include "map_loader.h"
#include <chrono>
#include "pozyxsim.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"

namespace Ui {
class MainWindow;
}

struct lineIntersectParams
{
	double t; //prva priamka
	double s; //druha priamka
};
struct cameraPos
{
	double x;
	double y;
};
struct sim_robot
{
	double x = 50;
	double y = 50;
	double fi = 0;
	unsigned short encoderLeft = 0;
	unsigned short encoderRight = 0;

	double encoderLeftA = 0;
	double encoderRightA = 0;
};

struct cameraInfo
{
	int imgsirka = 853;
	int imgvyska = 480;
	int startAngle = -32;
	int stopAngle = 32;
	int topAngle = 24;
	int bottomAngle = -24;
	std::map<int, bool> whichIntersect; //bool pouzivame nato ze ci to bolo v usporiadani pouzite a preradene dalej
};
class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	void generateAndSentImage();
	cameraInfo camerainf;
	int stopAll;
	QUdpSocket *robotRecv;

	QUdpSocket *laserRecv;
	QUdpSocket *pozyxRecv;
	QUdpSocket *pozyxAlmanachRecv;
	QHostAddress laseraddress;
	quint16 port;
	QHostAddress robotaddress;
	quint16 robotport;
	QHostAddress pozyxaddress;
	quint16 pozyxport;
	QHostAddress pozyxalmaaddress;
	quint16 pozyxalmaport;
	int hasLaserAddress;
	int hasRobotAddress;
	int hasPozyxAddress;
	int hasPozyxAlmaAddress;
	void readPendingDatagrams();
	void readPendingRoboDatagrams();
	void readPendingPozyxDatagrams();
	void readPendingPozyxAlmaDatagrams();
	std::vector<unsigned char> lastRobotMessage;

	std::thread robotsimthreadHandle;
	std::thread lasersimthreadHandle;
	std::thread pozyxsimthreadHandle;
	sim_robot robotik;
	map_loader mapload;
	TMapArea mapa;
	pozyxSim pozyx;
	explicit MainWindow(QWidget *parent = 0);

	~MainWindow();
	void robotprocess();
	void laserprocess();
	void processThisLidar(LaserMeasurement &laserData);

	void processThisRobot();
	// HANDLE robotthreadHandle; // handle na vlakno
	// DWORD robotthreadID;  // id vlakna

	void pozyxSimulator();
	void laserSimulator();
	void robotSimulator();
	void robotSimExec(std::vector<unsigned char> mess);


	double findLaserPoint(TMapArea &mapicka, sim_robot robotik, double angle);
	double lineIntersection(double X1, double Y1, double X2, double Y2, double Rx, double Ry, double Rx2, double Ry2);
	std::vector<int> getOrganizedObstaclesForImage();
	bool isObstacleInFronOfObstacle(int first, int second);
	bool isObstacleInFrontOfWall(int obstacle, int wall);
	bool isWallInFrontOfWall(int first, int second);
	lineIntersectParams obstlineIntersection(double X1, double Y1, double X2, double Y2, double X3, double Y3, double X4, double Y4);
	bool isInWay(double Rx, double Ry, double X1, double Y1, double X2, double Y2, double Px1, double Py1, double Px2, double Py2);
	double robotObstacleDistance(double X1, double Y1, double X2, double Y2, double Rx, double Ry);
	bool isRobotInCollision(TMapArea &mapicka, sim_robot robotik, double safedist);

	float sign(double p1x, double p1y, double p2x, double p2y, double p3x, double p3y);


	bool PointInTriangle(double ptx, double pty, double v1x, double v1y, double v2x, double v2y, double v3x, double v3y);
	QTcpServer *m_TcpHttpServer;
	QTcpSocket *m_TcpHttpClient;
	bool clientReady;


	std::promise<void> ready_promise;

	std::shared_future<void> readyFuture;

	double sphere_x;
	double sphere_y;

	bool generatedSphere;
	bool generatedHexagon;
private slots:
	void TcpHttpconnected();
	void TcpHttpreadyRead();
	void sendImageToWeb();

	void on_pushButton_clicked();

	void on_pushButton_2_clicked();

	void on_pushButton_3_clicked();

private:
	Ui::MainWindow *ui;
	void paintEvent(QPaintEvent *event); // Q_DECL_OVERRIDE;
	void closeEvent(QCloseEvent *event);
	int updateLaserPicture;
	LaserMeasurement copyOfLaserData;
	LaserMeasurement localLaser;
	std::string ipaddress;
	CKobuki robot;
	TKobukiData robotdata;
	int datacounter;
	cv::Mat frame;

	QMutex mutex;
	bool frameReadydy;
signals:
	void imageReady();
};

#endif // MAINWINDOW_H
