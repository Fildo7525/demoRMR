#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <QDebug>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
unsigned char emptyMessage[] = { 0x1A, 0x01, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x00, 0x00,
								 0x00, 0x00, 0x00, 0x00, 0x04, 0x07, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, ui(new Ui::MainWindow)
{
	srand(time(0));
	readyFuture = ready_promise.get_future();
	//tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
	ipaddress = "127.0.0.1";
	hasLaserAddress = 0;
	hasRobotAddress = 0;
	ui->setupUi(this);
	datacounter = 0;
	frame = cv::Mat::zeros(camerainf.imgvyska, camerainf.imgsirka, CV_8UC3);
	mapload.load_map("priestor.txt", mapa);
	frameReadydy = false;
	pozyx.loadBeacons("pozyx.txt");

	/*  std::function<void(void)> f2 =std::bind(&MainWindow::laserSimulator,this);
    lasersimthreadHandle=std::move(std::thread(f2));
    std::function<void(void)> f =std::bind(&MainWindow::robotSimulator,this);
    robotsimthreadHandle=std::move(std::thread(f));
    std::function<void(void)> f3 =std::bind(&MainWindow::pozyxSimulator,this);
    pozyxsimthreadHandle=std::move(std::thread(f3));
    stopAll=0;
    laserRecv = new QUdpSocket(this);
    laserRecv->bind(QHostAddress::Any, 5299);

    connect(laserRecv, &QUdpSocket::readyRead,
            this, &MainWindow::readPendingDatagrams);

    robotRecv = new QUdpSocket(this);
    robotRecv->bind(QHostAddress::Any, 5300);

    connect(robotRecv, &QUdpSocket::readyRead,
            this, &MainWindow::readPendingRoboDatagrams);

    pozyxRecv = new QUdpSocket(this);
    pozyxRecv->bind(QHostAddress::Any, 5301);

    connect(pozyxRecv, &QUdpSocket::readyRead,
            this, &MainWindow::readPendingPozyxDatagrams);

    pozyxAlmanachRecv = new QUdpSocket(this);
    pozyxAlmanachRecv->bind(QHostAddress::Any, 5302);

    connect(pozyxAlmanachRecv, &QUdpSocket::readyRead,
            this, &MainWindow::readPendingPozyxAlmaDatagrams);

    connect(this,&MainWindow::imageReady,this,&MainWindow::sendImageToWeb);
    m_TcpHttpServer = new QTcpServer();

      m_TcpHttpServer->connect(m_TcpHttpServer,
                               SIGNAL(newConnection()),
                               this,
                               SLOT(TcpHttpconnected()));

      m_TcpHttpServer->listen(QHostAddress::Any,
                              8889);*/
	clientReady = false;

	generatedSphere = false;
	generatedHexagon = false;

	sphere_x = -1; //mapload.minX + (float)(static_cast <float> (rand()) /( static_cast <float> (RAND_MAX)))*(mapload.maxX-mapload.minX);;
	sphere_y = -1; //mapload.minY + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(mapload.maxY-mapload.minY)));;
	// std::cout << "gula " << sphere_x << " " << sphere_y << std::endl;
	on_pushButton_3_clicked();
}

void MainWindow::sendImageToWeb()
{
	return;
	if (m_TcpHttpClient->isOpen()) {
		if (readyFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
			return;
		// Send Image

		// std::cout << "pisem na web" << std::endl;
		std::vector<uchar> buff;
		imencode(".jpg", frame, buff);
		std::string content(buff.begin(), buff.end());
		QByteArray CurrentImg(QByteArray::fromStdString(content));


		QByteArray BoundaryString = ("--boundary\r\n"
									 "Content-Type: image/jpeg\r\n"
									 "Content-Length: ");

		BoundaryString.append(QString::number(CurrentImg.length()));
		BoundaryString.append("\r\n\r\n");

		m_TcpHttpClient->write(BoundaryString);
		m_TcpHttpClient->write(CurrentImg); // Write The Encoded Image

		m_TcpHttpClient->flush();
		//  QThread::msleep(10);
	}
}

void MainWindow::TcpHttpconnected()
{
	m_TcpHttpClient = m_TcpHttpServer->nextPendingConnection();

	m_TcpHttpClient->connect(m_TcpHttpClient, SIGNAL(readyRead()), this, SLOT(TcpHttpreadyRead()));
}

void MainWindow::TcpHttpreadyRead()
{
	// std::cout << "akceptujem klienta" << std::endl;
	m_TcpHttpClient->readAll(); // Discard "Get Request String"

	QByteArray ContentType = ("HTTP/1.0 200 OK\r\n"
							  "Server: en.code-bude.net example server\r\n"
							  "Cache-Control: no-cache\r\n"
							  "Cache-Control: private\r\n"
							  "Content-Type: multipart/x-mixed-replace;boundary=--boundary\r\n\r\n");

	m_TcpHttpClient->write(ContentType);

	//  cv::Mat frame(480,640,CV_8UC1);

	clientReady = true;
	//  return;

	while (1) {
		if (readyFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
			// std::cout << "uz nieje klient" << std::endl;
			m_TcpHttpClient->disconnect(); //Should never be Reached
			break;
		}
		if (m_TcpHttpClient->isOpen()) {
			// Send Image

			mutex.lock();
			if (frameReadydy == true) {
				cv::rectangle(frame, cv::Rect(0, 0, camerainf.imgsirka, camerainf.imgvyska), cv::Scalar(rand() % 255, rand() % 255, rand() % 255));
				std::vector<uchar> buff;
				imencode(".jpg", frame, buff);

				std::string content(buff.begin(), buff.end());
				QByteArray CurrentImg(QByteArray::fromStdString(content));


				QByteArray BoundaryString = ("--boundary\r\n"
											 "Content-Type: image/jpeg\r\n"
											 "Content-Length: ");

				BoundaryString.append(QString::number(CurrentImg.length()));
				BoundaryString.append("\r\n\r\n");

				m_TcpHttpClient->write(BoundaryString);
				m_TcpHttpClient->write(CurrentImg); // Write The Encoded Image

				m_TcpHttpClient->flush();
				frameReadydy = false;
			}
			mutex.unlock();
			// return;
			QCoreApplication::processEvents();
			//  QThread::msleep(10);
		}
		else {
			// std::cout << "uz nieje klient" << std::endl;
			m_TcpHttpClient->disconnect(); //Should never be Reached
			return;
		}
	}

	// std::cout << "vypina sa tcp image" << std::endl;
	m_TcpHttpClient->disconnect(); //Should never be Reached
}

MainWindow::~MainWindow()
{
	// std::cout << "skoncim??" << std::endl;
	stopAll = 1;
	lasersimthreadHandle.join();
	robotsimthreadHandle.join();
	pozyxsimthreadHandle.join();

	robotRecv->close();
	laserRecv->close();
	if (m_TcpHttpClient != NULL)
		m_TcpHttpClient->disconnect(); //Should never be Reached
	std::cout << "koncim s ui" << std::endl;
	delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	// std::cout << "skoncim??" << std::endl;
	ready_promise.set_value();
}
void MainWindow::paintEvent(QPaintEvent *event)
{
	//   std::cout<<"robot "<<robotik.x<<" "<<robotik.y<<" "<<robotik.fi<<std::endl;
	QPainter painter(this);
	painter.setBrush(Qt::black);
	QPen pero;
	pero.setStyle(Qt::SolidLine);
	pero.setWidth(3);
	pero.setColor(Qt::green);
	QRect rect(20, 120, 700, 500);
	rect = ui->frame->geometry();
	rect.translate(0, 15);
	painter.drawRect(rect);
	//  double xmod=rect.width();
	//  double ymod=rect.height();
	for (int i = 0; i < mapa.wall.points.size(); i++) {
		painter.setPen(pero);
		int xmin = rect.width() * (mapa.wall.points[i].point.x - mapload.minX) / (mapload.maxX - mapload.minX);
		int xmax = rect.width() * (mapa.wall.points[(i + 1) % mapa.wall.points.size()].point.x - mapload.minX) / (mapload.maxX - mapload.minX);
		int ymin = rect.height() - rect.height() * (mapa.wall.points[i].point.y - mapload.minY) / (mapload.maxY - mapload.minY);
		int ymax = rect.height() - rect.height() * (mapa.wall.points[(i + 1) % mapa.wall.points.size()].point.y - mapload.minY) / (mapload.maxY - mapload.minY);
		painter.drawLine(rect.x() + xmin, rect.y() + ymin, rect.x() + xmax, rect.y() + ymax);
	}


	for (int i = 0; i < mapa.obstacle.size(); i++) {
		//   std::cout<<"obst "<<i<<std::endl;
		for (int j = 0; j < mapa.obstacle[i].points.size(); j++) {
			painter.setPen(pero);
			int xmin = rect.width() * (mapa.obstacle[i].points[j].point.x - mapload.minX) / (mapload.maxX - mapload.minX);
			int xmax = rect.width() * (mapa.obstacle[i].points[(j + 1) % mapa.obstacle[i].points.size()].point.x - mapload.minX) / (mapload.maxX - mapload.minX);
			int ymin = rect.height() - rect.height() * (mapa.obstacle[i].points[j].point.y - mapload.minY) / (mapload.maxY - mapload.minY);
			int ymax = rect.height()
				- rect.height() * (mapa.obstacle[i].points[(j + 1) % mapa.obstacle[i].points.size()].point.y - mapload.minY) / (mapload.maxY - mapload.minY);
			//   std::cout<<"xsur "<<mapa.obstacle[i].points[j].point.x<<" "<< mapa.obstacle[i].points[(j+1)%mapa.obstacle[i].points.size()].point.x<<std::endl;
			//   std::cout<<"ysur "<<mapa.obstacle[i].points[j].point.y<<" "<< mapa.obstacle[i].points[(j+1)%mapa.obstacle[i].points.size()].point.y<<std::endl;
			//   std::cout<<"xpic "<<xmin<<" "<< xmax<<std::endl;
			//   std::cout<<"ypic "<<ymin<<" "<< ymax<<std::endl;
			painter.drawLine(rect.x() + xmin, rect.y() + ymin, rect.x() + xmax, rect.y() + ymax);
		}
	}
	pero.setColor(Qt::red);
	painter.setPen(pero);
	int xrobot = rect.width() * (robotik.x - mapload.minX) / (mapload.maxX - mapload.minX);
	int yrobot = rect.height() - rect.height() * (robotik.y - mapload.minY) / (mapload.maxY - mapload.minY);
	int xpolomer = rect.width() * (20) / (mapload.maxX - mapload.minX);
	int ypolomer = rect.height() * (20) / (mapload.maxY - mapload.minY);
	painter.drawEllipse(QPoint(rect.x() + xrobot, rect.y() + yrobot), xpolomer, ypolomer);
	painter.drawLine(rect.x() + xrobot, rect.y() + yrobot, rect.x() + xrobot + xpolomer * cos((360 - robotik.fi) * 3.14159 / 180),
					 rect.y() + ((yrobot + ypolomer * sin((360 - robotik.fi) * 3.14159 / 180))));

	if (updateLaserPicture == 1) {
		/// ****************
		///you can change pen or pen color here if you want
		/// ****************
		painter.setPen(pero);
		for (int k = 0; k < copyOfLaserData.numberOfScans; k++) {
			int dist = copyOfLaserData.Data[k].scanDistance / 10;

			int xp = rect.x() + xrobot
				+ rect.width() * (dist * cos((robotik.fi + 360.0 - copyOfLaserData.Data[k].scanAngle) * 3.14159 / 180.0)) / (mapload.maxX - mapload.minX);
			int yp = rect.y() + yrobot
				+ (-(rect.height() * (dist * sin((robotik.fi + 360.0 - copyOfLaserData.Data[k].scanAngle) * 3.14159 / 180.0)) / (mapload.maxY - mapload.minY)));
			//  if(xp<721 && xp>19 && yp<621 && yp>121)
			painter.drawEllipse(QPoint(xp, yp), 2, 2);
		}
	}


	if (sphere_x != -1 && sphere_y != -1) {
		pero.setColor(Qt::blue);
		painter.setPen(pero);
		int xmin = rect.width() * (sphere_x - mapload.minX) / (mapload.maxX - mapload.minX);
		int ymin = rect.height() - rect.height() * (sphere_y - mapload.minY) / (mapload.maxY - mapload.minY);

		//      painter.drawEllipse(QPoint(xmin, ymin),7,7);
	}
	ui->statusBar->showMessage(QString::number(robotik.x - 50) + " " + QString::number(robotik.y - 50) + " " + QString::number(robotik.fi));
}


void MainWindow::laserSimulator()
{
	double frequencyBetweenpoints = 1000000 / (360 * 7);
	auto first = std::chrono::steady_clock::now();
	auto second = std::chrono::steady_clock::now();
	int angleit = 0;
	double angle = 0;
	while (stopAll == 0) {
		if (stopAll == 1) {
			// std::cout << "mam koniec" << std::endl;
			break;
		}
		if (readyFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
			cv::destroyAllWindows();
			break;
		}
		second = std::chrono::steady_clock::now();
		// if(((std::chrono::duration_cast<std::chrono::microseconds>(second-first)).count())>505)
		{
			int cas = ((std::chrono::duration_cast<std::chrono::microseconds>(second - first)).count());
			angle = angle
				+ 1.3; //(cas/frequencyBetweenpoints); //cas medzi vzorkami a frequencyBetweenpoints je preratane na kolko otacok za sekundu mame... ak by bolo 5.5,tak nam vyjde jeden stupen
			first = second;
			// std::cout<<angle<<std::endl;
			//   qDebug()<<angle;
			if (angle > 360.0) {
				//         std::cout<<"posielam "<<localLaser.numberOfScans<<std::endl;
				angle -= 360.0;
				angleit = 0;
				memset(&copyOfLaserData, 0, sizeof(LaserMeasurement));
				copyOfLaserData.numberOfScans = localLaser.numberOfScans;
				for (int i = 0; i < copyOfLaserData.numberOfScans; i++) {
					copyOfLaserData.Data[i].scanAngle = localLaser.Data[copyOfLaserData.numberOfScans - 1 - i].scanAngle;
					copyOfLaserData.Data[i].scanDistance = localLaser.Data[copyOfLaserData.numberOfScans - 1 - i].scanDistance;
				}
				// memcpy( &copyOfLaserData,&localLaser,sizeof(LaserMeasurement));
				//       std::cout<<copyOfLaserData.numberOfScans<<std::endl;
				//tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
				// ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
				updateLaserPicture = 1;
				localLaser.numberOfScans = 0;
				int returnval;
				if (hasLaserAddress == 1)
					returnval = laserRecv->writeDatagram((const char *)&copyOfLaserData.Data, sizeof(LaserData) * copyOfLaserData.numberOfScans, laseraddress,
														 /*52999*/ port);

				generateAndSentImage();
				camerainf.whichIntersect.clear();
				update();
				std::this_thread::sleep_for(std::chrono::milliseconds(170));
				//    Sleep(170);
			}
			localLaser.Data[angleit].scanDistance = findLaserPoint(mapa, robotik, angle) * 10;
			localLaser.Data[angleit].scanAngle = 360 - angle;
			localLaser.numberOfScans++;
			angleit++;
		}
		//std::this_thread::sleep_for (std::chrono::nanoseconds(100));
	}
}

double MainWindow::findLaserPoint(TMapArea &mapicka, sim_robot robotik, double angle)
{
	double dist = 5000;
	double vzdpx = robotik.x + 5000 * cos((robotik.fi + angle) * 3.14159 / 180.0);
	double vzdpy = robotik.y + 5000 * sin((robotik.fi + angle) * 3.14159 / 180.0);
	double ret;
	int minOBSid = -1;
	int wallID = -1;
	for (int i = 0; i < mapicka.wall.points.size(); i++) {
		ret = lineIntersection(mapicka.wall.points[i].point.x, mapicka.wall.points[i].point.y, mapicka.wall.points[(i + 1) % mapicka.wall.points.size()].point.x,
							   mapicka.wall.points[(i + 1) % mapicka.wall.points.size()].point.y, robotik.x, robotik.y, vzdpx, vzdpy);
		if (ret > 0) {
			if (dist > ret) {
				dist = ret;
				wallID = i;
			}
		}
	}


	for (int i = 0; i < mapicka.obstacle.size(); i++) {
		//   std::cout<<"obst "<<i<<std::endl;
		for (int j = 0; j < mapicka.obstacle[i].points.size(); j++) {
			ret = lineIntersection(mapicka.obstacle[i].points[j].point.x, mapicka.obstacle[i].points[j].point.y,
								   mapicka.obstacle[i].points[(j + 1) % mapicka.obstacle[i].points.size()].point.x,
								   mapicka.obstacle[i].points[(j + 1) % mapicka.obstacle[i].points.size()].point.y, robotik.x, robotik.y, vzdpx, vzdpy);
			if (ret > 0) {
				if (dist > ret) {
					dist = ret;
					wallID = -1;
					minOBSid = i;
				}
			}
		}
	}
	bool isInCamera = false;
	if (angle > 180) {
		if (angle > (360 + camerainf.startAngle))
			isInCamera = true;
	}
	else {
		if (angle < camerainf.stopAngle)
			isInCamera = true;
	}
	if (isInCamera == true) {
		if (wallID != -1) {
			camerainf.whichIntersect[-1 * wallID] = true;
		}
		else if (minOBSid != -1) {
			camerainf.whichIntersect[minOBSid + 1] = true;
		}
	}
	return dist > 500 ? 0 : dist;
}


std::vector<int> MainWindow::getOrganizedObstaclesForImage()
{
	std::vector<int> rets;
	while (!camerainf.whichIntersect.empty()) {
		for (auto const &xt : camerainf.whichIntersect) {
			bool used = true;
			for (auto const &yt : camerainf.whichIntersect) {
				if (readyFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
					break;

				if (xt.first > yt.first)
					continue;
				if (isObstacleInFronOfObstacle(xt.first, yt.first) == true) {
					used = false;
					break;
				}
			}
			if (used) {
				rets.push_back(xt.first);
				camerainf.whichIntersect.erase(xt.first);
				break;
			}
		}
	}
	return rets;
}


bool MainWindow::isObstacleInFronOfObstacle(int first, int second)
{
	/// najskor specialne (a jednoduche pripady)
	if (first == second)
		return false;
	if (first <= 0 && second <= 0) //ak su obidve sucast steny
	{
		return isWallInFrontOfWall(abs(first), abs(second));
	}
	if (first <= 0) //ak je prvy stena
	{
		//--- ten co sa dotazujeme je stena, cize ak je prekazka pred stenou, vraciam false, lebo stena nieje pred prekazkou
		return isObstacleInFrontOfWall(second - 1, abs(first));
	}
	if (second <= 0) //ak je druhy stena
	{
		//dotazujeme sa na prekazku, cize ak je pred stenou vraciam true
		return !isObstacleInFrontOfWall(first - 1, abs(second));
	}
	///------------------
	///
	///a teraz ked obidve su prekazky
	/// hnusne preiterovanie
	int firstID = first - 1;
	int secondID = second - 1;
	for (int i = 0; i < mapa.obstacle[firstID].points.size(); i++) {
		int nextI = (i + 1) % mapa.obstacle[firstID].points.size();
		for (int j = 0; j < mapa.obstacle[secondID].points.size(); j++) {
			int nextJ = (j + 1) % mapa.obstacle[secondID].points.size();
			if (isInWay(robotik.x, robotik.y, mapa.obstacle[firstID].points[i].point.x, mapa.obstacle[firstID].points[i].point.y,
						mapa.obstacle[firstID].points[nextI].point.x, mapa.obstacle[firstID].points[nextI].point.y, mapa.obstacle[secondID].points[j].point.x,
						mapa.obstacle[secondID].points[j].point.y, mapa.obstacle[secondID].points[nextJ].point.x, mapa.obstacle[secondID].points[nextJ].point.y)
				== true) {
				//      std::cout<<"prekazka "<<first<<" ma pred sebou "<<second<<" zisteny prienik z bodu "<<i<<"prvej a "<<" druhej "<<j<<std::endl;
				return true;
			}
		}
	}
	return false;
}

bool MainWindow::isObstacleInFrontOfWall(int obstacle, int wall)
{
	return true; //pre standardny priestor vzdy pravda.. konvexna stena.. zatial neimplementujeme zvysok
}

bool MainWindow::isWallInFrontOfWall(int first, int second)
{
	//zatial neriesim. stena je pred stenou.
	return true;
}

void MainWindow::generateAndSentImage()
{
	double z0 = 0 - 28.5;  //15 je poloha robota
	double z1 = 40 - 28.5; // 40 je vyska platne
	mutex.lock();
	int imgsirka = camerainf.imgsirka;
	int imgvyska = camerainf.imgvyska;
	cv::Mat imgr = cv::Mat::zeros(imgvyska, imgsirka, CV_8UC3);
	cv::rectangle(frame, cv::Rect(0, imgvyska / 2, imgsirka, imgvyska / 2), cv::Scalar(128, 128, 128), -1);
	cv::rectangle(frame, cv::Rect(0, 0, imgsirka, imgvyska / 2), cv::Scalar(128, 50, 50), -1);
	double f = (imgsirka / 2) / tan(camerainf.stopAngle * 3.14159 / 180.0); // polka sirky obrazka, deleno atan polky rozsahu uhla
	double xfoc = f / (imgsirka / 2);
	// std::cout << "focal je " << f << std::endl;

	std::vector<int> orgn = getOrganizedObstaclesForImage();

	///zistim gulu, kde sa nachadza..
	///
	if (sphere_x != -1 && sphere_y != -1) {
		double sphere_angl = (atan2((sphere_y - robotik.y), (sphere_x - robotik.x)) * 180.0 / 3.14159) - robotik.fi;
		double sphere_dst = sqrt(pow((sphere_x - robotik.x), 2) + pow((sphere_y - robotik.y), 2));

		sphere_angl = sphere_angl < -180 ? sphere_angl + 360 : sphere_angl > 180 ? sphere_angl - 360 : sphere_angl;
		double sphere_x2 = sphere_dst * cos(sphere_angl * 3.14159 / 180.0);
		double sphere_y2 = sphere_dst * sin(sphere_angl * 3.14159 / 180.0);
		//      std::cout<<"a "<<x2<<" "<<y2<<std::endl;


		// std::cout << "sphere angle " << sphere_angl << std::endl;


		double sphere_xobr = imgsirka / 2 - (sphere_y2 / fabs(sphere_x2)) * f;
		double spehere_yobr = (-50 / fabs(sphere_x2)) * f + imgvyska / 2;


		double sphere_radius = atan2(20, sphere_dst) * f;
		;


		if (fabs(sphere_angl) < 90)
			cv::circle(frame, cv::Point(sphere_xobr, spehere_yobr), sphere_radius, cv::Scalar(50, 200, 50), cv::FILLED);
	}
	cameraPos campos;
	double camlidoffset = 14;
	campos.x = robotik.x + cos(robotik.fi * 3.14159 / 180.0) * camlidoffset;
	campos.y = robotik.y + sin(robotik.fi * 3.14159 / 180.0) * camlidoffset;
	for (auto it = orgn.rbegin(); it != orgn.rend(); ++it) {
		//  std::cout<<"stena "<< *it<<std::endl;
		/*  std::cout << xt.first  // string (key)
                  << ':'
                  << xt.second // string's value
                  << std::endl;*/

		if (*it <= 0) {
			double xobr, yobr1, yobr2;
			cv::Scalar color = cv::Scalar(255, 255, 255);
			cv::Point ptts[1][4];
			int npt[] = { 4 };
			int sur = abs(*it);

			double dst = sqrt(pow((mapa.wall.points[sur].point.x - campos.x), 2) + pow((mapa.wall.points[sur].point.y - campos.y), 2));
			double angl = (atan2((mapa.wall.points[sur].point.y - campos.y), (mapa.wall.points[sur].point.x - campos.x)) * 180.0 / 3.14159) - robotik.fi;

			double x1 = dst * cos(angl * 3.14159 / 180.0);
			double y1 = dst * sin(angl * 3.14159 / 180.0);
			//  std::cout<<"poloha steny je "<<x1<<" "<<y1<<std::endl;


			dst = sqrt(pow((mapa.wall.points[(sur + 1) % mapa.wall.points.size()].point.x - campos.x), 2)
					   + pow((mapa.wall.points[(sur + 1) % mapa.wall.points.size()].point.y - campos.y), 2));
			angl = (atan2((mapa.wall.points[(sur + 1) % mapa.wall.points.size()].point.y - campos.y),
						  (mapa.wall.points[(sur + 1) % mapa.wall.points.size()].point.x - campos.x))
					* 180.0 / 3.14159)
				- robotik.fi;

			double x2 = dst * cos(angl * 3.14159 / 180.0);
			double y2 = dst * sin(angl * 3.14159 / 180.0);
			//      std::cout<<"a "<<x2<<" "<<y2<<std::endl;

			if (x1 < 0) {
				double A = (y2 - y1) / (x2 - x1);
				//  double xzel=((f/320.0) *(y1 -x1*A))/(1.0-(f/320.0)*A);


				double yzel = y1 + (xfoc - x1) * A;

				double xzel = xfoc;
				//     std::cout<<"nove x a y "<<xzel<<" "<<yzel<<std::endl;;
				x1 = xzel;
				y1 = yzel;
			}
			else if (x2 < 0) {
				double A = (y1 - y2) / (x1 - x2);
				//  double xzel=((f/320.0) *(y1 -x1*A))/(1.0-(f/320.0)*A);


				double yzel = y2 + (xfoc - x2) * A;

				double xzel = xfoc;
				//    std::cout<<"nove x a y "<<xzel<<" "<<yzel<<std::endl;;
				x2 = xzel;
				y2 = yzel;
			}


			xobr = imgsirka / 2 - (y1 / fabs(x1)) * f;
			yobr1 = (z0 / fabs(x1)) * f + imgvyska / 2;
			yobr2 = (z1 / fabs(x1)) * f + imgvyska / 2;

			ptts[0][0] = cv::Point(xobr, yobr1);
			ptts[0][1] = cv::Point(xobr, yobr2);
			//  std::cout<<"pixels "<<xobr<<" "<<yobr1<<" "<<yobr2<<std::endl;


			xobr = imgsirka / 2 - (y2 / fabs(x2)) * f;
			yobr1 = (z0 / fabs(x2)) * f + imgvyska / 2;
			yobr2 = (z1 / fabs(x2)) * f + imgvyska / 2;

			ptts[0][2] = cv::Point(xobr, yobr2);
			ptts[0][3] = cv::Point(xobr, yobr1);
			//    std::cout<<"pixels "<<xobr<<" "<<yobr1<<" "<<yobr2<<std::endl;

			const cv::Point *ppt[1] = { ptts[0] };
			cv::fillPoly(frame, ppt, npt, 1, color);
			cv::polylines(frame, ppt, npt, 1, true, cv::Scalar(170, 170, 170));
		}
		else {
			double xobr, yobr1, yobr2;
			int obstc = *it - 1;
			int i = 0;
			for (int i = 0; i < mapa.obstacle[obstc].points.size(); i++) {
				cv::Scalar color = cv::Scalar(255, 255, 255);
				cv::Point ptts[1][4];
				int npt[] = { 4 };
				int sur = i;
				double dst = sqrt(pow((mapa.obstacle[obstc].points[sur].point.x - campos.x), 2) + pow((mapa.obstacle[obstc].points[sur].point.y - campos.y), 2));
				double angl = (atan2((mapa.obstacle[obstc].points[sur].point.y - campos.y), (mapa.obstacle[obstc].points[sur].point.x - campos.x)) * 180.0 / 3.14159)
					- robotik.fi;

				double x1 = dst * cos(angl * 3.14159 / 180.0);
				double y1 = dst * sin(angl * 3.14159 / 180.0);
				//      std::cout<<"poloha steny je "<<x1<<" "<<y1<<std::endl;
				sur = (i + 1) % mapa.obstacle[obstc].points.size();
				dst = sqrt(pow((mapa.obstacle[obstc].points[sur].point.x - campos.x), 2) + pow((mapa.obstacle[obstc].points[sur].point.y - campos.y), 2));
				angl = (atan2((mapa.obstacle[obstc].points[sur].point.y - campos.y), (mapa.obstacle[obstc].points[sur].point.x - campos.x)) * 180.0 / 3.14159)
					- robotik.fi;

				double x2 = dst * cos(angl * 3.14159 / 180.0);
				double y2 = dst * sin(angl * 3.14159 / 180.0);
				if (x1 < 0) {
					double A = (y2 - y1) / (x2 - x1);
					//  double xzel=((f/320.0) *(y1 -x1*A))/(1.0-(f/320.0)*A);


					double yzel = y1 + (xfoc - x1) * A;

					double xzel = xfoc;
					//       std::cout<<"nove x a y "<<xzel<<" "<<yzel<<std::endl;;
					x1 = xzel;
					y1 = yzel;
				}
				else if (x2 < 0) {
					double A = (y1 - y2) / (x1 - x2);
					//  double xzel=((f/320.0) *(y1 -x1*A))/(1.0-(f/320.0)*A);


					double yzel = y2 + (xfoc - x2) * A;

					double xzel = xfoc;
					//      std::cout<<"nove x a y "<<xzel<<" "<<yzel<<std::endl;;
					x2 = xzel;
					y2 = yzel;
				}


				xobr = imgsirka / 2 - (y1 / fabs(x1)) * f;
				yobr1 = (z0 / fabs(x1)) * f + imgvyska / 2;
				yobr2 = (z1 / fabs(x1)) * f + imgvyska / 2;

				ptts[0][0] = cv::Point(xobr, yobr1);
				ptts[0][1] = cv::Point(xobr, yobr2);
				//     std::cout<<"pixels "<<xobr<<" "<<yobr1<<" "<<yobr2<<std::endl;


				xobr = imgsirka / 2 - (y2 / fabs(x2)) * f;
				yobr1 = (z0 / fabs(x2)) * f + imgvyska / 2;
				yobr2 = (z1 / fabs(x2)) * f + imgvyska / 2;

				ptts[0][2] = cv::Point(xobr, yobr2);
				ptts[0][3] = cv::Point(xobr, yobr1);
				//     std::cout<<"pixels "<<xobr<<" "<<yobr1<<" "<<yobr2<<std::endl;

				const cv::Point *ppt[1] = { ptts[0] };
				cv::fillPoly(frame, ppt, npt, 1, cv::Scalar(170 + 5 * obstc, 255 - 5 * obstc, 255));
				cv::polylines(frame, ppt, npt, 1, true, cv::Scalar(170, 170, 170));
			}
		}
	}

	/*   for(int i=0;i<copyOfLaserData.numberOfScans;i++)
    {
        if(copyOfLaserData.Data[i].scanAngle>33 && copyOfLaserData.Data[i].scanAngle<327)
            continue;
        double X=copyOfLaserData.Data[i].scanDistance*cos(-copyOfLaserData.Data[i].scanAngle*3.14159/180.0)/10.0;
        double Y=copyOfLaserData.Data[i].scanDistance*sin(-copyOfLaserData.Data[i].scanAngle*3.14159/180.0)/10.0;


        double Z=21; //0 lebo dole je -9.5 co je voci lidaru.. alternativne, tu by malo byt 21 a potom dole +11.5

        double xobr=imgsirka/2 -(f*Y)/(X-14.5);
        double yobr=imgvyska/2 + (f*(-Z+11.5))/(X-14.5);
         cv::circle(frame,cv::Point(xobr,yobr),3,cv::Scalar(0,170,0));
    }*/
	//frame=imgr.clone();
	frameReadydy = true;
	mutex.unlock();
	cv::imshow("server", frame);
	if (!clientReady)
		return;

	emit imageReady();
}
double MainWindow::lineIntersection(double X1, double Y1, double X2, double Y2, double Rx, double Ry, double Rx2, double Ry2)
{
	int pouzi = 0;
	double Ax, Ay, Bx, By, Cx, Cy, Dx, Dy, u1, u2, v1, v2, t, s;
	double vystupX, vystupY;
	//nastavenie prvej usecky zadanej bodmy A a B
	Ax = Rx; //prvy bod je robot
	Ay = Ry;
	Bx = Rx2; // druhy bod je koncovy bod kam laser dosiahne
	By = Ry2;
	u1 = Bx - Ax; //smerovy vektor pre prvu usecku
	u2 = By - Ay;

	{
		//druha usecka zadana bodmi C a D
		Cx = (X1); //vyberieme prvy bod
		Cy = (Y1);
		//druhy bod vyberieme tak,ze ak sme do C dali posledny bod, tak do D ide prvy, inak ide nasledujuci
		Dx = X2;
		Dy = Y2;
		v1 = Dx - Cx; //smerovy vektor druhej usecky
		v2 = Dy - Cy;

		t = (u1 * Ay + u2 * Cx - Ax * u2 - Cy * u1)
			/ (v2 * u1 - u2 * v1);	 //vyratame bod prieniku (ak je prienik useciek ,tak musi byt s intervalu 0 -1, ak je mimo tak je to prienik priamok)
		s = (Cx - Ax + v1 * t) / u1; //to iste pre druhy smerovy vektor
		if ((t > 0.0) && (s > 0.00) && (t < 1.00) && (s < 1.00)) {
			//--je prienik. zistim bod a zratam dlzku..
			vystupX = Rx + s * u1;
			vystupY = Ry + s * u2;
			return sqrt(pow(Rx - vystupX, 2) + pow(Ry - vystupY, 2));
		}

		return -1;
	}
}

//ci je prekazka (alebo jej cast) v ceste nasej casti prekazky
///myslienka. spravim najskor ci ma spojnica R a XY1 prienik s priamkov  P1P2. ak nie, overim ci to nema usecka R a XY2
/// ak su obidve mimo,som v pohode a nieje v ceste.
/// ak je aspon jedna vo vnutri treba zistit, ako je na tom usecka P1P2, ak je parameter v rozsahu 0 1,tak je problem.
/// ak nieje, treba overit, ci este nahodou nieje tak, ze ci nieje cela usecka vo vnutri.. zratam ci je bod P1(alebo P2) vo vnutri trojuholnika
/// https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
bool MainWindow::isInWay(double Rx, double Ry, double X1, double Y1, double X2, double Y2, double Px1, double Py1, double Px2, double Py2)
{
	int pouzi = 0;

	lineIntersectParams firstpriamka = obstlineIntersection(Rx, Ry, X1, Y1, Px1, Py1, Px2, Py2);


	lineIntersectParams secondpriamka = obstlineIntersection(Rx, Ry, X2, Y2, Px1, Py1, Px2, Py2);
	if (firstpriamka.t >= 1 && firstpriamka.t <= 0 && secondpriamka.t >= 1 && secondpriamka.t <= 0) {
		//   std::cout<<"vyhodnotil som na zaklade t "<<firstpriamka.t<<" "<<secondpriamka.t<<std::endl;
		return false;
	}
	//je tu riziko
	if (firstpriamka.s < 1 && firstpriamka.s > 0 && firstpriamka.t < 1 && firstpriamka.t > 0) {
		//    std::cout<<"first priamka riziko "<<firstpriamka.s<<std::endl;
		return true; //ano, je to priamo prienik s prvou spojnicou robot prekazka
	}
	if (secondpriamka.s < 1 && secondpriamka.s > 0 && secondpriamka.t < 1 && secondpriamka.t > 0) {
		//   std::cout<<"second priamka riziko "<<secondpriamka.s<<std::endl;
		return true; //ano, je to priamo prienik s druhou spojnicou robot prekazka
	}
	//uz moze byt problem len tak,ze je vo vnutri cela usecka. zistim ci nieje jeden bod v trojuholniku (jedno ktory)
	if (PointInTriangle(Px1, Py1, Rx, Ry, X1, Y1, X2, Y2) == true) {
		//   std::cout<<"som v trojuholniku"<<std::endl;
		return true;
	}
	return false;
}

lineIntersectParams MainWindow::obstlineIntersection(double X1, double Y1, double X2, double Y2, double X3, double Y3, double X4, double Y4)
{
	lineIntersectParams retparams;

	int pouzi = 0;
	double Ax, Ay, Bx, By, Cx, Cy, Dx, Dy, u1, u2, v1, v2, t, s;
	double vystupX, vystupY;
	//nastavenie prvej usecky zadanej bodmy A a B
	Ax = X1; //prvy bod je robot
	Ay = Y1;
	Bx = X2; // druhy bod je koncovy bod kam laser dosiahne
	By = Y2;
	u1 = Bx - Ax; //smerovy vektor pre prvu usecku
	u2 = By - Ay;

	{
		//druha usecka zadana bodmi C a D
		Cx = (X3); //vyberieme prvy bod
		Cy = (Y3);
		//druhy bod vyberieme tak,ze ak sme do C dali posledny bod, tak do D ide prvy, inak ide nasledujuci
		Dx = X4;
		Dy = Y4;
		v1 = Dx - Cx; //smerovy vektor druhej usecky
		v2 = Dy - Cy;

		t = (u1 * Ay + u2 * Cx - Ax * u2 - Cy * u1)
			/ (v2 * u1 - u2 * v1);	 //vyratame bod prieniku (ak je prienik useciek ,tak musi byt s intervalu 0 -1, ak je mimo tak je to prienik priamok)
		s = (Cx - Ax + v1 * t) / u1; //to iste pre druhy smerovy vektor

		retparams.s = t;
		retparams.t = s;
		return retparams;
	}
}

float MainWindow::sign(double p1x, double p1y, double p2x, double p2y, double p3x, double p3y)
{
	return (p1x - p3x) * (p2y - p3y) - (p2x - p3x) * (p1y - p3y);
}

bool MainWindow::PointInTriangle(double ptx, double pty, double v1x, double v1y, double v2x, double v2y, double v3x, double v3y)
{
	float d1, d2, d3;
	bool has_neg, has_pos;

	d1 = sign(ptx, pty, v1x, v1y, v2x, v2y);
	d2 = sign(ptx, pty, v2x, v2y, v3x, v3y);
	d3 = sign(ptx, pty, v3x, v3y, v1x, v1y);

	has_neg = (d1 <= 0) || (d2 <= 0) || (d3 <= 0);
	has_pos = (d1 >= 0) || (d2 >= 0) || (d3 >= 0);

	return !(has_neg && has_pos);
}

bool MainWindow::isRobotInCollision(TMapArea &mapicka, sim_robot robotik, double safedist)
{
	double ret;
	for (int i = 0; i < mapicka.wall.points.size(); i++) {
		ret = robotObstacleDistance(mapicka.wall.points[i].point.x, mapicka.wall.points[i].point.y,
									mapicka.wall.points[(i + 1) % mapicka.wall.points.size()].point.x,
									mapicka.wall.points[(i + 1) % mapicka.wall.points.size()].point.y, robotik.x, robotik.y);
		if (ret < safedist)
			return true;
	}


	for (int i = 0; i < mapicka.obstacle.size(); i++) {
		//   std::cout<<"obst "<<i<<std::endl;
		for (int j = 0; j < mapicka.obstacle[i].points.size(); j++) {
			ret = robotObstacleDistance(mapicka.obstacle[i].points[j].point.x, mapicka.obstacle[i].points[j].point.y,
										mapicka.obstacle[i].points[(j + 1) % mapicka.obstacle[i].points.size()].point.x,
										mapicka.obstacle[i].points[(j + 1) % mapicka.obstacle[i].points.size()].point.y, robotik.x, robotik.y);
			if (ret < safedist)
				return true;
		}
	}
	return false;
}

double MainWindow::robotObstacleDistance(double X1, double Y1, double X2, double Y2, double Rx, double Ry)
{
	double A = Rx - X1;
	double B = Ry - Y1;
	double C = X2 - X1;
	double D = Y2 - Y1;

	double dot = A * C + B * D;
	double len_sq = C * C + D * D;
	double param = -1;
	if (len_sq != 0) //in case of 0 length line
		param = dot / len_sq;

	double xx, yy;

	if (param < 0) {
		xx = X1;
		yy = Y1;
	}
	else if (param > 1) {
		xx = X2;
		yy = Y2;
	}
	else {
		xx = X1 + param * C;
		yy = Y1 + param * D;
	}

	double dx = Rx - xx;
	double dy = Ry - yy;
	return sqrt(dx * dx + dy * dy);
}
void MainWindow::robotSimulator()
{
	double frequencyBetweenpoints = 1000000 / (360 * 7);
	auto first = std::chrono::steady_clock::now();
	auto second = std::chrono::steady_clock::now();
	int angleit = 0;
	double angle = 0;
	while (stopAll == 0) {
		if (readyFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
			break;
		second = std::chrono::steady_clock::now();
		if (((std::chrono::duration_cast<std::chrono::microseconds>(second - first)).count()) >= 25000) {
			first = second;
			robotSimExec(lastRobotMessage);
			if (hasRobotAddress == 1) {
				unsigned short temp = (unsigned short)robotik.encoderLeftA;
				emptyMessage[9] = temp >> 8;
				emptyMessage[8] = temp % 256;
				temp = (unsigned short)robotik.encoderRightA;
				emptyMessage[11] = temp >> 8;
				emptyMessage[10] = temp % 256;
				double uhol = robotik.fi;

				std::cout << uhol << std::endl;
				if (uhol > 180) {
					robotik.fi = robotik.fi - 360;
					uhol = robotik.fi;
				}
				if (uhol < -180) {
					robotik.fi = 360 + robotik.fi;
					uhol = robotik.fi;
				}
				signed short GyroAngle = (uhol * 100);
				emptyMessage[21] = GyroAngle >> 8;
				emptyMessage[20] = GyroAngle % 256;
				unsigned char chckSum = 0;
				for (int i = 0; i < emptyMessage[0] + 1; i++) {
					chckSum ^= emptyMessage[i];
				}
				emptyMessage[emptyMessage[0] + 1] = chckSum;
				robotRecv->writeDatagram((char *)&emptyMessage, 28, robotaddress, robotport);
			}
		}
		std::this_thread::sleep_for(std::chrono::microseconds(1000));
	}
	std::cout << "skoncil robot " << std::endl;
}

void MainWindow::robotSimExec(std::vector<unsigned char> mess)
{
	//najprv zistime,co za data dosli...
	int itrt = 3;
	short speed = 0;
	short radius = 0;
	double relspeed = 0;
	double relradius = 0;
	if (mess.size() < 4)
		return;
	if (mess[0] == 0xaa && mess[1] == 0x55) {
		do {
			if (mess.size() <= itrt + 1)
				return;
			if (mess[itrt] == 0x01)
				break;

			itrt = itrt + mess[itrt + 1] + 2;
			if (readyFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
				break;
		} while (mess[itrt] != 0x01);
		if (mess[itrt] == 0x01) {
			speed = mess[itrt + 2] + mess[itrt + 3] * 256;
			radius = mess[itrt + 4] + mess[itrt + 5] * 256;

			//--prepocty
			if (radius == 1) //--specialny pripad tocenia na mieste
			{
				relspeed = (double)speed * 2.0 / 230.0;
				relradius = 0;
			}
			else if (radius == 0) //--specialny pripad dopredu
			{
				relspeed = speed;
				relradius = 32767;
			}
			else {
				//int speedvalue = mmpersec * ((radius + (radius>0? 230:-230) )/ 2 ) / radius;
				relspeed = (double)speed * radius / ((double)(radius + (radius > 0 ? 230.0 : -230.0)) / 2.0);
				relradius = radius;
			}
			//   std::cout<<"vyratal "<<relspeed<<" "<<relradius<<std::endl;
		}
	}
	sim_robot temp_robot;

	short encoderLeft = 0, encoderRight = 0;
	double encll = 0, encrr = 0;
	if (relradius == 32767) {
		temp_robot.x = robotik.x + cos(robotik.fi * 3.14159 / 180.0) * (relspeed / 10) * 0.025;
		temp_robot.y = robotik.y + sin(robotik.fi * 3.14159 / 180.0) * (relspeed / 10) * 0.025;
		temp_robot.fi = robotik.fi;
		encoderLeft += 0.025 * ((relspeed / 1000.0) / 0.000085292090497737556558);
		encll = 0.025 * ((relspeed / 1000.0) / 0.000085292090497737556558);
		encoderRight += 0.025 * ((relspeed / 1000.0) / 0.000085292090497737556558);
		encrr += 0.025 * ((relspeed / 1000.0) / 0.000085292090497737556558);
	}
	else if (relradius == 0) {
		temp_robot.x = robotik.x;
		temp_robot.y = robotik.y;
		temp_robot.fi = robotik.fi + (180.0 / 3.14159) * relspeed * 0.025;
		encoderLeft -= 0.025 * ((relspeed * 230.0 / 2.0) / 1000.0) / 0.000085292090497737556558;
		encll -= 0.025 * ((relspeed * 230.0 / 2.0) / 1000.0) / 0.000085292090497737556558;
		encoderRight += 0.025 * ((relspeed * 230.0 / 2.0) / 1000.0) / 0.000085292090497737556558;
		encrr += 0.025 * ((relspeed * 230.0 / 2.0) / 1000.0) / 0.000085292090497737556558;
	}
	else {
		double lspeed = relspeed - ((115.0 * (relspeed)) / radius);
		double rspeed = relspeed + ((115.0 * (relspeed)) / radius);
		double helpangle = robotik.fi + ((rspeed - lspeed) / 230.0) * (0.025 * 180.0 / 3.14159);
		temp_robot.x = robotik.x + (radius / 10.0) * (sin(helpangle * 3.14159 / 180.0) - sin(robotik.fi * 3.14159 / 180.0));
		temp_robot.y = robotik.y - (radius / 10.0) * (cos(helpangle * 3.14159 / 180.0) - cos(robotik.fi * 3.14159 / 180.0));
		temp_robot.fi = helpangle;
		encoderLeft += 0.025 * (lspeed / 1000.0) / 0.000085292090497737556558;
		encll = 0.025 * (lspeed / 1000.0) / 0.000085292090497737556558;
		encoderRight += 0.025 * (rspeed / 1000.0) / 0.000085292090497737556558;
		encrr += 0.025 * (rspeed / 1000.0) / 0.000085292090497737556558;
	}
	// std::cout << "encoder diff " << encoderLeft << " " << encll << std::endl;
	if (isRobotInCollision(mapa, temp_robot, 20) == false) {
		encoderLeft = encoderLeft + robotik.encoderLeft;
		encoderRight = encoderRight + robotik.encoderRight;
		encrr = encrr + robotik.encoderRightA;
		encll = encll + robotik.encoderLeftA;
		robotik = temp_robot;
		robotik.encoderLeft = encoderLeft;
		robotik.encoderRight = encoderRight;

		robotik.encoderLeft = robotik.encoderLeft > 65535 ? robotik.encoderLeft - 65535 : robotik.encoderLeft;
		robotik.encoderRight = robotik.encoderRight > 65535 ? robotik.encoderRight - 65535 : robotik.encoderRight;
		robotik.encoderLeft = robotik.encoderLeft < 0 ? 65536 + robotik.encoderLeft : robotik.encoderLeft;
		robotik.encoderRight = robotik.encoderRight < 0 ? 65536 + robotik.encoderRight : robotik.encoderRight;


		encrr = encrr + robotik.encoderRightA;
		encll = encll + robotik.encoderLeftA;
		robotik.encoderRightA = encrr;
		robotik.encoderLeftA = encll;

		robotik.encoderRightA = robotik.encoderRightA > 65535.0 ? robotik.encoderRightA - 65536.0
			: robotik.encoderRightA < 0							? robotik.encoderRightA + 65536
																: robotik.encoderRightA;
		robotik.encoderLeftA = robotik.encoderLeftA > 65535.0 ? robotik.encoderLeftA - 65536.0
			: robotik.encoderLeftA < 0						  ? robotik.encoderLeftA + 65536.0
															  : robotik.encoderLeftA;
	}
	// std::cout << robotik.x << " " << robotik.y << " " << robotik.encoderRightA << " " << robotik.encoderLeftA << std::endl;
	update();
}


void MainWindow::readPendingDatagrams()
{
	while (laserRecv->hasPendingDatagrams()) {
		if (readyFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
			break;
		int sized = laserRecv->pendingDatagramSize();

		char *data = (char *)calloc(sized, sizeof(char));
		laserRecv->readDatagram(data, sized, &laseraddress, &port);
		hasLaserAddress = 1;
		//       processTheDatagram(datagram);
	}
}

void MainWindow::readPendingRoboDatagrams()
{
	while (robotRecv->hasPendingDatagrams()) {
		if (readyFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
			break;
		int sized = robotRecv->pendingDatagramSize();

		char *data = (char *)calloc(sized, sizeof(char));
		robotRecv->readDatagram(data, sized, &robotaddress, &robotport);
		std::vector<unsigned char> dd(&data[0], &data[sized]);
		lastRobotMessage = dd;
		hasRobotAddress = 1;
		//       processTheDatagram(datagram);
	}
}

void MainWindow::readPendingPozyxDatagrams()
{
	while (pozyxRecv->hasPendingDatagrams()) {
		if (readyFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
			break;
		int sized = pozyxRecv->pendingDatagramSize();

		if (sized == 0)
			return;
		char *data = (char *)calloc(sized, sizeof(char));
		pozyxRecv->readDatagram(data, sized, &pozyxaddress, &pozyxport);
		// std::cout << "dostal som pozyx request" << std::endl;
		hasPozyxAddress = 1;
		//       processTheDatagram(datagram);
	}
}

void MainWindow::readPendingPozyxAlmaDatagrams()
{
	while (pozyxAlmanachRecv->hasPendingDatagrams()) {
		if (readyFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
			break;
		int sized = pozyxAlmanachRecv->pendingDatagramSize();
		if (sized == 0)
			return;
		char *data = (char *)calloc(sized, sizeof(char));
		pozyxAlmanachRecv->readDatagram(data, sized, &pozyxalmaaddress, &pozyxalmaport);

		// std::cout << "dostal som alma request" << std::endl;
		hasPozyxAlmaAddress = 1;
		//       processTheDatagram(datagram);
	}
}

void MainWindow::pozyxSimulator()
{
	auto first = std::chrono::steady_clock::now();
	auto second = std::chrono::steady_clock::now();

	auto first2 = std::chrono::steady_clock::now();
	auto second2 = std::chrono::steady_clock::now();
	int angleit = 0;
	double angle = 0;
	while (stopAll == 0) {
		if (readyFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
			break;
		second = std::chrono::steady_clock::now();
		second2 = std::chrono::steady_clock::now();
		if (((std::chrono::duration_cast<std::chrono::microseconds>(second - first)).count()) > 1000000) {
			if (hasPozyxAddress == 1) {
				sim_robot robot = robotik;
				std::vector<distBeacon> dists = pozyx.calculateDistForBeacons(robot.x, robot.y);
				char *beacondists = (char *)calloc(dists.size(), sizeof(double) + sizeof(int));
				int offset = 0;
				for (int i = 0; i < dists.size(); i++) {
					memcpy(&beacondists[offset], &(dists[i].id), sizeof(int));
					offset += sizeof(int);
					memcpy(&beacondists[offset], &(dists[i].dist), sizeof(double));
					offset += sizeof(double);
				}
				pozyxRecv->writeDatagram(beacondists, dists.size() * (sizeof(double) + sizeof(int)), pozyxaddress, pozyxport);
			}
			first = second;
		}

		if (((std::chrono::duration_cast<std::chrono::microseconds>(second2 - first2)).count()) > 10000000) {
			if (hasPozyxAlmaAddress == 1) {
				std::vector<beacon> dists = pozyx.getBeacons();
				char *beacondists = (char *)calloc(dists.size(), sizeof(double) + sizeof(double) + sizeof(int));
				int offset = 0;
				for (int i = 0; i < dists.size(); i++) {
					memcpy(&beacondists[offset], &(dists[i].id), sizeof(int));
					offset += sizeof(int);
					memcpy(&beacondists[offset], &(dists[i].x), sizeof(double));
					offset += sizeof(double);
					memcpy(&beacondists[offset], &(dists[i].y), sizeof(double));
					offset += sizeof(double);
				}
				pozyxAlmanachRecv->writeDatagram(beacondists, dists.size() * (sizeof(double) + sizeof(double) + sizeof(int)), pozyxalmaaddress, pozyxalmaport);
			}
			first2 = second2;
		}
		std::this_thread::sleep_for(std::chrono::nanoseconds(100));
	}
}

void MainWindow::on_pushButton_clicked()
{
	QStringList pos = ui->lineEdit->text().split(" ");
	qDebug() << pos;

	QVector< double > vector;
    for ( const auto& item : pos )
    {
        bool ok = true;
        const double value = item.toDouble( &ok );
        if ( ok ) {
            vector << value;
        }
        // ... do something if the conversion failes.
    }
    // double randPositionX= mapload.minX + (float)(static_cast <float> (rand()) /( static_cast <float> (RAND_MAX)))*(mapload.maxX-mapload.minX);;
    // double randPositionY= mapload.minY + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(mapload.maxY-mapload.minY)));;
	qDebug() << vector;
    mapload.updateRandomHexagon(vector.first(),vector.last(),mapa);
    update();
}


void MainWindow::on_pushButton_2_clicked()
{
	sphere_x = mapload.minX + (float)(static_cast<float>(rand()) / (static_cast<float>(RAND_MAX))) * (mapload.maxX - mapload.minX);
	;
	sphere_y = mapload.minY + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (mapload.maxY - mapload.minY)));
	;
}


void MainWindow::on_pushButton_3_clicked()
{
	static bool fin = false;

	if (fin == true)
		return;
	fin = true;


	ui->groupBox->hide();
	ui->groupBox_2->hide();
	ui->groupBox_3->hide();
	ui->pushButton_3->hide();
	std::function<void(void)> f2 = std::bind(&MainWindow::laserSimulator, this);
	lasersimthreadHandle = std::move(std::thread(f2));
	std::function<void(void)> f = std::bind(&MainWindow::robotSimulator, this);
	robotsimthreadHandle = std::move(std::thread(f));
	std::function<void(void)> f3 = std::bind(&MainWindow::pozyxSimulator, this);
	pozyxsimthreadHandle = std::move(std::thread(f3));
	stopAll = 0;
	laserRecv = new QUdpSocket(this);
	auto lasret = laserRecv->bind(QHostAddress::Any, (quint16)ui->spinBox_3->value());

	connect(laserRecv, &QUdpSocket::readyRead, this, &MainWindow::readPendingDatagrams);

	robotRecv = new QUdpSocket(this);
	auto robret = robotRecv->bind(QHostAddress::Any, (quint16)ui->spinBox->value());


	connect(robotRecv, &QUdpSocket::readyRead, this, &MainWindow::readPendingRoboDatagrams);

	// std::cout << "mam spojenie " << lasret << " " << robret << std::endl;
	pozyxRecv = new QUdpSocket(this);
	pozyxRecv->bind(QHostAddress::Any, (quint16)(ui->spinBox->value() + 1));

	connect(pozyxRecv, &QUdpSocket::readyRead, this, &MainWindow::readPendingPozyxDatagrams);

	pozyxAlmanachRecv = new QUdpSocket(this);
	pozyxAlmanachRecv->bind(QHostAddress::Any, (quint16)(ui->spinBox->value() + 2));

	connect(pozyxAlmanachRecv, &QUdpSocket::readyRead, this, &MainWindow::readPendingPozyxAlmaDatagrams);

	connect(this, &MainWindow::imageReady, this, &MainWindow::sendImageToWeb);
	m_TcpHttpServer = new QTcpServer();

	m_TcpHttpServer->connect(m_TcpHttpServer, SIGNAL(newConnection()), this, SLOT(TcpHttpconnected()));

	m_TcpHttpServer->listen(QHostAddress::Any, ui->spinBox_5->value());
	/*  ready_promise.set_value();
    std::cout<<"skoncim??"<<std::endl;
    stopAll=1;
    lasersimthreadHandle.join();
    robotsimthreadHandle.join();
    pozyxsimthreadHandle.join();
    robotRecv->deleteLater();
    laserRecv->deleteLater();
    robotRecv->close();
    laserRecv->close();

    std::cout<<"aky je robot "<<robotRecv->isOpen()<<std::endl;
    delete robotRecv;
    delete laserRecv;
    if(m_TcpHttpClient!=NULL)
     m_TcpHttpClient->disconnect(); //Should never be Reached
    std::cout<<"koncim s ui"<<std::endl;

stopAll=0;
    ready_promise=std::promise<void>();
    readyFuture=ready_promise.get_future();
std::function<void(void)> f2 =std::bind(&MainWindow::laserSimulator,this);

    lasersimthreadHandle=std::move(std::thread(f2));
    std::function<void(void)> f =std::bind(&MainWindow::robotSimulator,this);
    robotsimthreadHandle=std::move(std::thread(f));
    std::function<void(void)> f3 =std::bind(&MainWindow::pozyxSimulator,this);
    pozyxsimthreadHandle=std::move(std::thread(f3));

    laserRecv = new QUdpSocket(this);
    bool laserret=laserRecv->bind(QHostAddress::Any,  ui->spinBox_3->value());

    connect(laserRecv, &QUdpSocket::readyRead,
            this, &MainWindow::readPendingDatagrams);

    robotRecv = new QUdpSocket(this);
   bool robotret= robotRecv->bind(QHostAddress::Any, ui->spinBox->value());

    connect(robotRecv, &QUdpSocket::readyRead,
            this, &MainWindow::readPendingRoboDatagrams);

    std::cout<<"mam otvorene "<<laserret<<" "<<robotret<<std::endl;;
    pozyxRecv = new QUdpSocket(this);
    pozyxRecv->bind(QHostAddress::Any, ui->spinBox->value()+1);

    connect(pozyxRecv, &QUdpSocket::readyRead,
            this, &MainWindow::readPendingPozyxDatagrams);

    pozyxAlmanachRecv = new QUdpSocket(this);
    pozyxAlmanachRecv->bind(QHostAddress::Any, ui->spinBox->value()+2);

    connect(pozyxAlmanachRecv, &QUdpSocket::readyRead,
            this, &MainWindow::readPendingPozyxAlmaDatagrams);

    connect(this,&MainWindow::imageReady,this,&MainWindow::sendImageToWeb);
    m_TcpHttpServer = new QTcpServer();

      m_TcpHttpServer->connect(m_TcpHttpServer,
                               SIGNAL(newConnection()),
                               this,
                               SLOT(TcpHttpconnected()));

      m_TcpHttpServer->listen(QHostAddress::Any,
                              ui->spinBox_5->value());


   // laserRecv->bind(QHostAddress::Any, ui->spinBox_3->value());
  //  robotRecv->bind(QHostAddress::Any, ui->spinBox->value());

   /* m_TcpHttpClient->disconnect();
    m_TcpHttpServer->connect(m_TcpHttpServer,
                             SIGNAL(newConnection()),
                             this,
                             SLOT(TcpHttpconnected()));

    m_TcpHttpServer->listen(QHostAddress::Any,
                            ui->spinBox_5->value());
    clientReady=false;*/
}
