#include "mainwindow.h"
#include "lidarMapper.h"
#include "ui_mainwindow.h"

#include <QDebug>
#include <QLineEdit>
#include <QObject>
#include <QPainter>
#include <QPoint>
#include <QPushButton>
#include <QThread>
#include <cmath>

static const QString IP_ADDRESSES[2] {"127.0.0.1", "192.168.1."};
///TOTO JE DEMO PROGRAM...AK SI HO NASIEL NA PC V LABAKU NEPREPISUJ NIC,ALE SKOPIRUJ SI MA NIEKAM DO INEHO FOLDERA
/// AK HO MAS Z GITU A ROBIS NA LABAKOVOM PC, TAK SI HO VLOZ DO FOLDERA KTORY JE JASNE ODLISITELNY OD TVOJICH KOLEGOV
/// NASLEDNE V POLOZKE Projects SKONTROLUJ CI JE VYPNUTY shadow build...
/// POTOM MIESTO TYCHTO PAR RIADKOV NAPIS SVOJE MENO ALEBO NEJAKY INY LUKRATIVNY IDENTIFIKATOR
/// KED SA NAJBLIZSIE PUSTIS DO PRACE, SKONTROLUJ CI JE MIESTO TOHTO TEXTU TVOJ IDENTIFIKATOR
/// AZ POTOM ZACNI ROBIT... AK TO NESPRAVIS, POJDU BODY DOLE... A NIE JEDEN,ALEBO DVA ALE BUDES RAD
/// AK SA DOSTANES NA SKUSKU

#define PI 3.14159
#define SHORT_MAX 65'535

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, ui(new Ui::MainWindow)
	, lastLeftEncoder(0)
	, lastRightEncoder(0)
	, m_fi(0)
	, m_x(0)
	, m_y(0)
	, m_xTarget(0)
	, m_yTarget(0)
	, m_controllerThread(new QThread(this))
	, m_plannerThread(new QThread(this))
	, m_robotStartupLocation(false)
{
	qDebug() << "MainWindow starting";
	//tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
	ipaddress = "127.0.0.1"; //192.168.1.12 toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
							 //  cap.open("http://192.168.1.11:8000/stream.mjpg");
	// ROBOT IP
	// ipaddress = "192.168.1.12";
	ui->setupUi(this);
	m_lidarMapper = new LidarMapper(this);

	ui->comboBox->addItem(IP_ADDRESSES[0]);
	for (size_t i = 11; i < 15; i++) {
		ui->comboBox->addItem(IP_ADDRESSES[1]+QString::number(i));
	}

	datacounter = 0;
	//  timer = new QTimer(this);
	//	connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
	actIndex = -1;
	useCamera1 = false;

	// Object for managing the robot speed interactions.
	m_trajectoryController = std::make_shared<RobotTrajectoryController>(&robot, this);

	m_floodPlanner = std::make_shared<FloodPlanner>("map.txt");

	// Creating all connections
	connect(this, &MainWindow::moveForward, m_trajectoryController.get(), &RobotTrajectoryController::onMoveForwardMove, Qt::QueuedConnection);
	connect(this, &MainWindow::changeRotation, m_trajectoryController.get(), &RobotTrajectoryController::onChangeRotationRotate, Qt::QueuedConnection);

	connect(this, &MainWindow::linResultsReady, m_trajectoryController.get(), &RobotTrajectoryController::handleLinResults, Qt::QueuedConnection);
	connect(this, &MainWindow::arcResultsReady, m_trajectoryController.get(), &RobotTrajectoryController::handleArcResults, Qt::QueuedConnection);

	connect(this, &MainWindow::lidarDataReady, m_trajectoryController.get(), &RobotTrajectoryController::on_lidarDataReady_map, Qt::QueuedConnection);
	connect(this, &MainWindow::requestPath, m_floodPlanner.get(), &FloodPlanner::on_requestPath_plan, Qt::QueuedConnection);
	connect(m_floodPlanner.get(), &FloodPlanner::pathPlanned, this, &MainWindow::handlePath, Qt::QueuedConnection);

	connect(ui->linSubmitTargetButton, &QPushButton::clicked, this, &MainWindow::onLinSubmitButtonClicked, Qt::QueuedConnection);
	connect(ui->arcSubmitTargetButton, &QPushButton::clicked, this, &MainWindow::onArcSubmitButtonClicked, Qt::QueuedConnection);

	// Starting threads
	m_controllerThread->start();
	m_plannerThread->start();

	// std::cout << __FUNCTION__ << " " << std::this_thread::get_id() << std::endl;

	m_trajectoryController->moveToThread(m_controllerThread);
	m_floodPlanner->moveToThread(m_plannerThread);
}

MainWindow::~MainWindow()
{
	m_controllerThread->exit(0);
	m_plannerThread->exit(0);
	delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	///prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
	/// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
	painter.setBrush(Qt::black); //cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
	QPen pero;
	pero.setStyle(Qt::SolidLine); //styl pera - plna ciara
	pero.setWidth(3);			  //hrubka pera -3pixely
	pero.setColor(Qt::green);	  //farba je zelena
	QRect rect;
	rect = ui->frame->geometry(); //ziskate porametre stvorca,do ktoreho chcete kreslit
	rect.translate(0, 15);
	painter.drawRect(rect);

	if (useCamera1 == true && actIndex > -1) /// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
	{
		// std::cout << actIndex << std::endl;
		QImage image = QImage((uchar *)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step,
							  QImage::Format_RGB888); //kopirovanie cvmat do qimage
		painter.drawImage(rect, image.rgbSwapped());
	}
	else {
		if (updateLaserPicture == 1) ///ak mam nove data z lidaru
		{
			updateLaserPicture = 0;

			painter.setPen(pero);
			//teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
			//   std::cout<<copyOfLaserData.numberOfScans<<std::endl;
			for (int k = 0; k < copyOfLaserData.numberOfScans /*360*/; k++) {
				int dist = copyOfLaserData.Data[k].scanDistance / 20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
				int xp = rect.width() - (rect.width() / 2 + dist * 2 * sin((360.0 - copyOfLaserData.Data[k].scanAngle) * 3.14159 / 180.0))
					+ rect.topLeft().x(); //prepocet do obrazovky
				int yp = rect.height() - (rect.height() / 2 + dist * 2 * cos((360.0 - copyOfLaserData.Data[k].scanAngle) * 3.14159 / 180.0))
					+ rect.topLeft().y();  //prepocet do obrazovky
				if (rect.contains(xp, yp)) //ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
					painter.drawEllipse(QPoint(xp, yp), 2, 2);
			}
		}
	}
}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{
	memcpy(&copyOfLaserData, &laserData, sizeof(LaserMeasurement));
	//tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
	// ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
	updateLaserPicture = 1;
	update(); //tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

	// qDebug() << "m_x: " << m_x << " m_y: " << m_y << " m_fi: " << m_fi;
	emit lidarDataReady(std::move(copyOfLaserData));

	return 0;
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{
	calculateOdometry(robotdata);

	///tu mozete robit s datami z robota
	/// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
	///teraz tu posielam rychlosti na zaklade toho co setne joystick a vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
	/// tuto joystick cast mozete vklude vymazat,alebo znasilnit na vas regulator alebo ake mate pohnutky... kazdopadne, aktualne to blokuje gombiky cize tak
	// if(instance->count()>0)
	// {
	// 	if(forwardspeed==0 && rotationspeed!=0)
	// 		robot.setRotationSpeed(rotationspeed);
	// 	else if(forwardspeed!=0 && rotationspeed==0)
	// 		robot.setTranslationSpeed(forwardspeed);
	// 	else if((forwardspeed!=0 && rotationspeed!=0))
	// 		robot.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
	// 	else
	// 		robot.setTranslationSpeed(0);
	//
	// }
	///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

	if (datacounter % 5) {
		///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
		// ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
		//ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
		//ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
		/// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
		/// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
		/// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
		///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
		emit uiValuesChanged(m_x, m_y, m_fi);
		///toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
		/// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow.ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
		/// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde
	}
	datacounter++;

	if (m_trajectoryController->isInAutoMode())
	{
		m_trajectoryController->obstacleAvoidanceTrajectoryHandle(copyOfLaserData,m_x,m_y,m_fi);
	}

	return 0;
}

/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void MainWindow::setUiValues(double robotX, double robotY, double robotFi)
{
	ui->lineEdit_2->setText(QString::number(robotX));
	ui->lineEdit_3->setText(QString::number(robotY));
	ui->lineEdit_4->setText(QString::number(robotFi));
}

///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z kamery
int MainWindow::processThisCamera(cv::Mat cameraData)
{
	cameraData.copyTo(frame[(actIndex + 1) % 3]); //kopirujem do nasej strukury
	actIndex = (actIndex + 1) % 3;				  //aktualizujem kde je nova fotka
	updateLaserPicture = 1;
	return 0;
}

void MainWindow::calculateOdometry(const TKobukiData &robotdata)
{
	int diffLeftEnc = robotdata.EncoderLeft - lastLeftEncoder;
	int diffRightEnc = robotdata.EncoderRight - lastRightEncoder;

	if (lastRightEncoder > 60'000 && robotdata.EncoderRight < 1'000)
		diffRightEnc += SHORT_MAX;
	if (lastLeftEncoder > 60'000 && robotdata.EncoderLeft < 1'000)
		diffLeftEnc += SHORT_MAX;

	if (lastRightEncoder < 1'000 && robotdata.EncoderRight > 60'000)
		diffRightEnc -= SHORT_MAX;
	if (lastLeftEncoder < 1'000 && robotdata.EncoderLeft > 60'000)
		diffLeftEnc -= SHORT_MAX;

	auto leftEncDist = robot.tickToMeter * diffLeftEnc;
	auto rightEncDist = robot.tickToMeter * diffRightEnc;

	lastLeftEncoder = robotdata.EncoderLeft;
	lastRightEncoder = robotdata.EncoderRight;

	double l = (rightEncDist + leftEncDist) / 2.0;
	{
		m_fi = robotdata.GyroAngle / 100. * TO_RADIANS - m_fiCorrection;
		m_x = m_x + l * std::cos(m_fi);
		m_y = m_y + l * std::sin(m_fi);

		if (!m_robotStartupLocation && datacounter % 5) {
			m_x = 0;
			m_y = 0;
			m_fiCorrection = m_fi;
			m_robotStartupLocation = true;
		}
	}
}

static QVector<QPointF> generateSequence(const QPointF &min, const QPointF &max, const QPointF &line)
{
	QVector<QPointF> ret;

	const double h = std::hypot(max.x() - min.x(), max.y() - min.y());
	const double dif = (max.x() - min.x()) / h;

	double var = min.x() + dif;
	while (var < max.x()-1) {
		ret.push_back({var, line.x() * var + line.y()});
		var += dif;
	}

	ret.push_back(max);

	return ret;
}

void MainWindow::_calculateTrajectory(RobotTrajectoryController::MovementType type)
{
	auto [distance, angle] = calculateTrajectory();

	QPointF min(m_x, m_y);
	QPointF max(m_xTarget, m_yTarget);
	QPointF line = computeLineParameters(min, max);

	QVector<QPointF> points = generateSequence(min, max, line);

	// qDebug() << "Points: " << points;
	if (type == RobotTrajectoryController::MovementType::Forward) {
		emit linResultsReady(distance, angle, points);
	}
	else if (type == RobotTrajectoryController::MovementType::Arc) {
		emit arcResultsReady(distance, angle, points);
	}
}

QPair<double, double> MainWindow::calculateTrajectory()
{
	// Get current position and orientation (actual values)
	auto [distanceToTarget, angleToTarget] = calculateTrajectoryTo({ m_xTarget, m_yTarget });

	ui->distanceToTarget->setText(QString::number(distanceToTarget));
	ui->angleToTarget->setText(QString::number(angleToTarget));

	return { distanceToTarget, angleToTarget };
}

QPair<double, double> MainWindow::calculateTrajectoryTo(QPair<double, double> point)
{
	// Get current position and orientation (actual values)
	double current_x = 0;
	double current_y = 0;

	{
		m_mutex.lock();
		current_x = m_x;
		current_y = m_y;
		m_mutex.unlock();
	}

	// Calculate angle to target
	double angleToTarget = std::atan2(point.second - current_y, point.first - current_x);
	double distanceToTarget = std::sqrt(std::pow(point.second - current_y, 2) + std::pow(point.first - current_x, 2));

	return { distanceToTarget, angleToTarget };
}

double MainWindow::finalRotationError()
{
	auto [dir, rot] = calculateTrajectory();
	double diff, fi;
	{
		m_mutex.lock();
		fi = m_fi;
		m_mutex.unlock();
	}

	if (fi > PI / 2 && rot < -PI / 2) {
		fi -= 2 * PI;
	}
	if (fi < -PI / 2 && rot > PI / 2) {
		fi += 2 * PI;
	}

	diff = fi - rot;

	return -diff;
}

double MainWindow::localRotationError(QPair<double, double> point)
{
	auto [dir, rot] = calculateTrajectoryTo(point);
	double diff, fi;
	{
		m_mutex.lock();
		fi = m_fi;
		m_mutex.unlock();
	}

	diff = fi - rot;

	if (diff > PI) {
		diff -= 2 * PI;
	}
	if (diff < -PI) {
		diff += 2 * PI;
	}

	return -diff;
}

void MainWindow::on_pushButton_8_clicked()
{
	// Stop all the robot's movement. This will stop the position controller.
	emit moveForward(0);

	m_x = 0;
	m_y = 0;
}

void MainWindow::on_pushButton_9_clicked() //start button
{
	ipaddress = ui->comboBox->currentText().toStdString();

	//ziskanie joystickov
	forwardspeed = 0;
	rotationspeed = 0;
	//tu sa nastartuju vlakna ktore citaju data z lidaru a robota
	connect(this, SIGNAL(uiValuesChanged(double, double, double)), this, SLOT(setUiValues(double, double, double)));

	///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
	/// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
	robot.setLaserParameters( ipaddress, 52999, 5299,
							/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/
							std::bind(&MainWindow::processThisLidar, this, std::placeholders::_1));
	robot.setRobotParameters(ipaddress, 53000, 5300, std::bind(&MainWindow::processThisRobot, this, std::placeholders::_1));
	//---simulator ma port 8889, realny robot 8000
	if (ipaddress == "127.0.0.1") {
		robot.setCameraParameters("http://" + ipaddress + ":8889/stream.mjpg", std::bind(&MainWindow::processThisCamera, this, std::placeholders::_1));
	}
	else {
		robot.setCameraParameters("http://" + ipaddress + ":8000/stream.mjpg", std::bind(&MainWindow::processThisCamera, this, std::placeholders::_1));
	}

	///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
	robot.robotStart();



}

void MainWindow::on_pushButton_2_clicked() //forward
{
	//pohyb dopredu
	emit moveForward(500);
}

void MainWindow::on_pushButton_3_clicked() //back
{
	emit moveForward(-250);
}

void MainWindow::on_pushButton_6_clicked() //left
{
	emit changeRotation(3.14159 / 2);
}

void MainWindow::on_pushButton_5_clicked() //right
{
	emit changeRotation(-3.14159 / 2);
}

void MainWindow::on_pushButton_4_clicked() //stop
{
	emit moveForward(0);
}

void MainWindow::on_pushButton_clicked()
{
	if (useCamera1 == true) {
		useCamera1 = false;
		ui->pushButton->setText("use camera");
	}
	else {
		useCamera1 = true;
		ui->pushButton->setText("use laser");
	}
}

void MainWindow::on_showMapButton_clicked()
{
	disconnect(m_connection);

	m_lidarMapper = new LidarMapper(this);
	m_lidarMapper->setWindowFlags(Qt::Window);
	m_lidarMapper->show();


	m_connection = connect(m_trajectoryController.get(), &RobotTrajectoryController::pointCloudCaluculated,
			m_lidarMapper, &LidarMapper::on_pointCloudCalculatedShow, Qt::QueuedConnection);

}

void MainWindow::on_startScanButton_clicked()
{
// Start:
// XX.00 | YY.YY
// ============
// 00.00 |  00.00
// 00.10 |  03.00
// 04.00 |  04.00
// 03.00 |  04.00
// 03.00 |  00.70
// 05.00 |  00.60
// 05.00 |  01.50
// 02.60 | -00.80
// 01.50 | -01.2
// End
	m_xTarget = 1.5;
	m_yTarget = -1.2;
	QVector<QPointF> points = {
		{0.1, 3},
		{4, 4},
		{3, 4},
		{3, 0.7},
		{5, 0.6},
		{5, 1.5},
		{5, 1},
		{3, 0.7},
		{2.6, -0.8},
		{1.5, -1.2}
	};
	auto [distance, angle] = calculateTrajectoryTo({ m_xTarget, m_yTarget });
	emit arcResultsReady(distance, angle, points);
}

void MainWindow::on_pathPlannerButton_clicked()
{
	bool ok1 = updateTarget(ui->targetXLine, m_xTarget);
	bool ok2 = updateTarget(ui->targetYLine, m_yTarget);
	if (!ok1 || !ok2) {
		return;
	}

	QPoint start(m_x, m_y);
	QPoint end(m_xTarget, m_yTarget);

	emit requestPath(start, end);
}

void MainWindow::timeout()
{
	m_trajectoryController->setTranslationSpeed(0, true);
}

bool MainWindow::updateTarget(QLineEdit *lineEdit, double &target)
{
	bool ok = true;
	double tmp = lineEdit->text().toDouble(&ok);
	if (!ok) {
		lineEdit->setText(QString::number(target));
		return false;
	}

	target = tmp;
	return true;
}

void MainWindow::onLinSubmitButtonClicked(bool clicked)
{
	bool ok1 = updateTarget(ui->targetXLine, m_xTarget);
	bool ok2 = updateTarget(ui->targetYLine, m_yTarget);

	if (ok1 && ok2) {
		_calculateTrajectory(RobotTrajectoryController::MovementType::Forward);
	}
}

void MainWindow::onArcSubmitButtonClicked(bool clicked)
{
	bool ok1 = updateTarget(ui->targetXLine, m_xTarget);
	bool ok2 = updateTarget(ui->targetYLine, m_yTarget);

	if (ok1 && ok2) {
		// TODO: firstly rotat the robot so that the robot is facing the target in range <-PI/4, PI/4>.
		_calculateTrajectory(RobotTrajectoryController::MovementType::Arc);
	}
}

void MainWindow::handlePath(QVector<QPointF> path)
{
	path.push_back(QPointF(m_xTarget, m_yTarget));
	auto [distance, angle] = calculateTrajectoryTo({ m_xTarget, m_yTarget });

	emit arcResultsReady(distance, angle, path);
}

void MainWindow::on_liveAvoidObstaclesButton_clicked()
{
	bool ok1 = updateTarget(ui->targetXLine, m_xTarget);
	bool ok2 = updateTarget(ui->targetYLine, m_yTarget);

	if (ok1 && ok2) {
		m_trajectoryController->obstacleAvoidanceTrajectoryInit(m_xTarget,m_yTarget,m_x,m_y,m_fi);
	}
}

