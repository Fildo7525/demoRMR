#include "mainwindow.h"
#include "pidcontroller.h"
#include "qlineedit.h"
#include "qnamespace.h"
#include "qobject.h"
#include "qpushbutton.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <cmath>
#include <math.h>
#include <QThread>
/// TOTO JE DEMO PROGRAM...AK SI HO NASIEL NA PC V LABAKU NEPREPISUJ NIC,ALE SKOPIRUJ SI MA NIEKAM DO INEHO FOLDERA
/// AK HO MAS Z GITU A ROBIS NA LABAKOVOM PC, TAK SI HO VLOZ DO FOLDERA KTORY JE JASNE ODLISITELNY OD TVOJICH KOLEGOV
/// NASLEDNE V POLOZKE Projects SKONTROLUJ CI JE VYPNUTY shadow build...
/// POTOM MIESTO TYCHTO PAR RIADKOV NAPIS SVOJE MENO ALEBO NEJAKY INY LUKRATIVNY IDENTIFIKATOR
/// KED SA NAJBLIZSIE PUSTIS DO PRACE, SKONTROLUJ CI JE MIESTO TOHTO TEXTU TVOJ IDENTIFIKATOR
/// AZ POTOM ZACNI ROBIT... AK TO NESPRAVIS, POJDU BODY DOLE... A NIE JEDEN,ALEBO DVA ALE BUDES RAD
/// AK SA DOSTANES NA SKUSKU

#define PI 3.14159
#define TO_RADIANS PI/180.07
#define SHORT_MAX 65'535

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
	, ui(new Ui::MainWindow)
	, lastLeftEncoder(0)
	, lastRightEncoder(0)
	, m_fi(0)
	, m_x(0)
	, m_y(0)
	, m_xControl(1, 0, 0)
	, m_yControl(1, 0, 0)
	, m_timer(this)
	, m_trajectoryTimer(this)
	, m_trajectoryThread(new QThread(this))
{
	// tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
	ipaddress="127.0.0.1"; // 192.168.1.11 127.0.0.1
	// cap.open("http://192.168.1.11:8000/stream.mjpg");
	ui->setupUi(this);
	datacounter=0;

	connect(&m_timer, SIGNAL(timeout()), this, SLOT(timeout()));

	m_timer.setTimerType(Qt::TimerType::PreciseTimer);
	m_timer.setSingleShot(true);
	m_timer.setInterval(3'000);
	m_timer.start();

	m_trajectoryTimer.setTimerType(Qt::TimerType::PreciseTimer);
	m_timer.setSingleShot(false);
	m_trajectoryTimer.setInterval(500);
	connect(&m_trajectoryTimer, &QTimer::timeout, this, &MainWindow::calculateTrajectory);

	m_trajectoryTimer.moveToThread(m_trajectoryThread);
	m_trajectoryTimer.start();
	m_trajectoryThread->start();

	connect(ui->submitTargetButton, &QPushButton::clicked, this, &MainWindow::onSubmitButtonClicked);

	datacounter=0;
}

MainWindow::~MainWindow()
{
	delete ui;
}

/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
	ui->lineEdit_2->setText(QString::number(robotX));
	ui->lineEdit_3->setText(QString::number(robotY));
	ui->lineEdit_4->setText(QString::number(robotFi));
}

/// toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{
	/// TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE,
	/// SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

	calculateOdometry(robotdata);

	// calculateTrajectory(robotdata);

	/// tu mozete robit s datami z robota
	/// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
	/// teraz tu posielam rychlosti na zaklade toho co setne joystick a vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
	/// tuto cast mozete vklude vymazat,alebo znasilnit na vas regulator alebo ake mate pohnutky
	// if(forwardspeed==0 && rotationspeed!=0)
	// 	robot.setRotationSpeed(rotationspeed);
	// else if(forwardspeed!=0 && rotationspeed==0)
	// 	robot.setTranslationSpeed(forwardspeed);
	// else if((forwardspeed!=0 && rotationspeed!=0))
	// 	robot.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
	// else
	// 	robot.setTranslationSpeed(0);

	if(datacounter%5==0) {
		/// ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
		// ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
		// ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
		// ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
		/// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
		/// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
		/// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
		/// posielame sem nezmysli. pohrajte sa nech sem idu zmysluplne veci
		emit uiValuesChanged(m_x, m_y, m_fi);
		/// toto neodporucam na nejake komplikovane struktury.signal slot robi kopiu dat. radsej vtedy posielajte
		/// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow
		/// ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
		/// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde
	}
	datacounter++;

	return 0;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	// prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
	// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
	painter.setBrush(Qt::black); // cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
	QPen pero;
	pero.setStyle(Qt::SolidLine); // styl pera - plna ciara
	pero.setWidth(3); // hrubka pera -3pixely
	pero.setColor(Qt::green); // farba je zelena
	QRect rect;
	rect = ui->frame->geometry(); // ziskate porametre stvorca,do ktoreho chcete kreslit
	rect.translate(0, 15);
	painter.drawRect(rect);

	// ak mam nove data z lidaru
	if(updateLaserPicture == 1) {
		updateLaserPicture = 0;

		painter.setPen(pero);
		// teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
		// std::cout<<copyOfLaserData.numberOfScans<<std::endl;
		for(int k = 0; k < copyOfLaserData.numberOfScans/*360*/; k++) {
			// vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
			int dist = copyOfLaserData.Data[k].scanDistance/20;
			// prepocet do obrazovky
			int xp = rect.width() - ( rect.width() / 2
									 + dist * 2 * sin((360.0 - copyOfLaserData.Data[k].scanAngle) * TO_RADIANS))
					 + rect.topLeft().x();
			// prepocet do obrazovky
			int yp = rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*TO_RADIANS))+rect.topLeft().y();

			// ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
			if(rect.contains(xp,yp))
				painter.drawEllipse(QPoint(xp, yp),2,2);
		}
	}
}

void MainWindow::calculateOdometry(const TKobukiData &robotdata)
{
	int diffLeftEnc = robotdata.EncoderLeft - lastLeftEncoder;
	int diffRightEnc = robotdata.EncoderRight - lastRightEncoder;

	ui->timestampLineEdit->setText(QString::number(robotdata.timestamp));

	if ( lastRightEncoder > 60'000 && robotdata.EncoderRight < 1'000)
		diffRightEnc += SHORT_MAX;
	if (lastLeftEncoder > 60'000 && robotdata.EncoderLeft < 1'000)
		diffLeftEnc += SHORT_MAX;

	if (diffLeftEnc != 0) {
		std::cout << "encL: " << robotdata.EncoderLeft << " encR: " << robotdata.EncoderRight << std::endl;
		std::cout<< "deL: " << diffLeftEnc << " deR: " << diffRightEnc << std::endl;
	}

	auto leftEncDist = robot.tickToMeter * diffLeftEnc;
	auto rightEncDist = robot.tickToMeter * diffRightEnc;

	lastLeftEncoder = robotdata.EncoderLeft;
	lastRightEncoder = robotdata.EncoderRight;

	double l = (rightEncDist + leftEncDist) / 2.0;
	m_fi = robotdata.GyroAngle / 100. * TO_RADIANS;

	if (leftEncDist != 0 || rightEncDist != 0)
		std::cout << "leftEncDist: " << leftEncDist << " rightEncDist: " << rightEncDist << std::endl;

	m_x += l * std::cos(m_fi);
	m_y += l * std::sin(m_fi);
}

void MainWindow::calculateTrajectory()
{
	// Get current position and orientation (actual values)
	double current_x = m_x;
	double current_y = m_y;
	double current_theta = m_fi;

	// Calculate PID control commands
	double dx = m_xControl.compute(current_x);
	double dy = m_yControl.compute(current_y);

	// Calculate angle to target
	double angleToTarget = std::atan2(m_yControl.target() - current_y, m_xControl.target() - current_x);
	double distanceToTarget = std::sqrt(
		std::pow( m_yControl.target() - current_y + m_xControl.target() - current_x,
		2)
	);

	ui->distanceToTarget->setText(QString::number(distanceToTarget));
	ui->angleToTarget->setText(QString::number(angleToTarget));
	// Calculate angular velocity
	// double angular_velocity = angle_to_target - current_theta;

	robot.setRotationSpeed(angleToTarget);
	robot.setTranslationSpeed(PI/3);

	//std::cout << "Angle to target: " << angle_to_target << " angular velocity: " << angular_velocity << "                                                         \r" << std::ends;

	// Check for obstacles using lidar data
	// std::vector<double> distances(copyOfLaserData.numberOfScans);
	// std::move(copyOfLaserData.Data[0], copyOfLaserData.Data[distances.size()], distances.begin());

	// bool obstacle_detected = std::any_of(distances.begin(), distances.end(), [](double d) { return d < 0.2; });
	// if (obstacle_detected)
	// 	std::cout << "Obstacle detected                  \r" << std::ends;
	// else
	// 	std::cout << "No Obstacle detected                  \r" << std::ends;

	// Adjust linear velocity if obstacle detected
	// if (obstacle_detected) {
	// 	linear_velocity = 0;
	// } else {
	// 	linear_velocity = 0.5; // Set linear velocity
	// }

	// // Move robot
	// move(linear_velocity, angular_velocity);
}

/// toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{
	memcpy(&copyOfLaserData, &laserData, sizeof(LaserMeasurement));
	// tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
	// ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
	updateLaserPicture = 1;
	// tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia
	update();

	return 0;
}

void MainWindow::on_pushButton_9_clicked() // start button
{
	forwardspeed = 0;
	rotationspeed = 0;
	// tu sa nastartuju vlakna ktore citaju data z lidaru a robota
	connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));

	/// setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.
	/// laser ma ze sa da dat callback aj ako lambda. lambdy su super, setria miesto a ak su rozumnej dlzky,
	/// tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
	robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/
							 std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
	robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));

	/// ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
	robot.robotStart();
}

void MainWindow::on_pushButton_2_clicked() // forward
{
	robot.setTranslationSpeed(500);
	m_timer.start();
}

void MainWindow::on_pushButton_3_clicked() // back
{
	robot.setTranslationSpeed(-250);
	m_timer.start();
}

void MainWindow::on_pushButton_6_clicked() // left
{
	robot.setRotationSpeed(3.14159/2);
	m_timer.start();
}

void MainWindow::on_pushButton_5_clicked() // right
{
	robot.setRotationSpeed(-3.14159/2);
	m_timer.start();
}

void MainWindow::on_pushButton_4_clicked() // stop
{
	robot.setTranslationSpeed(0);
}

void MainWindow::on_pushButton_clicked()
{
	ui->pushButton->setText("use laser");
}

void MainWindow::timeout()
{
	robot.setTranslationSpeed(0);
	m_timer.stop();
}

bool MainWindow::updateTarget(QLineEdit *lineEdit, PIDController *controller)
{
	bool ok = true;
	double target = lineEdit->text().toDouble(&ok);
	if (!ok) {
		lineEdit->setText(QString::number(controller->target()));
		return false;
	}

	controller->setTarget(target);
	return true;
}

void MainWindow::onSubmitButtonClicked(bool clicked)
{
	updateTarget(ui->targetXLine, &m_xControl);
	updateTarget(ui->targetYLine, &m_yControl);
}
