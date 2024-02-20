#include "mainwindow.h"
#include "qdebug.h"
#include "qlineedit.h"
#include "qnamespace.h"
#include "qobject.h"
#include "qpoint.h"
#include "qpushbutton.h"
#include "robotTrajectoryController.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <cmath>
#include <math.h>
#include <QThread>
#include <QDebug>
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
	, m_xTarget(0)
	, m_yTarget(0)

	, m_trajectoryThread(new QThread(this))
	, m_controllerThread(new QThread(this))
{
	// tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
	ipaddress="127.0.0.1"; // 192.168.1.11 127.0.0.1
	// cap.open("http://192.168.1.11:8000/stream.mjpg");
	ui->setupUi(this);
	datacounter=0;

	// Object for managing the robot speed interactions.
	m_trajectoryController = new RobotTrajectoryController(&robot, this);

    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress="127.0.0.1";//192.168.1.11toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
  //  cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter=0;
  //  timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));




    datacounter=0;


	// Starting threads
	m_trajectoryThread->start();
	m_controllerThread->start();

	std::cout << __FUNCTION__ << " " << std::this_thread::get_id() << std::endl;

	m_trajectoryController->moveToThread(m_controllerThread);
}

MainWindow::~MainWindow()
{
	m_controllerThread->exit(0);
	m_trajectoryThread->exit(0);
	delete ui;
	delete m_trajectoryController;
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
	// 	m_trajectoryController->setRotationSpeed(rotationspeed);
	// else if(forwardspeed!=0 && rotationspeed==0)
	// 	m_trajectoryController->setTranslationSpeed(forwardspeed);
	// else if((forwardspeed!=0 && rotationspeed!=0))
	// 	robot.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
	// else
	// 	m_trajectoryController->setTranslationSpeed(0);

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

    if(updateLaserPicture==1) ///ak mam nove data z lidaru
    {
        std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );//kopirovanie cvmat do qimage
        painter.drawImage(rect,image.rgbSwapped());
    }
    else
    {
        if(updateLaserPicture==1) ///ak mam nove data z lidaru
        {
            int dist=copyOfLaserData.Data[k].scanDistance/20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
            int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x(); //prepocet do obrazovky
            int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();//prepocet do obrazovky
            if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                painter.drawEllipse(QPoint(xp, yp),2,2);
        }
    }

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

	if (lastRightEncoder > 60'000 && robotdata.EncoderRight < 1'000)
		diffRightEnc += SHORT_MAX;
	if (lastLeftEncoder > 60'000 && robotdata.EncoderLeft < 1'000)
		diffLeftEnc += SHORT_MAX;

    ///tu mozete robit s datami z robota
    /// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
    ///teraz tu posielam rychlosti na zaklade toho co setne joystick a vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
    /// tuto joystick cast mozete vklude vymazat,alebo znasilnit na vas regulator alebo ake mate pohnutky... kazdopadne, aktualne to blokuje gombiky cize tak
    if(instance->count()>0)
    {
        if(forwardspeed==0 && rotationspeed!=0)
            robot.setRotationSpeed(rotationspeed);
        else if(forwardspeed!=0 && rotationspeed==0)
            robot.setTranslationSpeed(forwardspeed);
        else if((forwardspeed!=0 && rotationspeed!=0))
            robot.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
        else
            robot.setTranslationSpeed(0);

    }
///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

	lastLeftEncoder = robotdata.EncoderLeft;
	lastRightEncoder = robotdata.EncoderRight;

	double l = (rightEncDist + leftEncDist) / 2.0;
	{
		m_mutex.lock();
		m_fi = robotdata.GyroAngle / 100. * TO_RADIANS;
		m_x += l * std::cos(m_fi);
		m_y += l * std::sin(m_fi);
		m_mutex.unlock();
	}
}

static QVector<double> generateSequence(double min, double max)
{
	QVector<double> ret;

	for (int var = min+1; var < (int)max; var+=2) {
		ret.push_back(var);
	}
	ret.push_back(max);

	return ret;
}

static QPointF computeLineParameters(QPointF p1, QPointF p2) {
	QPointF line;
	// Compute slope (a)
	if (p1.x() != p2.x()) {
		auto x = (p2.y() - p1.y()) / (p2.x() - p1.x());
		line.setX(x);
	} else {
		// If the line is vertical, slope is infinity, so set a to a large value
		line.setY(1e9);
	}
	// Compute intercept (b)
	line.ry() = p1.y() - line.x() * p1.x();
	return line;
}

void MainWindow::_calculateTrajectory()
{
	auto [distance, angle] = calculateTrajectory();

	QPointF line = computeLineParameters({m_x, m_y}, {m_xTarget, m_yTarget});
	qDebug() << "Line: " << line;

	QVector<double> X = generateSequence(m_x, m_xTarget);
	QVector<QPointF> points;

	for (const double &var : X) {
		points.push_back({var, line.x() * var + line.y()});
	}

	qDebug() << "Points: " << points;
	emit resultsReady(distance, angle, points);
}

QPair<double, double> MainWindow::calculateTrajectory()
{
	// Get current position and orientation (actual values)
	auto [distanceToTarget, angleToTarget] = calculateTrajectoryTo({m_xTarget, m_yTarget});

	ui->distanceToTarget->setText(QString::number(distanceToTarget));
	ui->angleToTarget->setText(QString::number(angleToTarget));

	return {distanceToTarget, angleToTarget};
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
	double distanceToTarget = std::sqrt(
		std::pow(point.second - current_y + point.first - current_x,
				 2)
		);

	return {distanceToTarget, angleToTarget};
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

	if (fi > PI/2 && rot < -PI/2) {
		fi -= 2*PI;
	}
	if (fi < -PI/2 && rot > PI/2) {
		fi += 2*PI;
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

	if (fi > PI/2 && rot < -PI/2) {
		fi -= 2*PI;
	}
	if (fi < -PI/2 && rot > PI/2) {
		fi += 2*PI;
	}

	diff = fi - rot;

	return -diff;
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

///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z kamery
int MainWindow::processThisCamera(cv::Mat cameraData)
{
	forwardspeed = 0;
	rotationspeed = 0;
	// tu sa nastartuju vlakna ktore citaju data z lidaru a robota
	connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka
    updateLaserPicture=1;
    return 0;
}
void MainWindow::on_pushButton_9_clicked() //start button
{
    //ziskanie joystickov
    instance = QJoysticks::getInstance();
    forwardspeed=0;
    rotationspeed=0;
    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));
    //---simulator ma port 8889, realny robot 8000
    robot.setCameraParameters("http://"+ipaddress+":8889/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));

    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robot.robotStart();






    /// prepojenie joysticku s jeho callbackom... zas cez lambdu. neviem ci som to niekde spominal,ale lambdy su super. okrem toho mam este rad ternarne operatory a spolocneske hry ale to tiez nikoho nezaujima
    /// co vas vlastne zaujima? citanie komentov asi nie, inak by ste citali toto a ze tu je blbosti
    connect(
        instance, &QJoysticks::axisChanged,
        [this]( const int js, const int axis, const qreal value) { if(/*js==0 &&*/ axis==1){forwardspeed=-value*300;}
            if(/*js==0 &&*/ axis==0){rotationspeed=-value*(3.14159/2.0);}}
    );
}

void MainWindow::on_pushButton_2_clicked() // forward
{
	emit moveForward(500);
}

void MainWindow::on_pushButton_3_clicked() // back
{
	emit moveForward(-250);
}

void MainWindow::on_pushButton_6_clicked() // left
{
	emit changeRotation(3.14159/2);
}

void MainWindow::on_pushButton_5_clicked() // right
{
	emit changeRotation(-3.14159/2);
}

void MainWindow::on_pushButton_4_clicked() // stop
{
	emit moveForward(0);
}

void MainWindow::on_pushButton_clicked()
{
	ui->pushButton->setText("use laser");
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

void MainWindow::onSubmitButtonClicked(bool clicked)
{
	bool ok1 = updateTarget(ui->targetXLine, m_xTarget);
	bool ok2 = updateTarget(ui->targetYLine, m_yTarget);

	if (ok1 && ok2) {
		_calculateTrajectory();
	}
}
