#include "lidarMapper.h"
#include "mainwindow.h"

#include <QDebug>
#include <QPainter>
#include <QThread>
#include <qglobal.h>

LidarMapper::LidarMapper(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	m_rect = ui.frame->geometry(); //ziskate porametre stvorca,do ktoreho chcete kreslit
	m_rect.translate(0, 15);
}

void LidarMapper::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.setBrush(Qt::black);

	painter.drawRect(m_rect);

	QPen pero;
	pero.setStyle(Qt::SolidLine); //styl pera - plna ciara
	pero.setWidth(2);			  //hrubka pera -3pixely
	pero.setColor(Qt::white);	  //farba je zelena

	painter.setPen(pero);

	for (const QPointF &point : m_points) {
		if (m_rect.contains(point.x(), point.y())) {
			QPointF p = point + m_rect.center();
			painter.drawEllipse(p, 2, 2);
		}
	}
}

void LidarMapper::on_pointCloudCalculatedShow(QVector<QPointF> points)
{
	m_points.append(points);
	// m_points = points;
	update();
}

