#include "lidarMapper.h"

#include <QDebug>
#include <QPainter>
#include <QThread>
#include <cmath>

#define POINT_RADIUS 5

LidarMapper::LidarMapper(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	m_rect = ui.frame->geometry(); //ziskate porametre stvorca,do ktoreho chcete kreslit
								   // m_rect.translate(0, 15);
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
		painter.drawEllipse(point, 2, 2);
	}
}

void LidarMapper::on_pointCloudCalculatedShow(QVector<QPointF> points)
{
	// m_points.append(points);
	for (const QPointF &point : points) {
		auto p = point + m_rect.center();
		if (isInsertable(p)) {
			m_points.push_back(p);
		}
	}

	// m_points = points;
	update();
}

bool LidarMapper::isInsertable(const QPointF &point)
{
	if (!m_rect.contains(point.x(), point.y())) {
		return false;
	}

	for (const auto &p : m_points) {
		if (std::abs(std::sqrt(std::pow(p.x() - point.x(), 2) + std::pow(p.y() - point.y(), 2))) < POINT_RADIUS) {
			return false;
		}
	}

	return true;
}
