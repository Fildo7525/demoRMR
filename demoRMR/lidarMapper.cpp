#include "lidarMapper.h"
#include <qpainter.h>

LidarMapper::LidarMapper(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
}

void LidarMapper::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.setBrush(Qt::black);

	QRect rect;
	rect = ui.frame->geometry(); //ziskate porametre stvorca,do ktoreho chcete kreslit
	rect.translate(0, 15);
	painter.drawRect(rect);

	QPen pero;
	pero.setStyle(Qt::SolidLine); //styl pera - plna ciara
	pero.setWidth(3);			  //hrubka pera -3pixely
	pero.setColor(Qt::white);	  //farba je zelena

	painter.setPen(pero);

	for (const auto &point : m_points) {
		if (rect.contains(point.x(), point.y())) {
			painter.drawEllipse(point, 2, 2);
		}
	}
}

void LidarMapper::on_pointCalculated_show(const QPointF &point)
{
	m_points.append(point);

	if (m_points.size() % 10 == 0) {
		update();
	}
}

