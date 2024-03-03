#ifndef STYLESHEETEDITOR_H
#define STYLESHEETEDITOR_H

#include <QDialog>
#include <qpoint.h>

#include "ui_lidarMapper.h"

class LidarMapper : public QDialog
{
	Q_OBJECT

public:
	LidarMapper(QWidget *parent = nullptr);

private:
	void paintEvent(QPaintEvent *event); // Q_DECL_OVERRIDE;

public slots:
	void on_pointCloudCalculated_show(const QVector<QPointF> &point);

private:
	Ui::LidarMapper ui;
	QVector<QPointF> m_points;
	QRect m_rect;
};

#endif
