#pragma once

#include <optional>
#include <memory>

#include <QObject>
#include <QPoint>
#include <QString>
#include <QVector>

class FloodPlanner
	: public QObject
{
	using Map = QVector<QVector<int>>;
	friend std::ostream &operator<<(std::ostream &os, const Map &map);

public:
	explicit FloodPlanner(const QString &filename);
	QVector<QPointF> planPath(QPoint start, QPoint end);

// slots:
	void on_requestPath_plan(const QPoint &start, const QPoint &end);

private:
	void loadMap(const QString &filename);
	void fillMap(const QString &filename);
	void expandObstacles();

	FloodPlanner &setStart(QPoint start);
	FloodPlanner &setEnd(QPoint end);

	bool isTileValid(const Map &map, QPoint point);
	void markTiles(Map &map, QPoint curr, QPoint end, int distance = 0);

private:
	std::shared_ptr<Map> m_map;
	QPoint m_start;
	QPoint m_end;
};

std::ostream &operator<<(std::ostream &os, const FloodPlanner::Map &map);

