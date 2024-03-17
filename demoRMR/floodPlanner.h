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
	enum class TrajectoryType {
		Diagonal,
		Manhattan,
	};

	friend std::ostream &operator<<(std::ostream &os, const Map &map);

public:
	explicit FloodPlanner(const QString &filename);

// slots:
	void on_requestPath_plan(const QPoint &start, const QPoint &end);

private:
	QPoint toMapCoord(const QPointF &point);
	QPointF toWorlCoord(const QPoint &point);

	void loadMap(const QString &filename);
	void fillMap(const QString &filename);
	void expandObstacles();

	bool isTileValid(const Map &map, QPoint point);
	void markTiles(Map &map, const QPoint &start, const QPoint &end, TrajectoryType type);
	QVector<QPointF> planPath(const QPoint &start, const QPoint &end, TrajectoryType type = TrajectoryType::Manhattan);
	QVector<QPointF> pathFromMap(const Map &map, const QPoint &start, const QPoint &end, TrajectoryType type);

	QPair<QVector<int>, QVector<int>> getDirections(TrajectoryType type);

private:
	std::shared_ptr<Map> m_map;
	QPoint m_start;
	QPoint m_end;
};

std::ostream &operator<<(std::ostream &os, const FloodPlanner::Map &map);

