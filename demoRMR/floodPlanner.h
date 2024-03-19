#pragma once

#include <optional>
#include <memory>

#include <QObject>
#include <QPoint>
#include <QString>
#include <QVector>

// TODO: create link to Cfree space in the map.

class FloodPlanner
	: public QObject
{
	Q_OBJECT;

	using Map = QVector<QVector<int>>;
	enum class TrajectoryType {
		Diagonal,
		Manhattan,
	};

	friend std::ostream &operator<<(std::ostream &os, const Map &map);

public:
	explicit FloodPlanner(const QString &filename);

public slots:
	void on_requestPath_plan(const QPointF &start, const QPointF &end);

public:
signals:
	void pathPlanned(QVector<QPointF> path);

private:
	QPoint toMapCoord(const QPointF &point);
	QPointF toWorlCoord(const QPoint &point);

	void loadMap(const QString &filename);
	void fillMap(const QString &filename);
	void expandObstacles();

	bool isInCFree(const QPoint &point);
	QPoint nearestCFreePoint(const QPoint &point, TrajectoryType type);

	bool isTileValid(const Map &map, QPoint point);
	void markTiles(Map &map, const QPoint &start, const QPoint &end, TrajectoryType type);
	QVector<QPointF> planPath(const QPoint &start, const QPoint &end, TrajectoryType type = TrajectoryType::Manhattan);
	QVector<QPoint> pathFromMap(const Map &map, const QPoint &start, const QPoint &end, TrajectoryType type);
	QVector<QPointF> prunePath(const QVector<QPoint> &path);

	QPair<QVector<int>, QVector<int>> getDirections(TrajectoryType type);

	void printMapWithPath(const QVector<QPoint> &points);

private:
	std::shared_ptr<Map> m_map;
	QPoint m_start;
	QPoint m_end;
};

std::ostream &operator<<(std::ostream &os, const FloodPlanner::Map &map);

