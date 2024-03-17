#include "floodPlanner.h"

#include <QFile>
#include <QDebug>
#include <algorithm>
#include <iostream>
#include <iomanip>

#define START_FLAG -5
#define END_FLAG -6
#define TILE_SIZE 10

uint qHash(const QPoint &point)
{
	return qHash(point.x()) ^ qHash(point.y());
}

FloodPlanner::FloodPlanner(const QString &filename)
	: QObject(nullptr)
	, m_map(new Map())
{
	loadMap(filename);
}

QPoint FloodPlanner::toMapCoord(const QPointF &point)
{
	return QPoint(point.x(), -point.y()) * 1000 / 20 / TILE_SIZE + QPoint(m_map->at(0).size() / 4, m_map->size() / 2);
}

QPointF FloodPlanner::toWorlCoord(const QPoint &point)
{
	return QPointF( (point.x() - m_map->at(0).size() / 4.) * TILE_SIZE * 20. / 1000.,
				   -(point.y() - m_map->size() / 2.) * TILE_SIZE * 20. / 1000.);
}

void FloodPlanner::loadMap(const QString &filename)
{
	m_map->clear();

	fillMap(filename);
	// std::cout << "Loaded from file:\n" << *m_map << std::endl;
	expandObstacles();
	// std::cout << "Expanded with obstacles:\n" << *m_map << std::endl;
}

void FloodPlanner::on_requestPath_plan(const QPoint &start, const QPoint &end)
{
	QPoint s = toMapCoord(start);
	QPoint e = toMapCoord(end);

	qDebug() << "Start: " << s << " End: " << e;

	auto path = planPath(s, e, TrajectoryType::Diagonal);

	emit pathPlanned(path);
}

void FloodPlanner::fillMap(const QString &filename)
{
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly)) {
		throw std::runtime_error("Failed to open file");
	}

	while (!file.atEnd()) {
		QString line = file.readLine();
		QVector<int> row;
		for (auto c : line) {
			if (c == '#') {
				row.push_back(-1);
			}
			else {
				row.push_back(0);
			}
		}
		m_map->push_back(row);
	}
}

void FloodPlanner::expandObstacles()
{
	for (int y = 0; y < m_map->size(); ++y) {
		for (int x = 0; x < m_map->at(y).size(); ++x) {
			if (m_map->at(y).at(x) == -1) {
				if (y > 0 && m_map->at(y - 1).at(x) == 0) {
					(*m_map)[y - 1][x] = -2;
				}
				if (y < m_map->size() - 1 && m_map->at(y + 1).at(x) == 0) {
					(*m_map)[y + 1][x] = -2;
				}
				if (x > 0 && m_map->at(y).at(x - 1) == 0) {
					(*m_map)[y][x - 1] = -2;
				}
				if (x < m_map->at(y).size() - 1 && m_map->at(y).at(x + 1) == 0) {
					(*m_map)[y][x + 1] = -2;
				}
			}
		}
	}
}

bool FloodPlanner::isTileValid(const Map &map, QPoint point)
{
	// Check for out of bounds.
	if (point.x() < 0 || point.x() >= map[0].size()) {
		return false;
	}
	if (point.y() < 0 || point.y() >= map.size()) {
		return false;
	}

	// Check for obstacles.
	if (map[point.y()][point.x()] < 0) {
		return false;
	}

	return true;
}

void FloodPlanner::markTiles(Map &map, const QPoint &start, const QPoint &end, TrajectoryType type)
{
	auto [dx, dy] = getDirections(type);

	QVector<QPoint> toVisit{start};
	QSet<QPoint> visited{start};

	while (!toVisit.empty()) {
		QPoint curr = toVisit.front();
		toVisit.pop_front();

		if (curr == end) {
			return;
		}

		for (int i = 0; i < dx.size(); ++i) {
			QPoint next = curr + QPoint(dx[i], dy[i]);
			if (isTileValid(map, next) && !visited.contains(next)) {
				map[next.y()][next.x()] = map[curr.y()][curr.x()] + 1;
				toVisit.push_back(next);
				visited.insert(next);
			}
		}
	}
}

QVector<QPointF> FloodPlanner::planPath(const QPoint &start, const QPoint &end, TrajectoryType type)
{
	Map map = *m_map;
	markTiles(map, end, start, type);

	auto path = pathFromMap(map, start, end, type);

	auto pathF = prunePath(path);

	std::cout << "Pruned:\n";
	printMapWithPath(path);
	qDebug() << pathF;

	return pathF;
}

QVector<QPoint> FloodPlanner::pathFromMap(const Map &map, const QPoint &start, const QPoint &end, TrajectoryType type)
{
	QVector<QPoint> path;
	QPoint curr = start;
	auto [dx, dy] = getDirections(type);

	while (curr != end)
	{
		for (size_t i = 0; i < dx.size(); i++)
		{
			QPoint next = curr + QPoint{dx[i], dy[i]};
			if (isTileValid(map, next) && map[next.y()][next.x()] < map[curr.y()][curr.x()]) {
				path.push_back(curr);
				curr = next;
				break;
			}
		}
	}

	path.push_back(curr);

	return path;
}

QVector<QPointF> FloodPlanner::prunePath(const QVector<QPoint> &path)
{
	auto curr = path.begin();
	auto next = path.begin()+1;
	QPoint lastDiff = {next->x() - curr->x(), next->y() - curr->y()};

	QVector<QPointF> output;

	while (next != path.end()) {
		QPoint diff = {next->x() - curr->x(), next->y() - curr->y()};

		if (diff != lastDiff) {
			output.push_back(toWorlCoord(*curr));
			lastDiff = diff;
		}
		
		curr = next;
		next++;
	}

	return output;
}

QPair<QVector<int>, QVector<int>> FloodPlanner::getDirections(TrajectoryType type)
{
	// First is dx
	// Second is dy
	// (-1, 1) | (0,  1) | (1, 1)
	// (-1, 0) | (0,  0) | (1, 0)
	// (-1,-1) | (0, -1) | (1,-1)
	QPair<QVector<int>, QVector<int>> dirs;
	if (type == TrajectoryType::Manhattan) {
		dirs.first =  { 0, 1, 0, 0};
		dirs.second = {-1, 0, 1,-1};
	}
	else {
		dirs.first =  { 0, 1, 1, 1, 0,-1,-1,-1};
		dirs.second = {-1,-1, 0, 1, 1, 1, 0,-1};
	}

	return dirs;
}

void FloodPlanner::printMapWithPath(const QVector<QPoint> &points)
{
	Map map = *m_map;
	for (const auto &p : points) {
		map[p.y()][p.x()] = START_FLAG;
	}

	std::cout << map << std::endl;
}

std::ostream &operator<<(std::ostream &os, const FloodPlanner::Map &map)
{
	os << "  |";
	for (size_t i = 0; i < map[0].size(); i++) {
		os << std::setw(3) << i << "|";
	}
	os << std::endl << std::string(map[0].size() * 4 + 3, '-') << std::endl;

	int idx = 0;
	for (const auto &row : map) {
		int nu = 0;
		os << std::setw(2) << idx++ << "|";
		for (const auto &tile : row) {
			if (tile == 0) {
				os << "   ";
			}
			else if (tile > 0) {
				os << std::setw(3) << tile;
			}
			else if (tile == START_FLAG) {
				os << " S ";
			}
			else if (tile == END_FLAG) {
				os << " E ";
			}
			else {
				os << "###";
			}
			os << '|';
		}
		os << std::endl << std::string(row.size() * 4 + 3, '-') << std::endl;
	}
	return os;
}

