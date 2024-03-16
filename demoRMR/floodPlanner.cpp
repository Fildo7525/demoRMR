#include "floodPlanner.h"

#include <QFile>
#include <QDebug>
#include <iostream>

FloodPlanner::FloodPlanner(const QString &filename)
	: QObject(nullptr)
	, m_map(new Map())
{
	loadMap(filename);
}

void FloodPlanner::loadMap(const QString &filename)
{
	m_map->clear();

	fillMap(filename);
	std::cout << "Loaded from file:\n" << *m_map << std::endl;
	expandObstacles();
	std::cout << "Expanded with obstacles:\n" << *m_map << std::endl;
}

QVector<QPointF> FloodPlanner::planPath(QPoint start, QPoint end)
{
	setStart(start);
	setEnd(end);

	Map localMap = *m_map;
	QVector<QPointF> path;

	markTiles(localMap, end, start);

	std::cout << "Planned:\n" << localMap << std::endl;

	return {};
}

void FloodPlanner::on_requestPath_plan(const QPoint &start, const QPoint &end)
{
	qDebug() << __FUNCTION__ << " " << start << " " << end;
	auto path = planPath(start, end);
	qDebug() << "Path: " << path;
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

FloodPlanner& FloodPlanner::setStart(QPoint start)
{
	m_start = start;
	return *this;
}

FloodPlanner& FloodPlanner::setEnd(QPoint end)
{
	m_end = end;
	return *this;
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

void FloodPlanner::markTiles(Map &map, QPoint curr, QPoint end, int distance)
{
	if (map[curr.y()][curr.x()] == 0 || distance < map[curr.y()][curr.x()]) {
		map[curr.y()][curr.x()] = distance;
	}

	if (curr == end) {
		return;
	}

	for (size_t y = curr.y()-1; y < curr.y()+1; y++) {
		for (size_t x = curr.x()-1; x < curr.x()+1; x++) {
			if (isTileValid(map, QPoint(x, y)) && map[y][x] == 0) {
				markTiles(map, QPoint(x, y), end, distance + 1);
			}
		}
	}
}

std::ostream& operator<<(std::ostream &os, const FloodPlanner::Map &map)
{
	for (const auto &row : map) {
		for (const auto &tile : row) {
			if (tile == 0) {
				os << ' ';
			}
			else if (tile > 0) {
				os << tile;
			}
			else {
				os << '#';
			}
		}
		os << std::endl;
	}
	return os;
}
