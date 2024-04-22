#include "ParticleFilter.h"
#include "rplidar.h"
#include <algorithm>
#include <QPoint>
#include <qdebug.h>
#include <queue>
#include <random>


#define DEG2RAD(x) ((x) * M_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / M_PI)

// TODO: The laserData cannot be initialized like this. Just take the first measurement.
ParticleFilter::ParticleFilter(QObject *parent, const Position &position)
	: QObject(parent)
	, m_lastPosition(position)
	, m_randomDevice()
	, m_generator(m_randomDevice())
{
}

Position ParticleFilter::update(const Position &position, const LaserMeasurement &laserData)
{
	if (!isInitialized()) {
		qDebug() << "ParticleFilter is not yet initialized! Returning position";
		return position;
	}

	auto particles = resample(position, 0.1);
	emit particlesReady(particles);
	std::pair<double, Position> lowestError = std::make_pair(std::numeric_limits<double>::max(), m_lastPosition);

	// qDebug() << "Updating particles";
	// qDebug() << "laserData: " << laserData.numberOfScans;
	auto newLidarData = lidarDataToCartesian(laserData);

	m_tree.build(newLidarData);

	std::for_each(particles.begin(),
				  particles.end(),
				  [this, &lowestError, &newLidarData, &position](Position &particle) {

		auto paritcleLaserData = transformLidarData(m_lastLidarCartesianData, particle);
		double error = errorFromLidarData(newLidarData, paritcleLaserData);
		if (lowestError.first > error) {
			lowestError = {error, particle};
		}
	});

	auto [error, newPosition] = lowestError;

	// qDebug() << "Lowest error is " << error << ". The old position was "
	// 	<< m_lastPosition.x << ", " << m_lastPosition.y << ", " << m_lastPosition.rotation
	// 	<< " and the new position is " << newPosition.x << ", " << newPosition.y << ", " << newPosition.rotation;

	m_lastLidarCartesianData = std::move(newLidarData);
	m_lastPosition = newPosition;

	return newPosition;
}

QVector<Position> ParticleFilter::resample(const Position &position, double std, int count)
{
	// values near the mean are the most likely
	// standard deviation affects the dispersion of generated values from the mean
	std::normal_distribution xDist{0., (position.x - m_lastPosition.x) * cos(position.rotation)};
	std::normal_distribution yDist{0., (position.y - m_lastPosition.y) * sin(position.rotation)};
	std::normal_distribution rotDist{0., 0.01};

	// qDebug() << "resampling particles";
	// draw a sample from the normal distribution and round it to an integer
	auto random_x = [this, &xDist]{ return xDist(m_generator); };
	auto random_y = [this, &yDist]{ return yDist(m_generator); };
	auto random_fi = [this, &rotDist]{ return rotDist(m_generator); };

	QVector<Position> particles = {position};

	for (int i = 0; i < count-1; ++i) {
		Position p;

		p.x = position.x + random_x();
		p.y = position.y + random_y();
		p.rotation = position.rotation + random_fi();

		particles.push_back(p);
	}

	return particles;
}

QVector<QPointF> ParticleFilter::lidarDataToCartesian(const LaserMeasurement &laserData)
{
	QVector<QPointF> output;

	for (size_t i = 0; i < laserData.numberOfScans; i++) {
		LaserData point = laserData.Data[i];

		double x = point.scanDistance/1000. * cos(DEG2RAD(point.scanAngle));
		double y = point.scanDistance/1000. * sin(DEG2RAD(point.scanAngle));
		output.push_back(QPointF(x, y));
	}

	return output;
}

double ParticleFilter::errorFromLidarData(const QVector<QPointF> &newLidarData, const QVector<QPointF> &oldLidarData)
{
	double error = 0.0;
	int acceptablePoints = 0;

	for (int i = 0; i < oldLidarData.size(); ++i) {
		int idx = m_tree.nnSearch(MyPoint(oldLidarData[i]));
		auto tmp = (newLidarData[idx] - oldLidarData[i]).manhattanLength();
		// qDebug() << "Error: " << tmp;
		if (tmp < 0.01) {
			error += tmp;
			acceptablePoints++;
		}
	}

	acceptablePoints = std::max(acceptablePoints, 1);

	qDebug() << "Full error: " << error << " Acceptable points: " << acceptablePoints << " Average error: " << error / acceptablePoints;
	return error / acceptablePoints;
}

QVector<QPointF> ParticleFilter::transformLidarData(const QVector<QPointF> &laserData, const Position &particle)
{
	QVector<QPointF> output;

	for (const auto &point : laserData) {
		auto x = point.x() * cos(particle.rotation) - point.y() * sin(particle.rotation) + (particle.x - m_lastPosition.x);
		auto y = point.x() * sin(particle.rotation) + point.y() * cos(particle.rotation) + (particle.y - m_lastPosition.y);
		output.push_back(QPointF(x, y));
	}

	return output;
}
