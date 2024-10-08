#pragma once

#include <QObject>
#include <opencv2/core/mat.hpp>
#include <random>
#include "rplidar.h"
#include "kd-tree/kdtree.h"

struct Position
{
	double x;
	double y;
	double rotation;
};

/**
 * @class ParticleFilter
 * @brief Class based on a QObject. Filter of position calculated by odometry based on the lidar data.
 *
 * TODO: Export some parameters of the filter as parameters. Thus, the filter can be tuned by the user.
 */
class ParticleFilter
	: public QObject
{
	Q_OBJECT;

public:
	/**
	 * @brief Constructor of the ParticleFilter class.
	 *
	 * @param parent Parent that delets this object.
	 * @param position Initial position of the robot. Usually [0, 0, 0];
	 */
	explicit ParticleFilter(QObject *parent, const Position &position);

	/**
	 * @brief Initialize the ParticleFilter with the last lidar data.
	 *
	 * @param laserData LaserMeasurement data from the lidar.
	 */
	void setLastLaserData(const LaserMeasurement &laserData) { m_lastLidarCartesianData = lidarDataToCartesian(laserData); }

	/**
	 * @brief Check if the ParticleFilter is initialized.
	 *
	 * Being initialized means that the last lidar data is not empty.
	 *
	 * @see setLastLaserData If this function is called with non empty data, the ParticleFilter is initialized.
	 *
	 * @return True if the ParticleFilter is initialized, false otherwise.
	 */
	bool isInitialized() const { return !m_lastLidarCartesianData.empty(); }

	/**
	 * @brief Updata the current position calculated by odometry based on the current lidar data @a laserData.
	 *
	 * @param position Current position of the robot based on the odometry.
	 * @param laserData LaserMeasurement data from the lidar from current position.
	 * @return Current updated position of the robot.
	 */
	Position update(const Position &position, const LaserMeasurement &laserData);

signals:
	/**
	 * @brief Singal emitted when the particles are ready.
	 *
	 * This signal is just for debugging purposes. If anybody wants
	 * to see the generated particles, just connect a slot to this signal.
	 *
	 * @see resample Function that generates the particles.
	 *
	 * @param particles Vector of particles. It's size is determined by the resample function.
	 */
	void particlesReady(const QVector<Position> &particles);

private:
	/**
	 * @brief Creates particles based on the current position of the robot according to the normal Distribution.
	 *
	 * The distribution of the particles is based on the normal distribution with mean and standard deviation.
	 *
	 * @param position Current position of the robot.
	 * @param count Number of particles to be created.
	 * @return Vector of particles.
	 */
	QVector<Position> resample(const Position &position, int count = 20);

	/**
	 * @brief Transforms the laser data from the polar representation to the Cartesian representation.
	 *
	 * @param laserData Laser data from the lidar to be transformed to the Cartesian representation.
	 * @return Vector of points in the Cartesian representation.
	 */
	QVector<QPointF> lidarDataToCartesian(const LaserMeasurement &laserData);

	/**
	 * @brief Calculates a single error value from the new and old lidar data.
	 *
	 * The error is calculated from the points that are the same for both lidar scans.
	 *
	 * @param newLidarData Data from the current scan.
	 * @param oldLidarData Data from the previous scan.
	 * @param allowedError Allowed error in the Cartesian coordinates.
	 * @return Error in Cartesian coordinates.
	 */
	double errorFromLidarData(const QVector<QPointF> &newLidarData, const QVector<QPointF> &oldLidarData, double allowedError = 0.01);

	/**
	 * @brief Move the last lidar data from the last position to the position of the @a particle.
	 *
	 * @param laserData Lidar data from the last scan.
	 * @param particle Position to which the lidar data should be moved.
	 */
	QVector<QPointF> transformLidarData(const QVector<QPointF> &laserData, const Position &particle);

private:
	LaserMeasurement m_lastLaserData;
	QVector<QPointF> m_lastLidarCartesianData;
	Position m_lastPosition;

	std::random_device m_randomDevice;
	std::mt19937 m_generator;
	kdt::KDTree<MyPoint> m_tree;
};
