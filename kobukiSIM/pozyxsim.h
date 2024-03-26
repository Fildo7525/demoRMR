#ifndef POZYXSIM_H
#define POZYXSIM_H
#include <vector>
#include <string>
struct beacon
{
    int id;
    double x;
    double y;
};
struct distBeacon
{
    int id;
    double dist;
    distBeacon(int id,double dist):id(id),dist(dist){}
};
class pozyxSim
{

public:
    pozyxSim();
    void loadBeacons(std::string filename);
    std::vector<beacon> getBeacons();
    beacon getBeaconById(int id);
    distBeacon getDistnaceForBeacon(int id, double x,double y);
    std::vector<distBeacon> calculateDistForBeacons(double x, double y);
private:
    std::vector<beacon> beacons;

};

#endif // POZYXSIM_H
