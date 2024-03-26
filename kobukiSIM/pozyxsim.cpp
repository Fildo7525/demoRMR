#include "pozyxsim.h"
#include "string.h"
#include "math.h"
pozyxSim::pozyxSim()
{

}


void pozyxSim::loadBeacons(std::string filename)
{
    beacons.clear();
    FILE *fp=fopen(filename.c_str(),"r");
    if (fp==NULL)
    {

        printf("zly file\n");
        return ;
    }

    //tu nacitame obvodovu stenu
    char myLine[550];

    while(fgets(myLine,550,fp)!=NULL)
    {
        beacon beac;
        printf("%s\n",myLine);
        char *myCopy=(char*)calloc(strlen(myLine)+2,sizeof(char));
        memcpy(myCopy,myLine,sizeof(char)*strlen(myLine));
        char *freeMyCopy;
        freeMyCopy=myCopy;
        myCopy=strtok(myCopy," ");
        beac.id=atof(myCopy);
        myCopy=strtok(NULL," ");
        beac.x=atof(myCopy);
        myCopy=strtok(NULL," ");
        beac.y=atof(myCopy);
        free(freeMyCopy);
        beacons.push_back(beac);

    }

}

std::vector<beacon> pozyxSim::getBeacons()
{
    return beacons;
}

beacon pozyxSim::getBeaconById(int id)
{
    for(int i=0;i<beacons.size();i++)
    {
        if(beacons[i].id==id)
            return beacons[i];
    }
    beacon empty;
    empty.id=-1;
    return empty;
}

distBeacon pozyxSim::getDistnaceForBeacon(int id, double x,double y)
{
    for(int i=0;i<beacons.size();i++)
    {
        if(beacons[i].id==id)
            return distBeacon(beacons[i].id,sqrt(pow(x-beacons[i].x,2)+pow(y-beacons[i].y,2)));
    }
    distBeacon empty(-1,0);
    empty.id=-1;
    return empty;
}

std::vector<distBeacon> pozyxSim::calculateDistForBeacons(double x, double y)
{
    std::vector<distBeacon> output;
    for(int i=0;i<beacons.size();i++)
    {
        output.push_back(distBeacon(beacons[i].id,sqrt(pow(x-beacons[i].x,2)+pow(y-beacons[i].y,2))));
    }

    return output;
}
