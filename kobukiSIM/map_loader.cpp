#include "map_loader.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include <limits>

map_loader::map_loader()
{
	minX = std::numeric_limits<double>::max();
	maxX = std::numeric_limits<double>::min();
	minY = std::numeric_limits<double>::max();
	maxY = std::numeric_limits<double>::min();
}

void map_loader::load_map(const char filename[], TMapArea &mapss)
{
	FILE *fp = fopen(filename, "r");
	if (fp == NULL) {
		printf("zly file\n");
		return;
	}

	//tu nacitame obvodovu stenu
	char myLine[550];
	fgets(myLine, 550, fp);
	// printf("%s\n", myLine);
	char *myCopy = (char *)calloc(strlen(myLine) + 2, sizeof(char));
	memcpy(myCopy, myLine, sizeof(char) * strlen(myLine));
	char *freeMyCopy;
	freeMyCopy = myCopy;
	myCopy = strtok(myCopy, "[]");
	mapss.wall.numofpoints = (atoi(myCopy));
	// printf("num of points %i\n", mapss.wall.numofpoints);
	mapss.wall.points.reserve(mapss.wall.numofpoints);
	for (int i = 0; i < mapss.wall.numofpoints; i++) {
		TMapPoint temp;
		myCopy = strtok(NULL, "[,");
		temp.point.x = atof(myCopy);
		myCopy = strtok(NULL, "[,");
		temp.point.y = atof(myCopy);
		mapss.wall.points.push_back(temp);
		//   mapss.wall.points[i/2].suradnice[i%2]=atof(myCopy);

		minX = temp.point.x < minX ? temp.point.x : minX;
		maxX = temp.point.x > maxX ? temp.point.x : maxX;
		minY = temp.point.y < minY ? temp.point.y : minY;
		maxY = temp.point.y > maxY ? temp.point.y : maxY;
		;
		;
	}
	free(freeMyCopy);

	//tu nacitame jednotlive prekazky
	mapss.numofObjects = 0;
	mapss.obstacle.clear();
	while (fgets(myLine, 550, fp)) {
		// printf("%s\n", myLine);
		myCopy = (char *)calloc(strlen(myLine) + 2, sizeof(char));
		memcpy(myCopy, myLine, sizeof(char) * strlen(myLine));

		freeMyCopy = myCopy;
		myCopy = strtok(myCopy, "[]");
		if ((atoi(myCopy)) == 0)
			break;
		TMapObject tempObstacle;
		mapss.numofObjects++;

		tempObstacle.numofpoints = (atoi(myCopy));
		for (int i = 0; i < tempObstacle.numofpoints; i++) {
			TMapPoint temp;
			myCopy = strtok(NULL, "[,");
			temp.point.x = atof(myCopy);
			myCopy = strtok(NULL, "[,");
			temp.point.y = atof(myCopy);
			tempObstacle.points.push_back(temp);
		}
		free(freeMyCopy);
		mapss.obstacle.push_back(tempObstacle);
	}


	//--tu pridam este nahodnu prekazku...
	/*   double randPositionX= minX + (float)(static_cast <float> (rand()) /( static_cast <float> (RAND_MAX)))*(maxX-minX);;
      double randPositionY= minY + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(maxY-minY)));;
      TMapObject tempObstacle;
      mapss.numofObjects++;

      tempObstacle.numofpoints=(6);
     // for(int i=0;i< tempObstacle.numofpoints;i++)
       {
          TMapPoint temp;

          temp.point.x=0+randPositionX;
          temp.point.y=-20+randPositionY;
          tempObstacle.points.push_back(temp);
          temp.point.x=17.32+randPositionX;
          temp.point.y=-10+randPositionY;
          tempObstacle.points.push_back(temp);
          temp.point.x=17.32+randPositionX;
          temp.point.y=10+randPositionY;
          tempObstacle.points.push_back(temp);
          temp.point.x=0+randPositionX;
          temp.point.y=20+randPositionY;
          tempObstacle.points.push_back(temp);
          temp.point.x=-17.32+randPositionX;
          temp.point.y=10+randPositionY;
          tempObstacle.points.push_back(temp);
          temp.point.x=-17.32+randPositionX;
          temp.point.y=-10+randPositionY;
          tempObstacle.points.push_back(temp);

       }
       free(freeMyCopy);
       mapss.obstacle.push_back(tempObstacle);*/

	//------------------------------------
	fflush(stdout);
}


void map_loader::addRandomHexagon(double x, double y, TMapArea &mapss)
{
	double randPositionX = x;
	;
	double randPositionY = y;
	TMapObject tempObstacle;
	mapss.numofObjects++;

	tempObstacle.numofpoints = (6);
	// for(int i=0;i< tempObstacle.numofpoints;i++)
	{
		TMapPoint temp;

		temp.point.x = 0 + randPositionX;
		temp.point.y = -20 + randPositionY;
		tempObstacle.points.push_back(temp);
		temp.point.x = 17.32 + randPositionX;
		temp.point.y = -10 + randPositionY;
		tempObstacle.points.push_back(temp);
		temp.point.x = 17.32 + randPositionX;
		temp.point.y = 10 + randPositionY;
		tempObstacle.points.push_back(temp);
		temp.point.x = 0 + randPositionX;
		temp.point.y = 20 + randPositionY;
		tempObstacle.points.push_back(temp);
		temp.point.x = -17.32 + randPositionX;
		temp.point.y = 10 + randPositionY;
		tempObstacle.points.push_back(temp);
		temp.point.x = -17.32 + randPositionX;
		temp.point.y = -10 + randPositionY;
		tempObstacle.points.push_back(temp);
	}

	mapss.obstacle.push_back(tempObstacle);
}
void map_loader::updateRandomHexagon(double x, double y, TMapArea &mapss)
{
	if (mapss.obstacle.at(mapss.obstacle.size() - 1).points.size() != 6) {
		addRandomHexagon(x, y, mapss);
		return;
	}
	double randPositionX = x;
	;
	double randPositionY = y;


	// for(int i=0;i< tempObstacle.numofpoints;i++)
	{
		TMapPoint temp;

		mapss.obstacle.at(mapss.obstacle.size() - 1).points.at(0).point.x = 0 + randPositionX;
		mapss.obstacle.at(mapss.obstacle.size() - 1).points.at(0).point.y = -20 + randPositionY;

		mapss.obstacle.at(mapss.obstacle.size() - 1).points.at(1).point.x = 17.32 + randPositionX;
		mapss.obstacle.at(mapss.obstacle.size() - 1).points.at(1).point.y = -10 + randPositionY;

		mapss.obstacle.at(mapss.obstacle.size() - 1).points.at(2).point.x = 17.32 + randPositionX;
		mapss.obstacle.at(mapss.obstacle.size() - 1).points.at(2).point.y = 10 + randPositionY;

		mapss.obstacle.at(mapss.obstacle.size() - 1).points.at(3).point.x = 0 + randPositionX;
		mapss.obstacle.at(mapss.obstacle.size() - 1).points.at(3).point.y = 20 + randPositionY;

		mapss.obstacle.at(mapss.obstacle.size() - 1).points.at(4).point.x = -17.32 + randPositionX;
		mapss.obstacle.at(mapss.obstacle.size() - 1).points.at(4).point.y = 10 + randPositionY;

		mapss.obstacle.at(mapss.obstacle.size() - 1).points.at(5).point.x = -17.32 + randPositionX;
		mapss.obstacle.at(mapss.obstacle.size() - 1).points.at(5).point.y = -10 + randPositionY;
	}
}
