#include "ywmap.h"
extern YWMap map;
void YWMap::cmd_showmap()
{
	double lat,lon;
	int level;
	scanf("%lf%lf%d", &lat, &lon, &level);
	cv::Mat show = map.Plot(point(lat,lon), level);
	cv::imwrite("showmap.png", show);
}

void YWMap::cmd_shortestpath()
{
	char s1[100],s2[100];
	scanf("%s%s",s1,s2);
	unsigned id1 = strtoul(s1,NULL,0),id2 = strtoul(s2,NULL,0);
	//if(map.nodemap.find(id1) == map.nodemap.end()) id1 = map.queryName(s1)
	cv::Mat path = map.PlotShortestPath(map.AStarTime(id1,id2));
	cv::imwrite("shortestpath.png", path);
}
