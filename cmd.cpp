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
	if(map.getNodeIndexById(id1) == -1)
	{
		auto name_point =  map.queryName(s1);
		if(name_point.size() == 0)
		{
			printf("%s NOT FOUND\n", s1);
			return;
		}
		id1 = map.getNodeIdByIndex(map.nearest(name_point[0].second)[0]);
		printf("id1 = %u(%s)\n", id1, name_point[0].first.c_str());
	}
	if(map.getNodeIndexById(id2) == -1)
	{
		auto name_point = map.queryName(s2);
		if(name_point.size() == 0)
		{
			printf("%s NOT FOUND\n", s2);
			return;
		}
		id2 = map.getNodeIdByIndex(map.nearest(name_point[0].second)[0]);
		printf("id2 = %u(%s)\n", id2, name_point[0].first.c_str());
	}
	int slownum;
	scanf("%d",&slownum);
	std::set<unsigned> slowset;
	for(int i=0;i<slownum;i++)
	{
		unsigned id;
		scanf("%u", &id);
		slowset.insert(id);
	}
	//if(map.nodemap.find(id1) == map.nodemap.end()) id1 = map.queryName(s1)
	cv::Mat path = map.PlotShortestPath(map.SPFATime(id1,id2,slowset));
	cv::imwrite("shortestpath.png", path);
}
