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
	cv::Mat path = map.PlotShortestPath(map.AStarTime(id1,id2,slowset));
	cv::imwrite("shortestpath.png", path);
}

void YWMap::cmd_querybox()
{
	double minlat,maxlat,minlon,maxlon;
	scanf("%lf%lf%lf%lf", &maxlat, &minlon, &minlat, &maxlon);
	box b(point(minlat,minlon), point(maxlat,maxlon));
	clock_t t = clock();
	std::vector<std::pair<point,unsigned>> result = map.querybox(b);
	t = clock() - t;
	printf("[RTree query box]It takes %fms\n", (float)t / CLOCKS_PER_SEC * 1000);
	cv::Mat node = map.PlotPointInBox(b, result, cv::Scalar(0x24, 0xA7, 0x6D));
	cv::imwrite("querybox.png", node);
}

void YWMap::cmd_querynearest()
{
	double lat,lon;
	int k;
	scanf("%lf%lf%d", &lat, &lon, &k);
	point p(lat,lon);
	clock_t t = clock();
	std::vector<std::pair<point,unsigned>> result = map.nearestForPlot(p,k);
	t = clock() - t;
	printf("[RTree query nearest]It takes %fms\n", (float)t / CLOCKS_PER_SEC * 1000);
	cv::Mat nearest = map.PlotPointNearest(p, result, cv::Scalar(0x24, 0xA7, 0x6D));
	cv::imwrite("nearest.png", nearest);
}
