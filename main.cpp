#include <QCoreApplication>
#include <time.h>
#include "ywmap.h"
YWMap map;
std::map<std::pair<double,double>,cv::Mat> images;
double dt(int level)
{
	return 1310.72 * pow(2.0 ,-level);
}

int main(int argc, char *argv[])
{
	QCoreApplication a(argc, argv);
	map.loadPlotConf();
	map.loadSpeedConf();
	map.setDocosm("shanghai_map.xml");
	map.loadMap();
	printf("Load Finished!\n");
	//cv::Mat show = map.Plot(point(31.3200,121.4900),0.04, 200000);
	//printf("Plot Finished!\n");
	//cv::resize(show,show,cv::Size(700,683));
	//cv::imshow("show", show);
	/*double maxlat,minlon,l;
	while(scanf("%lf%lf%lf", &maxlat, &minlon, &l) != EOF)
	{
		clock_t t = clock();
		cv::Mat show = map.Plot(point(maxlat,minlon),l, 200000);
		cv::imwrite("image.png", show);
		t = clock() - t;
		printf("It takes %.3lfs to plot\n", (float)t/CLOCKS_PER_SEC);
	}*/
	//map.queryName("附属");
	//cv::Mat show = map.Plot(point(31.30186,121.51159), 13);//17/31.30186/121.51159
	//cv::Mat show = map.Plot(point(31.2976,121.4157), 14);
	//printf("Plot Finished!\n");
	//printf("Plot finished! Begin AStar...\n");
	//map.PlotShortestPath(show, map.AStarDist(676412071,3373413655));
	//map.PlotShortestPath(show, map.SPFA(676412071,3373413655), point(31.2300,121.570), 13, cv::Scalar(0,0,255));
	//cv::imwrite("image.png",show);
	//cv::resize(show,show,cv::Size(900,870));
	//cv::imshow("show", show);
	//cv::waitKey(0);

	//printf("finished\n");

	printf("Welcome!\n");
	printf(">> ");
	fflush(stdout);
	char buf[20];
	while(scanf("%s", buf))
	{
		if(strcmp(buf,"quit") == 0)break;
		//else if(strcmp(buf, "help") == 0) cmd_help();
		else if(strcmp(buf, "showmap") == 0) YWMap::cmd_showmap();
		else if(strcmp(buf, "shortestpath") == 0) YWMap::cmd_shortestpath(); //shortestpath 1008057714 1813887498
		//else if(strcmp(buf, "querynode") == 0) cmd_querynode();
		//else if(strcmp(buf, "queryway") == 0) cmd_queryway();
		//else if(strcmp(buf, ""))
		printf(">> ");
		fflush(stdout);
	}
	return a.exec();
}
