//#include <QCoreApplication>
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
	//QCoreApplication a(argc, argv);
	map.loadPlotConf();
	map.loadSpeedConf();
	map.setDocosm("shanghai_map.xml");
	map.loadMap();
	printf("Load Finished!\n");
	printf("Welcome!\n");
	printf(">> ");
	fflush(stdout);
	char buf[20];
	while(scanf("%s", buf))
	{
		if(strcmp(buf,"quit") == 0)break;
		else if(strcmp(buf, "help") == 0) YWMap::cmd_help();
		else if(strcmp(buf, "showmap") == 0) YWMap::cmd_showmap(); //showmap 31.30186 121.51159 13
		else if(strcmp(buf, "shortestpath") == 0) YWMap::cmd_shortestpath();
				//shortestpath 1008057714 1813887498 2 227962891 11635095
				//shortestpath 邯郸路 张江校区 2 8886548 8886576
		else if(strcmp(buf, "querynode") == 0) YWMap::cmd_querybox(); //querynode 31.2449 121.4886 31.2349 121.5073
		else if(strcmp(buf, "querynearest") == 0) YWMap::cmd_querynearest(); //querynearest 31.2392 121.4956 100
		else if(strcmp(buf, "queryname") == 0) YWMap::cmd_queryname();
		else if(strcmp(buf, "queryway") == 0) YWMap::cmd_queryway(); //queryway 肇嘉浜路 queryway 世纪大道 queryway 272231909
		else if(strcmp(buf, "loadtaxi") == 0) map.loadtaxi();
		else if(strcmp(buf, "querytaxi") == 0) YWMap::cmd_querytaxi();
		//querytaxi 10944 0:0:1 23:59:59 2 2 3(最后三个数字代表，输出两个点的信息，分别是第二个点和第三个点)
		else if(strcmp(buf, "shortesttest") == 0) YWMap::cmd_shortesttest();
		else if(strcmp(buf, "insertpoint") == 0) YWMap::cmd_insertpoint();
		//>> querynode 31.2449 121.4886 31.24 121.49
		//>> insertpoint 31.2440 121.489
		//>> querynode 31.2449 121.4886 31.24 121.49
		else if(strcmp(buf, "queryneartaxi") == 0) YWMap::cmd_queryNearestTaxi();
		//queryneartaxi 31.2363 121.4690 0.01 21:0:0 3600

		else printf("cmd not FOUND\n");
		//else if(strcmp(buf, ""))
		printf(">> ");
		fflush(stdout);
	}
	//return a.exec();
	return 0;
}
