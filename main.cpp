#include <QCoreApplication>
#include <time.h>
#include "ywmap.h"
YWMap map;
int main(int argc, char *argv[])
{
	QCoreApplication a(argc, argv);
	clock_t t;
	map.loadPlotConf();
	map.loadSpeedConf();
	map.setDocosm("zhangjiang_map.xml");
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
	cv::Mat show = map.Plot(point(31.2300,121.570), 0.06, 200000);
	printf("Plot finished! Begin AStar...\n");
	t = clock();
	map.PlotShortestPath(show, map.AStarDist(676412071,3373413655));
	t = clock() - t;
	printf("It takes %.3lfs to AStar\n", (float)t/CLOCKS_PER_SEC);
	//map.PlotShortestPath(show, map.AStarDist(676412514,1033088775));
	t = clock();
	map.PlotShortestPath(show, map.SPFA(676412071,3373413655), cv::Scalar(0,0,255));
	t = clock() - t;
	printf("It takes %.3lfs to SPFA\n", (float)t/CLOCKS_PER_SEC);
	//map.PlotShortestPath(show, map.AStarDist(676412514,1033088775));
	cv::imwrite("image.png",show);
	return a.exec();
}
