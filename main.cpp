#include <QCoreApplication>
#include <time.h>
#include "ywmap.h"
YWMap map;
int main(int argc, char *argv[])
{
	QCoreApplication a(argc, argv);
	map.loadPlotConf();
	map.setDocosm("shanghai_map.xml");
	map.loadMap();
	printf("Load Finished!\n");
	//cv::Mat show = map.Plot(point(31.3200,121.4900),0.04, 200000);
	//printf("Plot Finished!\n");
	//cv::resize(show,show,cv::Size(700,683));
	//cv::imshow("show", show);
	//cv::imwrite("image.png",show);
	//cv::waitKey(0);
	double maxlat,minlon,l;
	while(scanf("%lf%lf%lf", &maxlat, &minlon, &l) != EOF)
	{
		clock_t t = clock();
		cv::Mat show = map.Plot(point(maxlat,minlon),l, 200000);
		cv::imwrite("image.png", show);
		t = clock() - t;
		printf("It takes %.3lfs to plot\n", (float)t/CLOCKS_PER_SEC);
	}
	return a.exec();
}
