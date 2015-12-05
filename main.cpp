#include <QCoreApplication>
#include "ywmap.h"
YWMap map;
int main(int argc, char *argv[])
{
	QCoreApplication a(argc, argv);
	map.loadPlotConf();
	map.setDocosm("map.osm");
	map.loadMap();
	printf("Load Finished!\n");
	cv::Mat show = map.Plot(point(31.20,121.585),0.01, 200000);
	printf("Plot Finished!\n");
	//cv::resize(show,show,cv::Size(700,683));
	//cv::imshow("show", show);
	cv::imwrite("image.png",show);
	//cv::waitKey(0);
	return a.exec();
}
