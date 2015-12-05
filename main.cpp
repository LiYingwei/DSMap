#include <QCoreApplication>
#include "ywmap.h"
YWMap map;
int main(int argc, char *argv[])
{
	QCoreApplication a(argc, argv);
	map.loadPlotConf();
	map.setDocosm("shanghai_map.xml");
	map.loadMap();
	printf("Load Finished!\n");
	cv::Mat show = map.Plot(point(31.3200,121.4900),0.04, 200000);
	printf("Plot Finished!\n");
	//cv::resize(show,show,cv::Size(700,683));
	//cv::imshow("show", show);
	cv::imwrite("image.png",show);
	//cv::waitKey(0);
	return a.exec();
}
