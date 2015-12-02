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
//	map.Plot(point(31.20,121.585),0.01, 70000);
	printf("Plot Finished!\n");
	return a.exec();
}
