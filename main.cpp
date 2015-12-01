#include <QCoreApplication>
#include "ywmap.h"
YWMap map;
int main(int argc, char *argv[])
{
	QCoreApplication a(argc, argv);
	/*map.setDocosm("map.osm");
	map.loadMap();
	map.Plot(point(31.20,121.585),0.01, 70000);*/
	return a.exec();
}
