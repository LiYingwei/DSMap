#ifndef YWMAP_H
#define YWMAP_H
#define DEBUG
//#define INFO
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/box.hpp>
#include "Libs/pugixml/pugixml.hpp"

#define maxn 600000
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<point, unsigned> nodeinfo;

struct node_struct
{
	unsigned id;
	point p;//lat,lon
	pugi::xml_node node;
	bool isway;
	bool isbuilding;
	int fanout;
	std::vector<pugi::xml_node> nd_in_way;
	node_struct(unsigned id, point p, pugi::xml_node node, bool isway, bool isbuilding):
		id(id),p(p),node(node),isway(isway),isbuilding(isbuilding)
	{
		nd_in_way.clear();  //存的是在way中节点的信息
		fanout = 0;
	}
};

struct way_struct
{
	unsigned id;
	int layer;
	std::string type;
	cv::Scalar color,ccolor;
	int thickness, boundthick;
	double speed, slowspeed;
	bool oneway;
	way_struct(unsigned id, int layer, std::string type, cv::Scalar color, cv::Scalar ccolor, int thickness, int boundthick, double speed, double slowspeed,bool oneway = false):
		id(id),layer(layer),type(type),color(color),ccolor(ccolor),thickness(thickness),boundthick(boundthick),speed(speed), slowspeed(slowspeed), oneway(oneway)
	{
	}
};

struct building_struct
{
	unsigned id;
	int layer;
	std::string type, subtype;
	cv::Scalar color,ccolor;
	int boundthick;
	pugi::xml_node building;
	box b;
	building_struct(unsigned id, int layer, std::string type, std::string subtype,
					cv::Scalar color, cv::Scalar ccolor, int boundthick,pugi::xml_node building, box b):
		id(id),layer(layer),type(type),subtype(subtype),color(color),ccolor(ccolor),boundthick(boundthick),building(building),b(b)
	{
	}
};

struct edge
{
	unsigned from, to;
	double dist, time;
	edge(unsigned from,unsigned to, double dist, double time):from(from),to(to),dist(dist),time(time){}
};

class YWMap
{
public:
	void setDocosm(char* path);
	void loadPlotConf();
	void loadMap(); //load map node to rtree, way and relation to map(id -> xml_node)
	cv::Mat Plot(point p, double l, double scale);
	void loadSpeedConf();
	std::vector<unsigned> AStarDist(unsigned s,unsigned t);
	std::vector<unsigned> SPFA(unsigned s,unsigned t);
	void PlotShortestPath(cv::Mat &ret, std::vector<unsigned> total_path, cv::Scalar color=cv::Scalar(0xfa,0x9e,0x25));
private:
	//bool sortLayerCmp(std::pair<int,std::pair<box, unsigned>> a,std::pair<int,std::pair<box, unsigned>> b);
	pugi::xml_document doc_osm,doc_plot_conf,doc_speed_conf;
	double scalex,scaley,l;
	point p;

	std::vector<node_struct> nodevec;
	std::map<unsigned, unsigned> nodemap; // id -> index
	std::vector<way_struct> wayvec;
	std::map<unsigned, unsigned> waymap;
	std::vector<building_struct> buildvec;
	std::map<unsigned, unsigned> buildmap;

	std::map<std::pair<std::string,std::string>, unsigned> layermap;
	std::vector<std::map<std::string,std::string>> layervec;
	std::map<std::pair<std::string,std::string>, unsigned> elementmap;
	std::vector<std::map<std::string,std::string>> elementvec;

	bgi::rtree< std::pair<point,unsigned> , bgi::quadratic<16> > way_node_tree;  // point -> index of nodevec
	bgi::rtree< std::pair<box,unsigned> , bgi::quadratic<16> > build_tree;  // box -> index of buildvec

	void PlotWay(cv::Mat& ret,point p, double l, double scale);
	void PlotContour(cv::Mat& ret, point p, double l, double scale);

	cv::Scalar hex2BGR(std::string hex);
	cv::Point2d p2P(point v, point p, double scalex, double scaley);

	void plotLineBound(cv::Mat &ret,pugi::xml_node nd, point p, double l);
	void plotLineFill(cv::Mat &ret,pugi::xml_node nd, point p, double l);
	void plotline(cv::Mat &m, point start, point end, point p, double l,
				  cv::Scalar color, int thickness, int lineType=CV_AA);
	void plotPoly(cv::Mat &img, pugi::xml_node way, point p, cv::Scalar color, cv::Scalar ccolor, int boundthick);

	/*void plotPolyLayer(cv::Mat &img, pugi::xml_node way, point p, double scale);
	void plotPolyElement(cv::Mat &img, pugi::xml_node way, point p, double scale);
	void plotLineLayer(pugi::xml_node nd);
	void plotLineElement(pugi::xml_node nd);*/
	//////////////////////////最短路相关//////////////////////
	std::map<std::pair<std::string,std::string>, double> normalSpeed;
	std::map<std::pair<std::string,std::string>, double> slowSpeed;
	std::vector<unsigned> importantnode; //index 没用了
	std::map<unsigned,unsigned> importantnodes; // id -> index 放弃了
	std::vector<unsigned> G[maxn];
	std::vector<edge>E;
	double nodeDist(point p1, point p2);
	void addEdge(unsigned indexfrom, unsigned indexto, double speed, double slowspeed, bool oneway);
	std::map<unsigned,unsigned> Came_From;
	std::vector<unsigned> reconstruct_path(unsigned current);
};

#endif // YWMAP_H
