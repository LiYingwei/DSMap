#ifndef YWMAP_H
#define YWMAP_H
#define DEBUG
#define INFO
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
	std::vector<pugi::xml_node> nd_in_way;
	node_struct(unsigned id, point p, pugi::xml_node node, bool isway, bool isbuilding):
		id(id),p(p),node(node),isway(isway),isbuilding(isbuilding)
	{
		nd_in_way.clear();  //存的是在way中节点的信息
	}
};

struct way_struct
{
	unsigned id;
	int layer;
	std::string type;
	cv::Scalar color,ccolor;
	int thickness, boundthick;
	way_struct(unsigned id, int layer, std::string type, cv::Scalar color, cv::Scalar ccolor, int thickness, int boundthick):
		id(id),layer(layer),type(type),color(color),ccolor(ccolor),thickness(thickness),boundthick(boundthick)
	{
	}
};

class YWMap
{
public:
	void setDocosm(char* path);
	void loadPlotConf();
	void loadMap(); //load map node to rtree, way and relation to map(id -> xml_node)
	cv::Mat Plot(point p, double l, double scale);
private:
	pugi::xml_document doc_osm,doc_plot_conf;

	std::vector<node_struct> nodevec;
	std::map<unsigned, unsigned> nodemap; // id -> index
	std::vector<way_struct> wayvec;
	std::map<unsigned, unsigned> waymap;

	std::map<std::pair<std::string,std::string>, unsigned> layermap;
	std::vector<std::map<std::string,std::string>> layervec;
	std::map<std::pair<std::string,std::string>, unsigned> elementmap;
	std::vector<std::map<std::string,std::string>> elementvec;

	bgi::rtree< std::pair<point,unsigned> , bgi::quadratic<16> > way_node_tree;  // point -> index of nodevec

	cv::Scalar hex2BGR(std::string hex);
	cv::Point2d p2P(point v, point p, double scale);

	void plotLineBound(cv::Mat &ret,pugi::xml_node nd, point p, double l, double scale);
	void plotLineFill(cv::Mat &ret,pugi::xml_node nd, point p, double l, double scale);
	void plotline(cv::Mat &m, point start, point end, point p, double l, double scale,
				  cv::Scalar color, int thickness, int lineType=CV_AA);

	/*void plotPolyLayer(cv::Mat &img, pugi::xml_node way, point p, double scale);
	void plotPolyElement(cv::Mat &img, pugi::xml_node way, point p, double scale);
	void plotLineLayer(pugi::xml_node nd);
	void plotLineElement(pugi::xml_node nd);*/
};

#endif // YWMAP_H
