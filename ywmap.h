#ifndef YWMAP_H
#define YWMAP_H
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
	point p;
	pugi::xml_node node;
	std::vector<pugi::xml_node> way;
	std::vector<pugi::xml_node> relation;
	node_struct(point p,pugi::xml_node node):p(p),node(node)
	{
		way.clear();  //存的是在way中节点的信息
		relation.clear();
	}
};

struct way_struct
{
	pugi::xml_node way;
	std::vector<pugi::xml_node> relation;
	way_struct(pugi::xml_node way):way(way)
	{
		relation.clear();
	}
};


class YWMap
{
public:
	YWMap();
	void setDocosm(char* path);
	void loadMap(); //load map node to rtree, way and relation to map(id -> xml_node)
	cv::Mat Plot(point p, double l, double scale);
	//void shortestPathPreprocessor();
	//void shortestPath();
private:
	pugi::xml_document doc_osm;
	bgi::rtree< nodeinfo, bgi::quadratic<16> > nodertree;
	std::map<unsigned, unsigned> nodemap;
	std::map<unsigned, unsigned> waymap;
	std::vector<node_struct> nodevec;
	std::vector<way_struct> wayvec;
	cv::Point2d p2P(point v, point p, double scale);
	void plotline(cv::Mat &m, point start, point end, point p, double l, double scale,
				  cv::Scalar color, int thickness, int lineType=8);
};

#endif // YWMAP_H
