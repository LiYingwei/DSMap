#ifndef YWMAP_H
#define YWMAP_H
//#define DEBUG
//#define INFO
#include <string>
#include <vector>
#include <core/core.hpp>
#include <highgui/highgui.hpp>
#include <imgproc/imgproc.hpp>
#include <geometry.hpp>
#include <geometry/geometries/point.hpp>
#include <geometry/index/rtree.hpp>
#include <geometry/geometries/box.hpp>
#include <pugixml.hpp>

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
	bool isway,isbuilding,isline;
	int fanout;
	std::vector<pugi::xml_node> nd_in_way;
	int level;
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
	std::string name;
	double dist;
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
	double dist, time, slowtime;
	unsigned wayid;
	edge(unsigned from,unsigned to, double dist, double time, double slowtime, unsigned wayid):from(from),to(to),dist(dist),time(time),slowtime(slowtime),wayid(wayid){}
};

struct taxi_struct
{
	int id;
	short hh,mm,ss;
	short alert;
	double lon,lat;
	short empty;
	char light;
	short high, brake;
	double v,dir;
	short satellite;
	taxi_struct(){}
	taxi_struct(int id,short hh,short mm,short ss,short alert,double lon,double lat,
			short empty,char light,short high,short brake,double v,double dir, short satellite):
		id(id), hh(hh), mm(mm), ss(ss), alert(alert), lon(lon), lat(lat), empty(empty), light(light),
		high(high), brake(brake), v(v), dir(dir), satellite(satellite){}
	taxi_struct(int id,short hh,short mm,short ss):id(id),hh(hh),mm(mm),ss(ss){}
	taxi_struct(int id):id(id){}
	void read(char* buf)
	{
		char t[30],date[15],stime[15];
		sscanf(buf,"%d,%[^,],%hd,%lf,%lf,%hd,%c,%hd,%hd,%lf,%lf,%hd",
				   &id,t,&alert,&lon,&lat,&empty,&light,&high,&brake,&v,&dir,&satellite);
		sscanf(t,"%s%s",date,stime);
		sscanf(stime,"%hd:%hd:%hd", &hh,&mm,&ss);
	};
	void print()
	{
		printf("%d,%hd:%hd:%hd,%hd,%lf,%lf,%hd,%c,%hd,%hd,%lf,%lf,%hd\n",
				id,hh,mm,ss,alert,lon,lat,empty,light,high,brake,v,dir,satellite);
	}
	bool operator<(const taxi_struct& o) const
	{
		return lon < o.lon;
	}
};

class YWMap
{
public:
	void setDocosm(char* path);
	void loadPlotConf();
	void loadSpeedConf();
	void loadMap(); //load map node to rtree, way and relation to map(id -> xml_node)
	void loadtaxi();
	cv::Mat Plot(point p, double l, double scale, double factor);
	cv::Mat Plot(point p, int level, int div = 1);
	std::vector<unsigned> AStarDist(unsigned s,unsigned t);
	std::vector<unsigned> AStarTime(unsigned s,unsigned t, std::set<unsigned> slowset);
	std::vector<unsigned> SPFA(unsigned s, unsigned t);
	std::vector<unsigned> SPFATime(unsigned s, unsigned t, std::set<unsigned> slowset);
	std::vector<unsigned> dijkstraDist(unsigned s, unsigned t);
	std::vector<unsigned> dijkstraTime(unsigned s, unsigned t, std::set<unsigned> slowset);
	cv::Mat PlotShortestPath(std::vector<unsigned> total_path, cv::Scalar color=cv::Scalar(0x00,0x00,0xFF));
	cv::Mat PlotPath(std::vector<point> total_path, cv::Scalar color=cv::Scalar(0x00,0x00,0xFF));
	cv::Mat PlotPointInBox(box b, std::vector<std::pair<point,unsigned>> nodes, cv::Scalar color = cv::Scalar(0xFF, 0xCC, 0xCC));
	cv::Mat PlotPointNearest(point po, std::vector<std::pair<point,unsigned>> nodes, cv::Scalar color = cv::Scalar(0xFF, 0xCC, 0xCC));
	//void PlotShortestPath(cv::Mat &ret, std::vector<unsigned> total_path, point p, int level, cv::Scalar color=cv::Scalar(0xfa,0x9e,0x25));
	////////////////////////ui////////////////////////
	static void cmd_showmap();
	static void cmd_shortestpath();
	static void cmd_querybox();
	static void cmd_querynearest();
	static void cmd_queryname();
	static void cmd_queryway();
	static void cmd_querytaxi();
	//////////////visit private element///////////////
	unsigned getNodeIndexById(unsigned id);
	unsigned getNodeIdByIndex(unsigned index);
	//////////////////////query///////////////////////
	std::vector<std::pair<point,unsigned>> querybox(box b);
	std::vector<std::pair<std::string,point>> queryName(char *P);
	std::vector<std::pair<std::string,unsigned>> queryNameWay(char *P);
	std::vector<unsigned> nearest(point p, int k = 1);
	std::vector<std::pair<point, unsigned>> nearestForPlot(point p, int k = 1);



private:
	//bool sortLayerCmp(std::pair<int,std::pair<box, unsigned>> a,std::pair<int,std::pair<box, unsigned>> b);
	pugi::xml_document doc_osm,doc_plot_conf,doc_speed_conf;
	//double scalex,scaley,l;
	//point p;

	///////////////////////////data struct///////////////////
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
	bgi::rtree< std::pair<point,unsigned> , bgi::quadratic<16> > water_line_tree;
	bgi::rtree< std::pair<box,unsigned> , bgi::quadratic<16> > build_tree;  // box -> index of buildvec

	/////////////////////////Plot////////////////////////////
	void PlotWay(cv::Mat& ret, point p, double l, double scalex, double scaley, double factor);
	void PlotWaterWay(cv::Mat& ret, point p, double l, double scalex, double scaley, double factor);
	void PlotContour(cv::Mat& ret, point p, double l, double scalex, double scaley, double factor);

	cv::Scalar hex2BGR(std::string hex);
	cv::Point2d p2P(point v, point p, double scalex, double scaley);

	void plotLineBound(cv::Mat &ret, pugi::xml_node nd, point p, double l, double scalex, double scaley, double factor);
	void plotLineFill(cv::Mat &ret, pugi::xml_node nd, point p, double l, double scalex, double scaley, double factor);
	void plotline(cv::Mat &m, point start, point end, point p, double l, double scalex, double scaley,
				  cv::Scalar color, int thickness, int lineType=CV_AA);
	void plotPoly(cv::Mat &img, pugi::xml_node way, point p, double scalex, double scaley, cv::Scalar color, cv::Scalar ccolor, int boundthick);

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
	void addEdge(unsigned indexfrom, unsigned indexto, double speed, double slowspeed, bool oneway, unsigned wayid);
	std::map<unsigned,unsigned> Came_From;
	std::vector<unsigned> reconstruct_path(unsigned current);
	//////////////////////后缀数组相关////////////////////////
	std::vector<std::pair<std::string, point>> nameList;
	int SA_n;
	char s[500000];
	int index[500000], sa[500000];
	void build_sa();
	int cmp_suffix(char *pattern, int p, int m);
	std::vector<std::pair<std::string, unsigned>> nameListWay;
	int SA_nWay;
	char sWay[500000];
	int indexWay[500000], saWay[500000];
	void build_saWay();
	int cmp_suffixWay(char *pattern, int p, int m);
	//////////////////////////taxi related////////////////////
	std::vector<taxi_struct> taxiinfo;
	static int timecmp(short h1,short m1,short s1,short h2,short m2,short s2);

};

#endif // YWMAP_H
