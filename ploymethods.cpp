#include "ywmap.h"
/*using namespace std;
using namespace cv;
using namespace pugi;
void YWMap::plotPolyLayer(cv::Mat &img, xml_node way, point p, double scale)
{
	//do nothing.
}

void YWMap::plotPolyElement(cv::Mat &img, xml_node way, point p, double scale)
{
	vector<vector<cv::Point>>contourElement(1);
	for(pugi::xml_node nd = way.first_child(); nd; nd = nd.next_sibling())
	{
		point v = nodevec[nodemap[nd.attribute("ref").as_uint()]].p;
		contourElement[0].push_back(p2P(v,p,scale));
	}
	vector<cv::Point> tmp = contourElement.at(0);
	const cv::Point* elementPoints[1] = { &tmp[0] };
	int numberOfPoints = (int)tmp.size();
	cv::fillPoly(img, elementPoints, &numberOfPoints, 1, Scalar (0, 0, 255), 8);
}

void YWMap::plotLineElement(xml_node nd)
{
	// TODO :
}

void YWMap::plotLineLayer(xml_node nd)
{
	// TODO :
}
*/
void YWMap::plotline(cv::Mat &m, point s, point t, point p, double l, double scale, cv::Scalar color, int thickness, int lineType)
{
#ifdef PLOTLINE
	printf("ploting a line\n");
#endif
	cv::Point2d start = p2P(s,p,scale);
	cv::Point2d end = p2P(t,p,scale);
	cv::line(m, start, end, color, thickness, lineType);
}

void YWMap::plotLineBound(cv::Mat &ret, pugi::xml_node nd, point p, double l, double scale)
{
	unsigned wayid = nd.parent().attribute("id").as_uint(-1);
#ifdef DEBUG
	assert(wayid != -1);
#endif
	way_struct way = wayvec[waymap[wayid]];
	point u = nodevec[nodemap[nd.attribute("ref").as_uint()]].p;

	pugi::xml_node nextpoint = nd.next_sibling();
	if(strcmp(nextpoint.name(),"nd") == 0)
	{
		point v = nodevec[nodemap[nextpoint.attribute("ref").as_uint()]].p;
		plotline(ret, u, v, p, l, scale, way.ccolor, way.thickness + 2 * way.boundthick, CV_AA);
	}

	if(nd.parent().first_child() != nd)
	{
		pugi::xml_node prepoint = nd.previous_sibling();
		point v = nodevec[nodemap[prepoint.attribute("ref").as_uint()]].p;
		if(v.get<0>() > p.get<0>() || v.get<1>() < p.get<1>() ||
								v.get<0>() < p.get<0>() - l || v.get<1>() > p.get<1>() + l)
			plotline(ret, u, v, p, l, scale, way.ccolor, way.thickness + 2 * way.boundthick, CV_AA);
	}
}

void YWMap::plotLineFill(cv::Mat &ret, pugi::xml_node nd, point p, double l, double scale)
{
	unsigned wayid = nd.parent().attribute("id").as_uint(-1);
#ifdef DEBUG
	assert(wayid != -1);
#endif
	way_struct way = wayvec[waymap[wayid]];
	point u = nodevec[nodemap[nd.attribute("ref").as_uint()]].p;

	pugi::xml_node nextpoint = nd.next_sibling();
	if(strcmp(nextpoint.name(),"nd") == 0)
	{
		point v = nodevec[nodemap[nextpoint.attribute("ref").as_uint()]].p;
		plotline(ret, u, v, p, l, scale, way.color, way.thickness, CV_AA);
	}

	if(nd.parent().first_child() != nd)
	{
		pugi::xml_node prepoint = nd.previous_sibling();
		point v = nodevec[nodemap[prepoint.attribute("ref").as_uint()]].p;
		if(v.get<0>() > p.get<0>() || v.get<1>() < p.get<1>() ||
								v.get<0>() < p.get<0>() - l || v.get<1>() > p.get<1>() + l)
			plotline(ret, u, v, p, l, scale, way.color, way.thickness, CV_AA);
	}
}
