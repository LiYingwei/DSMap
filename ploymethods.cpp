#include "ywmap.h"
/*using namespace std;
using namespace cv;
using namespace pugi;
void YWMap::plotPolyLayer(cv::Mat &img, xml_node way, point p, double scale)
{
	//do nothing.
}*/

/*static cv::Scalar random_color(cv::RNG& rng)
{
	int icolor = (unsigned)rng;

	return Scalar(icolor&0xFF, (icolor>>8)&0xFF, (icolor>>16)&0xFF);
}*/

void YWMap::plotPoly(cv::Mat &img, pugi::xml_node way, point p, double scalex, double scaley,cv::Scalar color,cv::Scalar ccolor, int boundthick)
{
	std::vector<std::vector<cv::Point>>contourElement(1);
	for(pugi::xml_node nd = way.first_child(); nd; nd = nd.next_sibling())
	{
		if(nd.attribute("ref").as_uint()==0)break;
		point v = nodevec[nodemap[nd.attribute("ref").as_uint()]].p;
		contourElement[0].push_back(p2P(v,p, scalex, scaley));
	}
	std::vector<cv::Point> tmp = contourElement.at(0);
	const cv::Point* elementPoints[1] = { &tmp[0] };
	int numberOfPoints = (int)tmp.size();
	cv::fillPoly(img, elementPoints, &numberOfPoints, 1, color, CV_AA);
}

void YWMap::plotline(cv::Mat &m, point s, point t, point p, double l, double scalex, double scaley, cv::Scalar color, int thickness, int lineType)
{
#ifdef PLOTLINE
	printf("ploting a line\n");
#endif
	cv::Point2d start = p2P(s,p,scalex,scaley);
	cv::Point2d end = p2P(t,p,scalex,scaley);
	cv::line(m, start, end, color, thickness, lineType);
}

void YWMap::plotLineBound(cv::Mat &ret, pugi::xml_node nd, point p, double l, double scalex, double scaley, double factor)
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
		plotline(ret, u, v, p, l, scalex, scaley, way.ccolor, factor * way.thickness + 2 * way.boundthick, CV_AA);
	}

	if(nd.parent().first_child() != nd)
	{
		pugi::xml_node prepoint = nd.previous_sibling();
		point v = nodevec[nodemap[prepoint.attribute("ref").as_uint()]].p;
		if(v.get<0>() > p.get<0>() || v.get<1>() < p.get<1>() ||
								v.get<0>() < p.get<0>() - l || v.get<1>() > p.get<1>() + l)
			plotline(ret, u, v, p, l, scalex, scaley, way.ccolor, factor * way.thickness + 2 * way.boundthick, CV_AA);
	}
}

void YWMap::plotLineFill(cv::Mat &ret, pugi::xml_node nd, point p, double l,double scalex, double scaley, double factor)
{
	unsigned wayid = nd.parent().attribute("id").as_uint(-1);
#ifdef DEBUG
	assert(wayid != -1);
#endif
	way_struct way = wayvec[waymap[wayid]];
	node_struct &node = nodevec[nodemap[nd.attribute("ref").as_uint()]];
	point u = node.p;

	pugi::xml_node nextpoint = nd.next_sibling();
	if(strcmp(nextpoint.name(),"nd") == 0)
	{
		point v = nodevec[nodemap[nextpoint.attribute("ref").as_uint()]].p;
		plotline(ret, u, v, p, l, scalex, scaley, way.color, way.thickness * factor, CV_AA);
	}

	if(nd.parent().first_child() != nd)
	{
		pugi::xml_node prepoint = nd.previous_sibling();
		point v = nodevec[nodemap[prepoint.attribute("ref").as_uint()]].p;
		if(v.get<0>() > p.get<0>() || v.get<1>() < p.get<1>() ||
								v.get<0>() < p.get<0>() - l || v.get<1>() > p.get<1>() + l)
			plotline(ret, u, v, p, l, scalex, scaley,way.color, way.thickness * factor, CV_AA);
	}

	/*if(nd.parent().first_child() == nd || nd.next_sibling().value()[0] != 'n')
	{
#ifdef DEBUG
		cv::circle(ret, p2P(u,p,scale),5,cv::Scalar(0,0,255),1,CV_AA);
		printf("nodeid:%u\n",nd.attribute("ref").as_uint());
#endif
		for(pugi::xml_node Nd : node.nd_in_way)
		{
			printf("	wayid=%u\n",Nd.parent().attribute("id").as_uint());
			//if(Nd != nd)
			if(Nd == Nd.parent().first_child())
			{
				pugi::xml_node nextpoint = Nd.next_sibling();
				printf("Nd=%u next=%u\n", Nd.attribute("ref").as_uint(), nextpoint.attribute("ref").as_uint());
				point v = nodevec[nodemap[nextpoint.attribute("ref").as_uint()]].p;
				if(v.get<0>() > p.get<0>() || v.get<1>() < p.get<1>() ||
										v.get<0>() < p.get<0>() - l || v.get<1>() > p.get<1>() + l)
					plotline(ret, u, v, p, l, scale, way.color, way.thickness, CV_AA);
			}
			else if(strcmp(Nd.next_sibling().value(),"nd") != 0)
			{
				pugi::xml_node prepoint = Nd.previous_sibling();
				point v = nodevec[nodemap[prepoint.attribute("ref").as_uint()]].p;
				if(v.get<0>() > p.get<0>() || v.get<1>() < p.get<1>() ||
										v.get<0>() < p.get<0>() - l || v.get<1>() > p.get<1>() + l)
					plotline(ret, u, v, p, l, scale, way.color, way.thickness, CV_AA);
			}
		}

	}*/
}
