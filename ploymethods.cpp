/*#include "ywmap.h"
using namespace std;
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

void YWMap::plotline(Mat &m, point s, point t, point p, double l, double scale, Scalar color, int thickness, int lineType)
{
	cv::Point2d start = p2P(s,p,scale);
	cv::Point2d end = p2P(t,p,scale);
	cv::line(m, start, end, color, thickness, lineType);
}
*/
