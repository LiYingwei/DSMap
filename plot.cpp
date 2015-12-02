#include "ywmap.h"
cv::Mat YWMap::Plot(point p, double l, double scale)
{
	double scalex = scale, scaley = cos(p.get<0>()) * scale;
	int r = l * scalex, c = l * scaley;
#ifdef INFO
	printf("%d %d\n", r, c);
#endif
	cv::Mat ret(r,c,CV_8UC3,Scalar(0xf0,0xf5,0xf7));
	box query_box(point(p.get<0>() - l, p.get<1>()), point(p.get<0>(), p.get<1>() + l));
	std::vector<std::pair<point, unsigned>> nodes;
	way_node_tree.query(bgi::intersects(query_box), std::back_inserter(nodes));

////////////////////////分层////////////////////////
	std::vector<std::pair<int, unsigned>> layerSorted;
	for(int i=0;i < nodes.size(); i++)
	{
		//for(int )
		layerSorted.push_back(std::make_pair());
	return ret;
}
