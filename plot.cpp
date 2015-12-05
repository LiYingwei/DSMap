#include "ywmap.h"
cv::Mat YWMap::Plot(point p, double l, double scale)
{
	double scalex = scale, scaley = cos(p.get<0>()) * scale;
	int r = l * scalex, c = l * scaley;
#ifdef INFO
	printf("%d %d\n", r, c);
#endif
	cv::Mat ret(r,c,CV_8UC3,cv::Scalar(0xf0,0xf5,0xf7));
	box query_box(point(p.get<0>() - l, p.get<1>()), point(p.get<0>(), p.get<1>() + l));
	std::vector<std::pair<point, unsigned>> nodes;  //point -> index of nodevec
	way_node_tree.query(bgi::intersects(query_box), std::back_inserter(nodes));
#ifdef INFO
	printf("node size = %d\n", nodes.size());
#endif
////////////////////////分层////////////////////////
	std::vector<std::pair<int, pugi::xml_node>> layerSorted; //layer, nd_in_way
	for(int i=0;i < nodes.size(); i++)
	{
		node_struct &node = nodevec[nodes[i].second];
		//if(node.isway == false) continue;
		for(pugi::xml_node nd : node.nd_in_way)
		{
			unsigned wayid = nd.parent().attribute("id").as_uint(-1);
#ifdef DEBUG
			assert(wayid != -1);
			//assert(waymap.find(wayid) != waymap.end() || printf("%u %u\n", wayid,nodes[i].second));
#endif
			layerSorted.push_back(std::make_pair(wayvec[waymap[wayid]].layer, nd));
		}
	}
	std::sort(layerSorted.begin(),layerSorted.end());
#ifdef DEBUG
	assert(layerSorted.size()!=0);
#endif
#ifdef INFO
	printf("layer size = %d\n", layerSorted.size());
#endif
	int nowlayer = layerSorted[0].first;
	for(int i=0; i < layerSorted.size();)
	{
		int j;
		for(j=i;j< layerSorted.size() && layerSorted[j].first == nowlayer; j++)
			plotLineBound(ret,layerSorted[j].second, p, l, scale);
		for(j=i;j< layerSorted.size() && layerSorted[j].first == nowlayer; j++)
			plotLineFill(ret,layerSorted[j].second, p, l, scale);
		if(j==layerSorted.size())break;
		nowlayer = layerSorted[j].first;
		i = j;
	}
	//cv::imshow("map", ret);
	//cv::waitKey(0);
	return ret;
}
