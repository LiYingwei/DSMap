#include "ywmap.h"
cv::Mat YWMap::Plot(point p, double l, double scale,double factor)
{
	double scalex = scale, scaley = cos(p.get<0>()) * scale;
	int r = l * scalex, c = l * scaley;
//	printf("l = %f scalex = %f scaley = %f\n", l, scalex, scaley);
//	this->p = p;this->l = l;this -> scalex = scalex;this->scaley = scaley;
#ifdef INFO
	printf("%d %d\n", r, c);
#endif
	cv::Mat ret(r,c,CV_8UC3,cv::Scalar(0xf0,0xf5,0xf7));
	PlotContour(ret, p, l, scalex, scaley, factor);
	PlotWaterWay(ret, p, l, scalex, scaley, factor);
	PlotWay(ret, p, l, scalex, scaley, factor);
	//cv::imshow("map", ret);
	//cv::waitKey(0);
	return ret;
}
cv::Mat YWMap::Plot(point p, int level, int div)
{
	clock_t t = clock();
	double cols = 2560;
	double l;
	switch(level)
	{
	case 18:
		l = 0.005; break;
	case 17:
		l = 0.01; break;
	case 16:
		l = 0.02; break;
	case 15:
		l = 0.04; break;
	case 14:
		l = 0.08; break;
	case 13:
		l = 0.16; break;
	case 12:
		l = 0.32; break;
	case 11:
		l = 0.64; break;
	}
	p.set<0>(p.get<0>() + l/2);
	p.set<1>(p.get<1>() - l/2);
	cv::Mat ret = Plot(p, l/div, cols/l, pow(0.04,0.7) / pow(l,0.7));
	t = clock() - t;
	printf("Plot takes %.4f seconds\n", (float)t/CLOCKS_PER_SEC);
	return ret;
}

bool sortLayerCmp(std::pair<int,std::pair<box, unsigned>> a,std::pair<int,std::pair<box, unsigned>> b)
{
	return a.first < b.first;
}

void YWMap::PlotContour(cv::Mat &ret, point p, double l, double scalex, double scaley, double factor)
{
	box query_box(point(p.get<0>() - l, p.get<1>()), point(p.get<0>(), p.get<1>() + l));
	std::vector<std::pair<box, unsigned>> builds;
	build_tree.query(bgi::intersects(query_box), std::back_inserter(builds));
	std::vector<std::pair<int,std::pair<box,unsigned>>> tmp;
	for(auto build: builds) tmp.push_back(std::make_pair(buildvec[build.second].layer,build));
	std::sort(tmp.begin(), tmp.end(), sortLayerCmp);
	builds.clear();
	for(auto t: tmp)builds.push_back(t.second);
	for(auto build : builds)
	{
		building_struct &b = buildvec[build.second];
		plotPoly(ret, b.building, p, scalex, scaley,b.color,b.ccolor,b.boundthick * factor);
	}
}

void YWMap::PlotWaterWay(cv::Mat &ret, point p, double l, double scalex, double scaley, double factor)
{
	box query_box(point(p.get<0>() - l, p.get<1>()), point(p.get<0>(), p.get<1>() + l));
	std::vector<std::pair<point, unsigned>> nodes;  //point -> index of nodevec
	water_line_tree.query(bgi::intersects(query_box), std::back_inserter(nodes));
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
	if(layerSorted.size() == 0)return;
#ifdef DEBUG
	//assert(layerSorted.size()!=0);
#endif
#ifdef INFO
	printf("layer size = %d\n", layerSorted.size());
#endif
	int nowlayer = layerSorted[0].first;
	for(int i=0; i < layerSorted.size();)
	{
		int j;
		for(j=i;j< layerSorted.size() && layerSorted[j].first == nowlayer; j++)
			plotLineBound(ret,layerSorted[j].second, p, l, scalex, scaley, factor);
		for(j=i;j< layerSorted.size() && layerSorted[j].first == nowlayer; j++)
			plotLineFill(ret,layerSorted[j].second, p, l, scalex, scaley, factor);
		if(j==layerSorted.size())break;
		nowlayer = layerSorted[j].first;
		i = j;
	}
}

void YWMap::PlotWay(cv::Mat &ret, point p, double l, double scalex, double scaley, double factor)
{
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
	if(layerSorted.size() == 0)return;
#ifdef DEBUG
	//assert(layerSorted.size()!=0);
#endif
#ifdef INFO
	printf("layer size = %d\n", layerSorted.size());
#endif
	int nowlayer = layerSorted[0].first;
	for(int i=0; i < layerSorted.size();)
	{
		int j;
		for(j=i;j< layerSorted.size() && layerSorted[j].first == nowlayer; j++)
			plotLineBound(ret,layerSorted[j].second, p, l, scalex, scaley, factor);
		for(j=i;j< layerSorted.size() && layerSorted[j].first == nowlayer; j++)
			plotLineFill(ret,layerSorted[j].second, p, l, scalex, scaley, factor);
		if(j==layerSorted.size())break;
		nowlayer = layerSorted[j].first;
		i = j;
	}

}

cv::Mat YWMap::PlotPointNearest(point po, std::vector<std::pair<point,unsigned>> nodes, cv::Scalar color)
{
	assert(nodes.size() != 0);
	double minlat = nodes[0].first.get<0>(),
			minlon = nodes[0].first.get<1>(),
			maxlat = nodes[0].first.get<0>(),
			maxlon = nodes[0].first.get<1>();
	for(int i = 1; i < nodes.size(); i++)
	{
		minlat = fmin(minlat, nodes[i].first.get<0>());
		maxlat = fmax(maxlat, nodes[i].first.get<0>());
		minlon = fmin(minlon, nodes[i].first.get<1>());
		maxlon = fmax(maxlon, nodes[i].first.get<1>());
	}
	double l = fmax(maxlat - minlat, maxlon - minlon) * 1.3;
	int level=18;
	if(l>0.005)level = 17;
	if(l>0.01) level = 16;
	if(l>0.02) level = 15;
	if(l>0.04) level = 14;
	if(l>0.08) level = 13;
	if(l>0.16) level = 12;
	if(l>0.32) level = 11;
	point P = point((maxlat + minlat)/2, (maxlon + minlon)/2);
	//printf("%f %f===level = %d \n",(maxlat + minlat)/2, (maxlon + minlon)/2, level);
	cv::Mat ret = Plot(P,level);
	l = 1310.72 * pow(2.0 ,-level);
	point p(P.get<0>()+l/2,P.get<1>()-l/2);
	double scale = 2560/l;
	double scalex = scale;
	double scaley = scale * cos(p.get<0>());

	cv::circle(ret, p2P(po, p, scalex, scaley), 6, cv::Scalar(0,0,255), 3, CV_AA);
	for(auto node: nodes)
	{
		cv::circle(ret, p2P(node.first, p, scalex, scaley), 1, color, 3, CV_AA);
	}
	return ret;
}

cv::Mat YWMap::PlotPointInBox(box b, std::vector<std::pair<point, unsigned> > nodes, cv::Scalar color)
{
	double minlat = b.min_corner().get<0>(),
			minlon = b.min_corner().get<1>(),
			maxlat = b.max_corner().get<0>(),
			maxlon = b.max_corner().get<1>();
	double l = fmax(maxlat - minlat, maxlon - minlon) * 1.3;
	int level=18;
	if(l>0.005)level = 17;
	if(l>0.01) level = 16;
	if(l>0.02) level = 15;
	if(l>0.04) level = 14;
	if(l>0.08) level = 13;
	if(l>0.16) level = 12;
	if(l>0.32) level = 11;
	point P = point((maxlat + minlat)/2, (maxlon + minlon)/2);
	cv::Mat ret = Plot(P,level);
	l = 1310.72 * pow(2.0 ,-level);
	point p(P.get<0>()+l/2,P.get<1>()-l/2);
	double scale = 2560/l;
	double scalex = scale;
	double scaley = scale * cos(p.get<0>());

	plotline(ret, point(minlat,minlon), point(minlat, maxlon), p, l, scalex, scaley , color, 3, CV_AA);
	plotline(ret, point(maxlat,minlon), point(maxlat, maxlon), p, l, scalex, scaley , color, 3, CV_AA);
	plotline(ret, point(maxlat,minlon), point(minlat, minlon), p, l, scalex, scaley , color, 3, CV_AA);
	plotline(ret, point(maxlat,maxlon), point(minlat, maxlon), p, l, scalex, scaley , color, 3, CV_AA);

	for(auto node: nodes)
	{
		cv::circle(ret, p2P(node.first, p, scalex, scaley), 1, color, 3, CV_AA);
	}
	return ret;
}

cv::Mat YWMap::PlotShortestPath(std::vector<unsigned> total_path, cv::Scalar color)
{
	assert(total_path.size() != 0);
	double minlat = nodevec[total_path[0]].p.get<0>(),
			minlon = nodevec[total_path[0]].p.get<1>(),
			maxlat = nodevec[total_path[0]].p.get<0>(),
			maxlon = nodevec[total_path[0]].p.get<1>();
	for(int i = 1; i < total_path.size(); i++)
	{
		minlat = fmin(minlat, nodevec[total_path[i]].p.get<0>());
		maxlat = fmax(maxlat, nodevec[total_path[i]].p.get<0>());
		minlon = fmin(minlon, nodevec[total_path[i]].p.get<1>());
		maxlon = fmax(maxlon, nodevec[total_path[i]].p.get<1>());
	}
	double l = fmax(maxlat - minlat, maxlon - minlon) * 1.3;
	int level=18;
	if(l>0.005)level = 17;
	if(l>0.01) level = 16;
	if(l>0.02) level = 15;
	if(l>0.04) level = 14;
	if(l>0.08) level = 13;
	if(l>0.16) level = 12;
	if(l>0.32) level = 11;
	point P = point((maxlat + minlat)/2, (maxlon + minlon)/2);
	//printf("%f %f===level = %d \n",(maxlat + minlat)/2, (maxlon + minlon)/2, level);
	cv::Mat ret = Plot(P,level);
	l = 1310.72 * pow(2.0 ,-level);
	point p(P.get<0>()+l/2,P.get<1>()-l/2);
	double scale = 2560/l;
	double scalex = scale;
	double scaley = scale * cos(p.get<0>());
	for(int i = 0; i < total_path.size()-1; i++)
	{
		point s = nodevec[total_path[i]].p;
		point t = nodevec[total_path[i+1]].p;
		//printf("(%f,%f)(%f,%f)(%f,%f) l = %f scalex = %f scaley = %f\n",s.get<0>(), s.get<1>(), t.get<0>(), t.get<1>(), p.get<0>(), p.get<1>(), l,scalex,scaley);
		plotline(ret, s, t, p, l, scalex, scaley , color, 3, CV_AA);
	}
	//imwrite("out.png", ret);
	//cv::imshow("ret",ret);
	//cv::waitKey(0);
	return ret;
}

cv::Mat YWMap::PlotPath(std::vector<point> total_path, cv::Scalar color)
{
	//assert(total_path.size() != 0);
	if(total_path.size() == 0)
	{
		printf("Nothing to plot\n");
		return cv::Mat();
	}
	double minlat = total_path[0].get<0>(),
			minlon = total_path[0].get<1>(),
			maxlat = total_path[0].get<0>(),
			maxlon = total_path[0].get<1>();
	for(int i = 1; i < total_path.size(); i++)
	{
		minlat = fmin(minlat, total_path[i].get<0>());
		maxlat = fmax(maxlat, total_path[i].get<0>());
		minlon = fmin(minlon, total_path[i].get<1>());
		maxlon = fmax(maxlon, total_path[i].get<1>());
	}
	double l = fmax(maxlat - minlat, maxlon - minlon) * 1.3;
	int level=18;
	if(l>0.005)level = 17;
	if(l>0.01) level = 16;
	if(l>0.02) level = 15;
	if(l>0.04) level = 14;
	if(l>0.08) level = 13;
	if(l>0.16) level = 12;
	if(l>0.32) level = 11;
	point P = point((maxlat + minlat)/2, (maxlon + minlon)/2);
	//printf("%f %f===level = %d \n",(maxlat + minlat)/2, (maxlon + minlon)/2, level);
	cv::Mat ret = Plot(P,level);
	l = 1310.72 * pow(2.0 ,-level);
	point p(P.get<0>()+l/2,P.get<1>()-l/2);
	double scale = 2560/l;
	double scalex = scale;
	double scaley = scale * cos(p.get<0>());
	for(int i = 0; i < total_path.size()-1; i++)
	{
		point s = total_path[i];
		point t = total_path[i+1];
		//printf("(%f,%f)(%f,%f)(%f,%f) l = %f scalex = %f scaley = %f\n",s.get<0>(), s.get<1>(), t.get<0>(), t.get<1>(), p.get<0>(), p.get<1>(), l,scalex,scaley);
		plotline(ret, s, t, p, l, scalex, scaley , color, 3, CV_AA);
	}
	//imwrite("out.png", ret);
	//cv::imshow("ret",ret);
	//cv::waitKey(0);
	return ret;
}

/*void YWMap::PlotShortestPath(cv::Mat &ret, std::vector<unsigned> total_path, point p, int level, cv::Scalar color)
{
	double cols = 1900, rows = 1300;
	double l,w;
	// l = 1310.72 * pow(2,-level);
	switch(level)
	{
	case 18:
		l = 0.005; break;
	case 17:
		l = 0.01; break;
	case 16:
		l = 0.02; break;
	case 15:
		l = 0.04; break;
	case 14:
		l = 0.08; break;
	case 13:
		l = 0.16; break;
	case 12:
		l = 0.32; break;
	case 11:
		l = 0.64; break;
	}
	w = l/1.46;
	PlotShortestPath(ret, total_path, p, l, cols/l, pow(0.04,0.7) / pow(l,0.7), color);
}*/
