#include "ywmap.h"
#include <cstdio>
using namespace cv;
using namespace std;
using namespace pugi;

void YWMap::setDocosm(char* path)
{
	assert(doc_osm.load_file(path));
}

void YWMap::loadMap()
{
	pugi::xml_node osm = doc_osm.child("osm");
	for(pugi::xml_node obj = osm.first_child(); obj; obj = obj.next_sibling())
	{
		if(obj.name()[0] == 'n')
		{
			double lat,lon;
			unsigned id;
			sscanf( obj.attribute("lat").value(), "%lf", &lat);
			sscanf( obj.attribute("lon").value(), "%lf", &lon);
			sscanf( obj.attribute("id").value(), "%ud", &id);
			point node(lat, lon);
			nodertree.insert(std::make_pair(node,id));
			nodevec.push_back(node_struct(node, obj));
			nodemap[id] = nodevec.size() - 1;
		}
		if(obj.name()[0] == 'w')
		{
			int layerid;
			string type,subtype;
			bool iscontour;
			unsigned firstid=-1, id;
			pugi::xml_node nd;
			for(nd = obj.first_child(); strcmp(nd.name(),"nd") == 0; nd = nd.next_sibling())
			{
				unsigned index;
				sscanf(nd.attribute("ref").value(), "%ud", &id);
				if(firstid==-1)firstid = id;
				index = nodemap[id];
				nodevec[index].way.push_back(nd);
			}
			iscontour = (firstid == id);
			for(;nd; nd = nd.next_sibling())
			{
				pugi::xml_attribute key = nd.first_attribute();
				pugi::xml_attribute value = key.next_attribute();
				std::map<pair<std::string,std::string>, unsigned>::iterator it;
				it = layermap.find(make_pair(key.value(),value.value()));
				if(it != layermap.end())
				{
					layerid = it->second;
					type = key.value();
					subtype = value.value();
					break;
				}
				else
				{
					it = layermap.find(make_pair(key.value(),"default"));
					layerid = it->second;
					type = key.value();
					subtype = value.value();
					break;
				}
			}

			sscanf(obj.attribute("id").value(), "%ud", &id);
			wayvec.push_back(way_struct(obj,iscontour,layerid,type,subtype));
			waymap[id] = wayvec.size() - 1;
		}
		if(obj.name()[0] == 'r')
		{
			for(pugi::xml_node member = obj.first_child(); strcmp(member.name(),"member") == 0; member = member.next_sibling())
			{
				unsigned id, index;
				char type = member.attribute("type").value()[0];
				sscanf(member.attribute("ref").value(), "%ud", &id);
				if(type == 'n')
				{
					index = nodemap[id];
					nodevec[index].relation.push_back(member);
				}
				else if(type == 'w')
				{
					index = waymap[id];
					wayvec[index].relation.push_back(member);
				}
				else
				{
					std::cout << member.attribute("type").value() << std::endl;
					std::cout << member.parent().attribute("id").value() << std::endl;
					assert(0);
				}
			}
		}
	}
}

void YWMap::loadPlotConf()
{
	doc_plot_conf.load("plot_config.xml");
	pugi::xml_node plot_config = doc_plot_conf.child("YuxinMap");
	pugi::xml_node layers = plot_config.child("layers");
	pugi::xml_node elements = plot_config.child("elements");
	for(pugi::xml_node layer = layers.first_child(); layer; layer = layer.next_sibling())
	{
		std::map<std::string,std::string> layerattr; layerattr.clear();
		for(pugi::xml_attribute attr = layer.first_attribute(); attr; attr = attr.next_attribute())
			layerattr[attr.name()] = attr.value();
		layervec.push_back(layerattr);
		for(pugi::xml_node way = layer.first_child(); way; way = way.next_sibling())
		{
			pugi::xml_attribute key = way.first_child().first_attribute();
			string value = key.next_attribute().as_string("default");
			layermap[make_pair(key.value(),value)] = layervec.size() - 1;
		}
	}
	for(pugi::xml_node element = elements.first_child(); element; element = element.next_sibling())
	{
		std::map<std::string,std::string> elementattr; elementattr.clear();
		for(pugi::xml_attribute attr = element.first_attribute(); attr; attr = attr.next_attribute())
			elementattr[attr.name()] = attr.value();
		elementvec.push_back(elementattr);
		for(pugi::xml_node way = element.first_child(); way; way = way.next_sibling())
		{
			pugi::xml_attribute key = way.first_child().first_attribute();
			string value = key.next_attribute().as_string("default");
			elementmap[make_pair(key.value(), value)] = elementvec.size() - 1;
		}
	}
}

cv::Mat YWMap::Plot(point p, double l, double scale)
{
	double scalex = scale, scaley = cos(p.get<0>()) * scale;
	int r = l * scalex, c = l * scaley;
	printf("%d %d\n", r, c);
	cv::Mat ret(r,c,CV_8UC3,Scalar(0xf0,0xf5,0xf7));
	box query_box(point(p.get<0>() - l, p.get<1>()), point(p.get<0>(), p.get<1>() + l));
	std::vector<nodeinfo> nodeset;
	nodertree.query(bgi::intersects(query_box), std::back_inserter(nodeset));
	/*for(auto node : nodeset)
	{
		cv::Point2d nodepoint((node.first.get<1>() - p.get<1>()) * scale, (p.get<0>() - node.first.get<0>()) * scale); //没有搞清楚坐标情况，反正现在是对的。。
		circle(ret, nodepoint, 2, Scalar(0,0,255), -1, 8);
		std::cout << nodepoint << std::endl;
		//std::cout << bg::wkt<point>(node.first) << std::endl;
	}*/

	/*for(auto node : nodeset)
	{
		unsigned id = node.second, index = nodemap[id];
		node_struct &u = nodevec[index];
		for(pugi::xml_node nd : u.way)
		{
			if(nd!=nd.parent().last_child() && nd.next_sibling().name()[0] == 'n')
			{
				unsigned idv, indexv;
				sscanf(nd.next_sibling().attribute("ref").value(), "%ud", &idv);
				indexv = nodemap[idv];
				node_struct &v = nodevec[indexv];
				plotline(ret, u.p, v.p, p, l, scale ,Scalar(198,208,214), 4, CV_AA);
			}
			if(nd!=nd.parent().first_child() && nd.previous_sibling().name()[0] == 'n')
			{
				unsigned idv, indexv;
				sscanf(nd.previous_sibling().attribute("ref").value(), "%ud", &idv);
				indexv = nodemap[idv];
				node_struct &v = nodevec[indexv];
				if(v.p.get<0>() > p.get<0>() || v.p.get<1>() < p.get<1>() ||
						v.p.get<0>() < p.get<0>() - l || v.p.get<1>() > p.get<1>() + l)
					plotline(ret, u.p, v.p, p, l, scale ,Scalar(198,208,214), 4, CV_AA);
			}
		}
	}*/

	/*for(auto node : nodeset)
	{
		unsigned id = node.second, index = nodemap[id];
		node_struct &u = nodevec[index];
		for(pugi::xml_node nd : u.way)
		{
			if(nd!=nd.parent().last_child() && nd.next_sibling().name()[0] == 'n')
			{
				unsigned idv, indexv;
				sscanf(nd.next_sibling().attribute("ref").value(), "%ud", &idv);
				indexv = nodemap[idv];
				node_struct &v = nodevec[indexv];
				plotline(ret, u.p, v.p, p, l, scale ,Scalar(255,255,255), 2, CV_AA);
			}
			if(nd!=nd.parent().first_child() && nd.previous_sibling().name()[0] == 'n')
			{
				unsigned idv, indexv;
				sscanf(nd.previous_sibling().attribute("ref").value(), "%ud", &idv);
				indexv = nodemap[idv];
				node_struct &v = nodevec[indexv];
				if(v.p.get<0>() > p.get<0>() || v.p.get<1>() < p.get<1>() ||
						v.p.get<0>() < p.get<0>() - l || v.p.get<1>() > p.get<1>() + l)
					plotline(ret, u.p, v.p, p, l, scale ,Scalar(255,255,255), 2, CV_AA);
			}
		}
	}*/

	std::vector<layer_point_nd> predraw;
	for(nodeinfo node: nodeset)
	{
		unsigned id = node.second, index = nodemap[id];
		node_struct &u = nodevec[index];
		std::set<std::string> contour;
		for(pugi::xml_node nd : u.way) //layer, point, nd
		{
			way_struct way = wayvec[nd.parent().attribute("id").as_uint()];
			int layer;
			sscanf(layervec[way.layerid]["id"].c_str(),"%d",layer);
			bool iscontour = way.iscontour;
			if(!iscontour)predraw.push_back(layer_point_nd(layer, node.first, nd));
			else if(contour.find(nd.parent().attribute("id").value()) == contour.end())
			{
				contour.insert(nd.parent().attribute("id").value());
				predraw.push_back(layer_point_nd(layer, node.first, nd.parent()));
			}
		}
	}
	std::sort(predraw.begin(),predraw.end());
	std::vector<int> layersize;
	layersize.clear();
	for(int i=0;i<predraw.size();i++) if(i==0 || predraw[i].layer!=predraw[i-1].layer)
		layersize.push_back(1);
	else layersize[layersize.size()-1]++;
	int base=0;
	for(int i=0;i<layersize.size();i++)
	{
		bool contour = (predraw[base].nd.name()[0] == 'w');
		for(int j=0;j<layersize[i];j++)
		{
			if(contour)plotPolyLayer(ret, predraw[base + j].nd, p, scale);
			else plotLineLayer(predraw[base + j].nd); //1 means outline
			/*{
				pugi::xml_node &nd = predraw[base + j].nd;
				unsigned idu,indexu;
				sscanf(nd.attribute("ref").value(), "ud", &idu);
				indexu = nodemap[idu];
				node_struct &u = nodevec[indexu];
				if(nd!=nd.parent().last_child() && nd.next_sibling().name()[0] == 'n')
				{
					unsigned idv, indexv;
					sscanf(nd.next_sibling().attribute("ref").value(), "%ud", &idv);
					indexv = nodemap[idv];
					node_struct &v = nodevec[indexv];
					plotline(ret, u.p, v.p, p, l, scale ,Scalar(255,255,255), 2, CV_AA);
				}
				if(nd!=nd.parent().first_child() && nd.previous_sibling().name()[0] == 'n')
				{
					unsigned idv, indexv;
					sscanf(nd.previous_sibling().attribute("ref").value(), "%ud", &idv);
					indexv = nodemap[idv];
					node_struct &v = nodevec[indexv];
					if(v.p.get<0>() > p.get<0>() || v.p.get<1>() < p.get<1>() ||
							v.p.get<0>() < p.get<0>() - l || v.p.get<1>() > p.get<1>() + l)
						plotline(ret, u.p, v.p, p, l, scale ,Scalar(255,255,255), 2, CV_AA);
				}*/
		}
		for(int j=0;j<layersize[i];j++)
		{
			if(contour)plotPolyElement(ret, predraw[base + j].nd, p, scale);
			plotLineElement(predraw[base + j].nd);
			/*pugi::xml_node &nd = predraw[base].nd;
			if(nd!=nd.parent().first_child() && nd.previous_sibling().name()[0] == 'n')
			{
				unsigned idv, indexv;
				sscanf(nd.previous_sibling().attribute("ref").value(), "%ud", &idv);
				indexv = nodemap[idv];
				node_struct &v = nodevec[indexv];
				if(v.p.get<0>() > p.get<0>() || v.p.get<1>() < p.get<1>() ||
						v.p.get<0>() < p.get<0>() - l || v.p.get<1>() > p.get<1>() + l)
					plotline(ret, u.p, v.p, p, l, scale ,Scalar(255,255,255), 2, CV_AA);
			}*/
		}
		base += layersize[i];
	}
	imshow("map", ret);
	waitKey(0);
	return ret;
}

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
	cv::fillPoly(img, elementPoints, &numberOfPoints, 1, Scalar (0, 0, 0), 8);
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

/*void MyFilledCircle( Mat img, Point center )
{
 int thickness = -1;
 int lineType = 8;

 circle( ret,
		 center,
		 w/32.0,
		 Scalar( 0, 0, 255 ),
		 thickness,
		 lineType );
}*/
