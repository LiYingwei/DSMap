#include "ywmap.h"
using namespace cv;
YWMap::YWMap()
{

}

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
			for(pugi::xml_node nd = obj.first_child(); strcmp(nd.name(),"nd") == 0; nd = nd.next_sibling())
			{
				unsigned id, index;
				sscanf(nd.attribute("ref").value(), "%ud", &id);
				index = nodemap[id];
				nodevec[index].way.push_back(nd);
			}
			unsigned id;
			sscanf(obj.attribute("id").value(), "%ud", &id);
			wayvec.push_back(way_struct(obj));
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
					//assert(0);
				}
			}
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

	for(auto node : nodeset)
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
	}

	for(auto node : nodeset)
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
	}
	imshow("map", ret);
	waitKey(0);
	return ret;
}

cv::Point2d YWMap::p2P(point v, point p, double scale)
{
	cv::Point2d ret((v.get<1>() - p.get<1>()) * scale, (p.get<0>() - v.get<0>()) * scale);
	return ret;
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
