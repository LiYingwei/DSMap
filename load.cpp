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
	pugi::xml_node obj = osm.first_child(); // the first child is bounds, ignore it temporarily
	//while(strcmp(obj.value(),"node") != 0) {obj=obj.next_sibling();printf("%s\n",obj.value());}
/////////////////////把node的信息存入nodevec，nodemap是从id到下标的映射///////////////////////
#ifdef INFO
	int cnt=0;
#endif
	for(obj = osm.child("node") ; strcmp(obj.name(),"node") == 0; obj = obj.next_sibling())
	{
#ifdef INFO
		cnt++;
		if(cnt % 100000==0) printf("Adding node No.%d\n",cnt);
#endif
		unsigned id = obj.attribute("id").as_uint(-1);
		point p(obj.attribute("lat").as_double(-1),obj.attribute("lon").as_double(-1));
#ifdef DEBUG
		assert(id != -1 && p.get<0>() != -1 && p.get<1>() != -1);
#endif
		nodevec.push_back(node_struct(id,p,obj, false, false));
		nodemap[id] = nodevec.size() - 1;
	}
#ifdef INFO
	printf("Node added!\n");
#endif
/////////////////把way的信息存入wayvec，并且更新nodevec的isway和nd_in_way信息////////////////////
	for( ;strcmp(obj.name(),"way") == 0; obj = obj.next_sibling())
	{
		unsigned id = obj.attribute("id").as_uint(-1);
		pugi::xml_node tag;
		std::string type="default", subtype="default";
		int layer=0;
		bool bridge=false, oneway = false;
		for(tag = obj.last_child(); strcmp(tag.name(),"tag") == 0; tag = tag.previous_sibling())
		{
			//if(tag.attribute("k").as_string() == std::string("highway") ) waytype = tag.attribute("v").as_string("default");
			auto it = elementmap.find(make_pair(tag.attribute("k").as_string(),tag.attribute("v").as_string()));
			if(it != elementmap.end()) type = it->first.first, subtype = it->first.second;

			if(tag.attribute("k").as_string() == std::string("layer") ) layer = tag.attribute("v").as_int();
			if(tag.attribute("k").as_string() == std::string("bridge") ) bridge = true;
			if(tag.attribute("k").as_string() == std::string("oneway") && tag.attribute("v").as_string() == std::string("yes"))
				oneway = true;
		}

		auto it = elementmap.find(make_pair(type,subtype));
		if(it == elementmap.end())
		{
#ifdef DEBUG
			if(type!="default")printf("type,subtype %s,%s not found\n",type.c_str(), subtype.c_str());
#endif
			continue;
		}
		if(elementvec[it->second]["method"] == std::string("fill_poly"))
		{
			auto &elem = elementvec[it->second];
			cv::Scalar color = hex2BGR(elem["color"]), ccolor = color;
			auto itl = layermap.find(make_pair(type,subtype));
			if(itl!=layermap.end())
			{
				auto& lay = layervec[itl->second];
				if(lay.find("color")!=lay.end())ccolor = hex2BGR(lay["color"]);
				if(layer == 0) sscanf(lay["layer"].c_str(),"%d",&layer);
			}
			double x1=361,x2=-361,y1=361,y2=-361;
			for(pugi::xml_node nd = tag; nd; nd = nd.previous_sibling())
			{
				node_struct &node = nodevec[nodemap[nd.attribute("ref").as_uint()]];
				x1 = min(x1,node.p.get<0>());
				x2 = max(x2,node.p.get<1>());
				y1 = min(y1,node.p.get<0>());
				y2 = max(y2,node.p.get<1>());
				node.isbuilding = true;
			}
			building_struct build(id,layer,type,subtype,color,ccolor,1,obj,box(point(x1,y1),point(x2,y2)));
			buildvec.push_back(build);
			buildmap[id]=buildvec.size()-1;
		}
		else if(type == "highway")
		{
/*			if(it == elementmap.end())
			{
#ifdef DEBUG1
				printf("highway type not found: %s", waytype.c_str());
#endif
				continue; // this way is ignored
			}*/
			auto &elem = elementvec[elementmap[make_pair("highway",subtype)]];
			cv::Scalar color = hex2BGR(elem["color"]), ccolor(0xD0,0xCA,0xC1);
			int thickness,boundthick=1;
			if(bridge)
			{
				boundthick = 0;
				ccolor = color;
			}
			sscanf(elem["thickness"].c_str(),"%d", &thickness);
			double speed = normalSpeed[std::make_pair(type, subtype)];
			double slowspeed = slowSpeed[std::make_pair(type, subtype)];
			wayvec.push_back(way_struct(id, layer, subtype,color,ccolor,thickness,boundthick,speed,slowspeed,oneway));
			waymap[id] = wayvec.size() - 1;
			pugi::xml_node nd;
			for(nd = tag; nd; nd = nd.previous_sibling())
			{
				unsigned ref = nd.attribute("ref").as_uint(-1);
#ifdef DEBUG
				assert(ref != -1);
#endif
				unsigned index = nodemap[ref];
				nodevec[index].isway = true;
				nodevec[index].nd_in_way.push_back(nd);
				if(nd == tag || nd == nd.parent().first_child()) nodevec[index].fanout++;
				else nodevec[index].fanout += 2;

				if(nd != tag)addEdge(nodemap[nd.attribute("ref").as_uint()], nodemap[nd.next_sibling().attribute("ref").as_uint()],speed, slowspeed, oneway);
			}
		}
	}
#ifdef INFO
	printf("Way added!\n");
#endif
////////////////把节点信息加入R树以及取出重要点///////////////////
#ifdef DEBUG
	int nodenum = 0, importantnodenum = 0;
#endif
	for(int i = 0; i < nodevec.size(); i++) if(nodevec[i].isway)
	{
		node_struct & node = nodevec[i];
#ifdef DEBUG
		nodenum ++;
		if(node.fanout != 2) importantnodenum ++;
#endif
#ifdef INFO
		//printf("node No.%u in a way\n",node.id);
#endif
		way_node_tree.insert(std::make_pair(node.p,i));
	};
/////////////////////把建筑信息加入R树////////////////////////
	for(int i=0;i<buildvec.size(); i++)
	{
		building_struct& b = buildvec[i];
		build_tree.insert(std::make_pair(b.b,i));
	}
#ifdef DEBUG
	printf("Edge num : %d\nNode num : %d\nNode(fanout = 2) num : %d\n", E.size(), nodenum, importantnodenum);
#endif
}

void YWMap::loadPlotConf()
{
	assert(doc_plot_conf.load_file("plot_config.xml"));
	pugi::xml_node plot_config = doc_plot_conf.child("YuxinMap");
	pugi::xml_node layers = plot_config.child("layers");
	pugi::xml_node elements = plot_config.child("elements");
	layervec.clear();
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
	/*for(auto it = layermap.begin(); it != layermap.end(); it++)
		printf("%s, %s\n",it->first.first.c_str(), it->first.second.c_str());*/
}

void YWMap::loadSpeedConf()
{
	doc_speed_conf.load_file("ShortestPath.conf");
	pugi::xml_node YuxinMap = doc_speed_conf.child("YuxinMap");
	pugi::xml_node normal = YuxinMap.first_child();
	pugi::xml_node slow = normal.next_sibling();
	for(pugi::xml_node way = normal.first_child(); way; way = way.next_sibling())
	{
		double speed = way.attribute("speed").as_double();
		pugi::xml_node tag = way.first_child();
		std::string type = tag.attribute("k").as_string();
		std::string subtype = tag.attribute("v").as_string();
		normalSpeed[std::make_pair(type,subtype)] = speed;
	}
	for(pugi::xml_node way = slow.first_child(); way; way = way.next_sibling())
	{
		double speed = way.attribute("speed").as_double();
		pugi::xml_node tag = way.first_child();
		std::string type = tag.attribute("k").as_string();
		std::string subtype = tag.attribute("v").as_string();
		slowSpeed[std::make_pair(type,subtype)] = speed;
	}
}
