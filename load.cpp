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
/////////////////////把node的信息存入nodevec，nodemap是从id到下标的映射///////////////////////
	for(obj = obj.next_sibling(); strcmp(obj.name(),"node") == 0; obj = obj.next_sibling())
	{
		unsigned id = obj.attribute("id").as_uint(-1);
		point p(obj.attribute("lat").as_double(-1),obj.attribute("lon").as_double(-1));
#ifdef DEBUG
		assert(id != -1 && p.get<0>() != -1 && p.get<1>() != -1);
#endif
		nodevec.push_back(node_struct(id,p,obj, false, false));
		nodemap[id] = nodevec.size() - 1;
	}
/////////////////把way的信息存入wayvec，并且更新nodevec的isway和nd_in_way信息////////////////////
	for( ;strcmp(obj.name(),"way") == 0; obj = obj.next_sibling())
	{
		unsigned id = obj.attribute("id").as_uint(-1);
		pugi::xml_node tag;
		std::string waytype = "not a way";
		int layer=0;
		for(tag = obj.last_child(); strcmp(tag.name(),"tag") == 0; tag = tag.previous_sibling())
		{
			if(tag.attribute("k").as_string() == std::string("highway") ) waytype = tag.attribute("v").as_string("default");
			if(tag.attribute("k").as_string() == std::string("layer") ) layer = tag.attribute("v").as_int();
		}
		if(waytype != "not a way")
		{
			auto it = elementmap.find(make_pair("highway",waytype));
			if(it == elementmap.end())
			{
#ifdef DEBUG1
				printf("highway type not found: %s", waytype.c_str());
#endif
				continue; // this way is ignored
			}
			auto &elem = elementvec[elementmap[make_pair("highway",waytype)]];
			cv::Scalar color = hex2BGR(elem["color"]);
			int thickness;
			sscanf(elem["thickness"].c_str(),"%d", &thickness);
			wayvec.push_back(way_struct(id, layer, waytype,color,cv::Scalar(0xD0,0xCA,0xC1),thickness,1));
			waymap[id] = wayvec.size() - 1;
			pugi::xml_node nd;
			for(nd = tag; nd; nd = nd.previous_sibling())
			{
				unsigned ref = tag.attribute("ref").as_uint(-1);
#ifdef DEBUG
				assert(ref != -1);
#endif
				unsigned index = nodemap[ref];
				nodevec[index].isway = true;
				nodevec[index].nd_in_way.push_back(nd);
			}
		}
	}
/////////////////////把节点信息加入R树////////////////////////
	for(int i = 0; i < nodevec.size(); i++) if(nodevec[i].isway)
	{
		node_struct & node = nodevec[i];
#ifdef INFO
		//printf("node No.%u in a way\n",node.id);
#endif
		way_node_tree.insert(std::make_pair(node.p,i));
	} //else printf("not a way\n");
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
