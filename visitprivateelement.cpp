#include "ywmap.h"

unsigned YWMap::getNodeIndexById(unsigned id)
{
	auto it = nodemap.find(id);
	if(it == nodemap.end()) return -1;
	return it->second;
}

unsigned YWMap::getNodeIdByIndex(unsigned index)
{
	return nodevec[index].id;
}
