#include "ywmap.h"

std::vector<std::pair<point,unsigned>> YWMap::querybox(box b)
{
	std::vector<std::pair<point,unsigned>> result;
	way_node_tree.query(bgi::intersects(b), std::back_inserter(result));
	return result;
}
