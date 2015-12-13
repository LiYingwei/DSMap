#include "ywmap.h"

std::vector<unsigned> YWMap::nearest(point p, int k)
{
	std::vector<std::pair<point, unsigned>> result_n;
	way_node_tree.query(bgi::nearest(p, k), std::back_inserter(result_n));
	std::vector<unsigned> result;
	for(int i=0;i<result_n.size(); i++) result.push_back(result_n[i].second);
	return result;
}
