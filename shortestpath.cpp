#include "ywmap.h"
#include <queue>
double YWMap::nodeDist(point p1, point p2)
{
#define sqr(a) ((a)*(a))
	return sqrt(sqr(p1.get<0>() - p2.get<0>()) + sqr( (p1.get<1>() - p2.get<1>()) * cos(p1.get<0>())));
#undef sqr
}

void YWMap::addEdge(unsigned indexfrom, unsigned indexto, double speed, double slowspeed, bool oneway)
{
	assert(slowspeed || true); //slow speed is not used now.
	node_struct from = nodevec[indexfrom], to = nodevec[indexto];
	double dist = nodeDist(from.p, to.p);
	double time = dist * speed;
	E.push_back(edge(indexfrom,indexto,dist,time));
	G[indexfrom].push_back(E.size() - 1);
	if(!oneway)
	{
		E.push_back(edge(indexto, indexfrom, dist, time));
		G[indexto].push_back(E.size() - 1);
	}
}

//typedef std::pair<double, unsigned> OpenSetStruct;

std::vector<unsigned> YWMap::reconstruct_path(unsigned current)
{
	printf("hit\n");
	std::vector<unsigned> total_path;
	total_path.push_back(current);
	printf("%u",nodevec[current].id);
	while(Came_From.find(current) != Came_From.end())
	{
		current = Came_From[current];
		total_path.push_back(current);
		printf("->%u",nodevec[current].id);
	}
	printf("\n");
	return total_path;
}

std::vector<unsigned> YWMap::SPFA(unsigned startid, unsigned goalid)
{
	unsigned s = nodemap[startid], t = nodemap[goalid];

	static std::queue<unsigned> Q;
	static std::map<unsigned,double> d;
	static std::set<unsigned> v;

	while(Q.size())Q.pop();
	Q.push(s);
	d.clear();
	d[s] = 0;
	v.insert(s);
	Came_From.clear();
	while(Q.size())
	{
		unsigned x = Q.front();Q.pop();
		v.erase(x);
		for(int i = 0; i < G[x].size(); i++)
		{
			edge&e = E[G[x][i]];
			if(d.find(e.to) == d.end() || d[e.to] > d[e.from] + e.dist)
			{
				d[e.to] = d[e.from] + e.dist;
				Came_From[e.to] = e.from;
				if(v.find(e.to) == v.end())
				{
					v.insert(e.to);
					Q.push(e.to);
				}
			}
		}
	}
	printf("SPFAdist = %f\n", d[t]);
	assert(d[t]<1e29);
	return reconstruct_path(t);
}

/*std::vector<unsigned> YWMap::Dijkstra(unsigned startid, unsigned goalid)
{
	unsigned s = nodemap[startid], t = nodemap[goalid];

	static std::priority_queue<std::pair<double,unsigned>, std::vector<std::pair<double,unsigned>>, std::greater<std::pair<double,unsigned>> >;
	static std::map<double, unsigned> d;

	while(Q.size())Q.pop();
	d.clear();

	d[s] = 0;
	Q.push(std::make_pair(d[s], s));

	while(Q.size())
	{
		unsigned
	}

}*/

std::vector<unsigned> YWMap::AStarDist(unsigned startid, unsigned goalid) // index, index
{
	unsigned s = nodemap[startid], t = nodemap[goalid];
	static std::set<unsigned> ClosedSet;
	static std::priority_queue<std::pair<double, unsigned>, std::vector<std::pair<double,unsigned>>, std::greater<std::pair<double,unsigned>> > OpenSet;
	static std::set<unsigned> inOpenSet;
	static std::map<unsigned,double> g_score;
	static std::map<unsigned,double> f_score;
	ClosedSet.clear();
	inOpenSet.clear();
	inOpenSet.insert(s);
	while(OpenSet.size())OpenSet.pop();
	OpenSet.push(std::make_pair(nodeDist(nodevec[s].p,nodevec[t].p),s));
	Came_From.clear();
	g_score.clear();g_score[s]=0;
	f_score.clear();f_score[s]=nodeDist(nodevec[s].p, nodevec[t].p);
	while(inOpenSet.size())
	{
		//auto it = OpenSet.begin();
		//unsigned current = *it;
		unsigned current = OpenSet.top().second; OpenSet.pop();
		if(inOpenSet.find(current) == inOpenSet.end()) continue;
		inOpenSet.erase(current);

		if(current == t)
		{
			printf("AStarDist = %f\n", g_score[t]);
			return reconstruct_path(t);
		}

		ClosedSet.insert(current);
		assert(nodevec[current].isway);
		for(unsigned index:G[current])
		{
			edge& e = E[index];
			if (ClosedSet.find(e.to)!=ClosedSet.end()) continue;
			double tentative_g_score = g_score[current] + e.dist;
			if(inOpenSet.find(e.to) != inOpenSet.end() && tentative_g_score >= g_score[e.to]) continue;
			Came_From[e.to] = current;
			g_score[e.to] = tentative_g_score;
			f_score[e.to] = g_score[e.to] + nodeDist(nodevec[e.to].p, nodevec[t].p);
			inOpenSet.insert(e.to);
			OpenSet.push(std::make_pair(f_score[e.to], e.to));
		}
	}
	assert(0);
}

void YWMap::PlotShortestPath(cv::Mat &ret, std::vector<unsigned> total_path,cv::Scalar color)
{
	for(int i = 0; i < total_path.size()-1; i++)
	{
		point s = nodevec[total_path[i]].p;
		point t = nodevec[total_path[i+1]].p;
		plotline(ret,s,t,p,l,color,5,CV_AA);
	}
}
