#include "ywmap.h"
#include <queue>
double YWMap::nodeDist(point p1, point p2)
{
#define sqr(a) ((a)*(a))
	double dist =  sqrt(sqr(p1.get<0>() - p2.get<0>()) + sqr( (p1.get<1>() - p2.get<1>()) * cos(p1.get<0>())));
	return 111.1111111 * dist;
#undef sqr
}

void YWMap::addEdge(unsigned indexfrom, unsigned indexto, double speed, double slowspeed, bool oneway, unsigned wayid)
{
	//assert(slowspeed || true); //slow speed is not used now.
	node_struct from = nodevec[indexfrom], to = nodevec[indexto];
	double dist = nodeDist(from.p, to.p);
	double time = dist / speed;
	double slowtime = dist / slowspeed;
	//printf("speed = %f, time = %f\n", speed, time);
	E.push_back(edge(indexfrom,indexto,dist,speed,time,slowtime,wayid));
	G[indexfrom].push_back(E.size() - 1);
	if(!oneway)
	{
		E.push_back(edge(indexto, indexfrom, dist,speed,time, slowtime,wayid));
		G[indexto].push_back(E.size() - 1);
	}
}

//typedef std::pair<double, unsigned> OpenSetStruct;

std::vector<unsigned> YWMap::reconstruct_path(unsigned current)
{
	//printf("hit\n");
	std::vector<unsigned> total_path;
	total_path.push_back(current);
	//printf("%u",nodevec[current].id);
	while(Came_From.find(current) != Came_From.end())
	{
		current = Came_From[current];
		total_path.push_back(current);
		//printf("->%u",nodevec[current].id);
	}
	//printf("\n");
	return total_path;
}

std::vector<unsigned> YWMap::SPFA(unsigned startid, unsigned goalid, clock_t &Time)
{
	unsigned s = nodemap[startid], t = nodemap[goalid];

	static std::queue<unsigned> Q;
	static double d[maxn];
	static int v[maxn];

	while(Q.size())Q.pop();
	Q.push(s);
	for(int i=0;i<maxn;i++) d[i] = 1e30;
	memset(v,0,sizeof(v));
	d[s] = 0;
	Came_From.clear();
	Time = clock();
	while(Q.size())
	{
		unsigned x = Q.front();Q.pop();v[x]=false;
		for(int i = 0; i < G[x].size(); i++)
		{
			edge&e = E[G[x][i]];
			if(d[e.to]>d[e.from]+e.dist)
			{
				d[e.to] = d[e.from] + e.dist;
				Came_From[e.to] = e.from;
				if(!v[e.to])
				{
					v[e.to]=true;
					Q.push(e.to);
				}
			}
		}
	}
	if(d[t]>1e20)
	{
		printf("[SPFA]No way found!\n");
		Time = clock();
		while((float)Time / CLOCKS_PER_SEC < 20) Time *= 10;
		return reconstruct_path(t);
	}
	printf("[SPFA]The dist is %fkm\n", d[t]);
	//assert(d[t]<1e29);
	Time = clock() - Time;
	printf("[SPFA]%fms used\n", (float)Time/CLOCKS_PER_SEC * 1000);
	return reconstruct_path(t);
}

std::vector<unsigned> YWMap::SPFATime(unsigned startid, unsigned goalid, clock_t &Time, std::set<unsigned> slowset = std::set<unsigned>())
{
	unsigned s = nodemap[startid], t = nodemap[goalid];

	static std::queue<unsigned> Q;
	static double d[maxn];
	static int v[maxn];

	while(Q.size())Q.pop();
	Q.push(s);
	for(int i=0;i<maxn;i++) d[i] = 1e30;
	memset(v,0,sizeof(v));
	d[s] = 0;
	Came_From.clear();
	Time = clock();
	while(Q.size())
	{
		unsigned x = Q.front();Q.pop();v[x]=false;
		for(int i = 0; i < G[x].size(); i++)
		{
			edge&e = E[G[x][i]];
			double time = (slowset.find(e.wayid)==slowset.end())?e.time:e.slowtime;
			time = time * 1.5;
			if(d[e.to]>d[e.from]+time)
			{
				d[e.to] = d[e.from] + time;
				Came_From[e.to] = e.from;
				if(!v[e.to])
				{
					v[e.to]=true;
					Q.push(e.to);
				}
			}
		}
	}
	if(d[t]>1e20)
	{
		printf("[SPFA]No way found!\n");
		Time = clock();
		while((float)Time / CLOCKS_PER_SEC < 20) Time *= 10;
		return reconstruct_path(t);
	}
	printf("[SPFA]It takes %f minutes\n", d[t] * 60);
	//assert(d[t]<1e29);
	Time = clock() - Time;
	printf("[SPFA]%fms used\n", (float)Time/CLOCKS_PER_SEC * 1000);
	return reconstruct_path(t);
}


std::vector<unsigned> YWMap::AStarDist(unsigned startid, unsigned goalid, clock_t &Time) // index, index
{
	/*clock_t Time = clock();
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
			printf("[AStar]The dist is %fkm\n", g_score[t]);
			Time = clock() - Time;
			printf("[AStar]%fms used\n", (float)Time/CLOCKS_PER_SEC * 1000);
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
	printf("[AStar] No way found!\n");*/
	unsigned s = nodemap[startid], t = nodemap[goalid];
	//static std::set<unsigned> ClosedSet;
	static bool ClosedSet[maxn];
	static std::priority_queue<std::pair<double, unsigned>, std::vector<std::pair<double,unsigned>>, std::greater<std::pair<double,unsigned>> > OpenSet;
	//static std::set<unsigned> inOpenSet;
	static bool inOpenSet[maxn];
	//static std::map<unsigned,double> g_score;
	static double g_score[maxn];
	//static std::map<unsigned,double> f_score;
	static double f_score[maxn];
	//ClosedSet.clear();
	memset(ClosedSet, 0, sizeof(ClosedSet));
	//inOpenSet.clear();
	memset(inOpenSet, 0,sizeof(inOpenSet));
	//inOpenSet.insert(s);
	inOpenSet[s] = true;
	while(OpenSet.size())OpenSet.pop();
	OpenSet.push(std::make_pair(nodeDist(nodevec[s].p,nodevec[t].p),s));
	Came_From.clear();
	//g_score.clear();
	for(int i=0;i<maxn;i++)g_score[i] = 1e30;
	g_score[s]=0;
	//f_score.clear();
	for(int i=0;i<maxn;i++)f_score[i] = 1e30;
	f_score[s]=nodeDist(nodevec[s].p, nodevec[t].p);
	Time = clock();
	while(OpenSet.size())
	{
		//auto it = OpenSet.begin();
		//unsigned current = *it;
		unsigned current = OpenSet.top().second; OpenSet.pop();
		//if(inOpenSet.find(current) == inOpenSet.end()) continue;
		if(!inOpenSet[current]) continue;
		//inOpenSet.erase(current);
		inOpenSet[current] = true;
		if(current == t)
		{
			printf("[AStar]The dist is %fkm\n", g_score[t]);
			Time = clock() - Time;
			printf("[AStar]%fms used\n", (float)Time/CLOCKS_PER_SEC * 1000);
			return reconstruct_path(t);
		}

		if(ClosedSet[current]) continue;
		//ClosedSet.insert(current);
		ClosedSet[current] = true;
		//assert(nodevec[current].isway);
		for(unsigned index:G[current])
		{
			edge& e = E[index];
			//if (ClosedSet.find(e.to)!=ClosedSet.end()) continue;
			if(ClosedSet[e.to]) continue;
			//double time = (slowset.find(e.wayid) == slowset.end())? e.time : e.slowtime;
			double tentative_g_score = g_score[current] + e.dist;
			//if(inOpenSet.find(e.to) != inOpenSet.end() && tentative_g_score >= g_score[e.to]) continue;
			if(inOpenSet[e.to] && tentative_g_score >= g_score[e.to]) continue;
			Came_From[e.to] = current;
			g_score[e.to] = tentative_g_score;
			f_score[e.to] = g_score[e.to] + nodeDist(nodevec[e.to].p, nodevec[t].p);
			//inOpenSet.insert(e.to);
			inOpenSet[e.to] = true;
			OpenSet.push(std::make_pair(f_score[e.to], e.to));
		}
	}
	printf("[AStar] No way found!\n");
	Time = clock() - Time;
	return std::vector<unsigned>();
}


std::vector<unsigned> YWMap::AStarTime(unsigned startid, unsigned goalid, clock_t &Time, std::set<unsigned> slowset = std::set<unsigned>()) // index, index
{
	unsigned s = nodemap[startid], t = nodemap[goalid];
	//static std::set<unsigned> ClosedSet;
	static bool ClosedSet[maxn];
	static std::priority_queue<std::pair<double, unsigned>, std::vector<std::pair<double,unsigned>>, std::greater<std::pair<double,unsigned>> > OpenSet;
	//static std::set<unsigned> inOpenSet;
	static bool inOpenSet[maxn];
	//static std::map<unsigned,double> g_score;
	static double g_score[maxn];
	//static std::map<unsigned,double> f_score;
	static double f_score[maxn];
	//ClosedSet.clear();
	memset(ClosedSet, 0, sizeof(ClosedSet));
	//inOpenSet.clear();
	memset(inOpenSet, 0,sizeof(inOpenSet));
	//inOpenSet.insert(s);
	inOpenSet[s] = true;
	while(OpenSet.size())OpenSet.pop();
	OpenSet.push(std::make_pair(nodeDist(nodevec[s].p,nodevec[t].p),s));
	Came_From.clear();
	//g_score.clear();
	for(int i=0;i<maxn;i++)g_score[i] = 1e30;
	g_score[s]=0;
	//f_score.clear();
	for(int i=0;i<maxn;i++)f_score[i] = 1e30;
	f_score[s]=nodeDist(nodevec[s].p, nodevec[t].p);
	Time = clock();
	while(OpenSet.size())
	{
		//auto it = OpenSet.begin();
		//unsigned current = *it;
		unsigned current = OpenSet.top().second; OpenSet.pop();
		//if(inOpenSet.find(current) == inOpenSet.end()) continue;
		if(!inOpenSet[current]) continue;
		//inOpenSet.erase(current);
		inOpenSet[current] = true;
		if(current == t)
		{
			printf("[AStar]It takes %f minutes\n", g_score[t] * 60);
			Time = clock() - Time;
			printf("[AStar]%fms used\n", (float)Time/CLOCKS_PER_SEC * 1000);
			return reconstruct_path(t);
		}

		if(ClosedSet[current]) continue;
		//ClosedSet.insert(current);
		ClosedSet[current] = true;
		//assert(nodevec[current].isway);
		for(unsigned index:G[current])
		{
			edge& e = E[index];
			//if (ClosedSet.find(e.to)!=ClosedSet.end()) continue;
			if(ClosedSet[e.to]) continue;
			double time = (slowset.find(e.wayid) == slowset.end())? e.time : e.slowtime;
			time = time * 1.5;
			double tentative_g_score = g_score[current] + time;
			//if(inOpenSet.find(e.to) != inOpenSet.end() && tentative_g_score >= g_score[e.to]) continue;
			if(inOpenSet[e.to] && tentative_g_score >= g_score[e.to]) continue;
			Came_From[e.to] = current;
			g_score[e.to] = tentative_g_score;
			f_score[e.to] = g_score[e.to] + nodeDist(nodevec[e.to].p, nodevec[t].p) / 80 * 1.5 ;
			//inOpenSet.insert(e.to);
			inOpenSet[e.to] = true;
			OpenSet.push(std::make_pair(f_score[e.to], e.to));
		}
	}
	printf("[AStar] No way found!\n");
	Time = clock() - Time;
	return std::vector<unsigned>();
}

std::vector<unsigned> YWMap::dijkstraDist(unsigned startid, unsigned goalid, clock_t &Time)
{
	unsigned s = nodemap[startid], t = nodemap[goalid];
	static std::priority_queue<std::pair<double,unsigned>,std::vector<std::pair<double,unsigned> >, std::greater<std::pair<double,unsigned> > >Q;
	static double d[maxn];
	static bool done[maxn];

	while(Q.size())Q.pop();
	for(int i=0;i<maxn;i++) d[i] = 1e30;
	memset(done,0,sizeof(done));

	Q.push(std::make_pair(0,s));
	d[s] = 0;
	Time = clock();
	while(Q.size())
	{
		unsigned x = Q.top().second;
		Q.pop();
		if(done[x])continue;
		done[x] = true;
		for(unsigned index:G[x])
		{
			edge&e=E[index];
			if(d[e.to] > d[x] + e.dist)
			{
				d[e.to] = d[x] + e.dist;
				Came_From[e.to] = x;
				Q.push(std::make_pair(d[e.to],e.to));
				if(e.to == t)
				{
					printf("[Dijkstra]The dist is %fkm\n", d[t]);
					Time = clock() - Time;
					printf("[Dijkstra]%fms used\n", (float)Time/CLOCKS_PER_SEC * 1000);
					return reconstruct_path(t);
				}
			}
		}
	}
	printf("[Dijkstra] No way found!\n");
	Time = clock() - Time;
	return std::vector<unsigned>();
}

std::vector<unsigned> YWMap::dijkstraTime(unsigned startid, unsigned goalid,  clock_t &Time, std::set<unsigned> slowset = std::set<unsigned>())
{
	unsigned s = nodemap[startid], t = nodemap[goalid];
	static std::priority_queue<std::pair<double,unsigned>,std::vector<std::pair<double,unsigned> >, std::greater<std::pair<double,unsigned> > >Q;
	static double d[maxn];
	static bool done[maxn];

	while(Q.size())Q.pop();
	for(int i=0;i<maxn;i++) d[i] = 1e30;
	memset(done,0,sizeof(done));
	Came_From.clear();

	Q.push(std::make_pair(0,s));
	d[s] = 0;
	Time = clock();
	while(Q.size())
	{
		unsigned x = Q.top().second;
		Q.pop();
		if(done[x])continue;
		done[x] = true;
		for(unsigned index:G[x])
		{
			edge&e=E[index];
			double time = (slowset.find(e.wayid) == slowset.end())? e.time : e.slowtime;
			time = time * 1.5;
			if(d[e.to] > d[e.from] + time)
			{
				d[e.to] = d[e.from] + time;
				Came_From[e.to] = e.from;
				Q.push(std::make_pair(d[e.to],e.to));
				if(e.to == t)
				{
					printf("[Dijkstra]It takes %f minutes\n", d[t] * 60);
					Time = clock() - Time;
					printf("[Dijkstra]%fms used\n", (float)Time/CLOCKS_PER_SEC * 1000);
					return reconstruct_path(t);
				}
			}
		}
	}
	printf("[Dijkstra] No way found!\n");
	Time = clock() - Time;
	return std::vector<unsigned>();
}
