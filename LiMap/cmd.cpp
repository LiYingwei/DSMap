#include "ywmap.h"
extern YWMap map;
void YWMap::cmd_showmap()
{
	double lat,lon;
	int level;
	scanf("%lf%lf%d", &lat, &lon, &level);
	cv::Mat show = map.Plot(point(lat,lon), level);
	cv::imwrite("showmap.png", show);
}

void YWMap::cmd_shortestpath()
{
	char s1[100],s2[100];
	scanf("%s%s",s1,s2);
	unsigned id1 = strtoul(s1,NULL,0),id2 = strtoul(s2,NULL,0);
	if(map.getNodeIndexById(id1) == -1)
	{
		auto name_point =  map.queryName(s1);
		if(name_point.size() == 0)
		{
			printf("%s NOT FOUND\n", s1);
			return;
		}
		id1 = map.getNodeIdByIndex(map.nearest(name_point[0].second)[0]);
		printf("id1 = %u(%s)\n", id1, name_point[0].first.c_str());
	}
	if(map.getNodeIndexById(id2) == -1)
	{
		auto name_point = map.queryName(s2);
		if(name_point.size() == 0)
		{
			printf("%s NOT FOUND\n", s2);
			return;
		}
		id2 = map.getNodeIdByIndex(map.nearest(name_point[0].second)[0]);
		printf("id2 = %u(%s)\n", id2, name_point[0].first.c_str());
	}
	int slownum;
	scanf("%d",&slownum);
	std::set<unsigned> slowset;
	for(int i=0;i<slownum;i++)
	{
		unsigned id;
		scanf("%u", &id);
		slowset.insert(id);
	}
	//if(map.nodemap.find(id1) == map.nodemap.end()) id1 = map.queryName(s1)
	clock_t Time;
	cv::Mat pathspfa = map.PlotShortestPath(map.SPFATime(id1,id2,Time,slowset));
	cv::Mat pathdijk = map.PlotShortestPath(map.dijkstraTime(id1,id2,Time,slowset));
	cv::Mat path = map.PlotShortestPath(map.AStarTime(id1,id2,Time,slowset));
	cv::imwrite("shortestpathSPFA.png", pathspfa);
	cv::imwrite("shortestpathDijk.png", pathdijk);
	cv::imwrite("shortestpath.png", path);
	cv::Mat pathspfaDist = map.PlotShortestPath(map.SPFA(id1,id2,Time));
	cv::Mat pathdijkDist = map.PlotShortestPath(map.dijkstraDist(id1,id2,Time));
	cv::Mat pathDist = map.PlotShortestPath(map.AStarDist(id1,id2,Time));
	cv::imwrite("shortestpathSPFADist.png", pathspfaDist);
	cv::imwrite("shortestpathDijkDist.png", pathdijkDist);
	cv::imwrite("shortestpathDist.png", pathDist);
}

void YWMap::cmd_queryname()
{
	char buf[80];
	scanf("%s", buf);
	std::vector<std::pair<std::string,point>> name_points = map.queryName(buf);
	if(name_points.size() == 0)
	{
		printf("%s NOT FOUND\n", buf);
		return;
	}
	printf("%s is found %u times\n", buf, name_points.size());
	for(std::pair<std::string,point> name_point : name_points)
	{
		printf("No.%u:%s(%f,%f)\n", map.getNodeIdByIndex(map.nearest(name_point.second)[0]),name_point.first.c_str(),
				name_point.second.get<0>(),name_point.second.get<1>());
	}
}

void YWMap::cmd_querybox()
{
	double minlat,maxlat,minlon,maxlon;
	scanf("%lf%lf%lf%lf", &maxlat, &minlon, &minlat, &maxlon);
	box b(point(minlat,minlon), point(maxlat,maxlon));
	clock_t t = clock();
	std::vector<std::pair<point,unsigned>> result = map.querybox(b);
	t = clock() - t;
	printf("[RTree query box]It takes %fms\n", (float)t / CLOCKS_PER_SEC * 1000);
	cv::Mat node = map.PlotPointInBox(b, result, cv::Scalar(0x24, 0xA7, 0x6D));
	cv::imwrite("querybox.png", node);
}

void YWMap::cmd_querynearest()
{
	double lat,lon;
	int k;
	scanf("%lf%lf%d", &lat, &lon, &k);
	point p(lat,lon);
	clock_t t = clock();
	std::vector<std::pair<point,unsigned>> result = map.nearestForPlot(p,k);
	t = clock() - t;
	printf("[RTree query nearest]It takes %fms\n", (float)t / CLOCKS_PER_SEC * 1000);
	cv::Mat nearest = map.PlotPointNearest(p, result, cv::Scalar(0x24, 0xA7, 0x6D));
	cv::imwrite("nearest.png", nearest);
}

void YWMap::cmd_queryway()
{
	char buf[80];
	scanf("%s", buf);
	unsigned id;
	id = strtoul(buf,NULL,0);
	if(map.waymap.find(id) == map.waymap.end())
	{
		std::vector<std::pair<std::string, unsigned>> ways = map.queryNameWay(buf);
		printf("Way Information:\n");
		double total=0;
		for(auto way:ways)
		{
			if(strcmp(buf,way.first.c_str()) == 0)
			{
				unsigned id = way.second;
				unsigned index = map.waymap[id];
				printf("	ID:%u\n", id);
				printf("	Name:%s\n", way.first.c_str());
				printf("	Dist:%fm\n", map.wayvec[index].dist * 1000);
				printf("	Type:%s\n\n", map.wayvec[index].type.c_str());
				//if(map.wayvec[index].speed >= 40)
					total += map.wayvec[index].dist;
			}
		}
		printf("Total length = %fkm\n", total);
		return;
	}
	const way_struct &way = map.wayvec[map.waymap[id]];
	printf("Way Information:\n");
	printf("	ID:%u\n", id);
	printf("	Name:%s\n", way.name.c_str());
	printf("	Dist:%fm\n", way.dist * 1000);
	printf("	Type:%s\n", way.type.c_str());
}

void YWMap::cmd_querytaxi()
{
	clock_t Time = clock();
	unsigned id;
	short h1,m1,s1,h2,m2,s2;
	scanf("%u %hd:%hd:%hd %hd:%hd:%hd", &id,&h1,&m1,&s1,&h2,&m2,&s2);
	int n,cnt=0;
	scanf("%d",&n);
	std::vector<int> queryvec;
	queryvec.clear();
	for(int i=0;i<n;i++)
	{
		int t;
		scanf("%d",&t);
		queryvec.push_back(t);
	}
	std::sort(queryvec.begin(), queryvec.end(), [](const int &a, const int &b)->bool{return a>b;});
	auto cmp = [](const taxi_struct& t1, const taxi_struct& t2) -> bool {
		if (t1.id < t2.id) return true;
		if (t1.id > t2.id) return false;
		return timecmp(t1.hh,t1.mm,t1.ss,t2.hh,t2.mm,t2.ss) == -1;
		//return t1.id < t2.id;
	};
	int s = std::lower_bound(map.taxiinfo.begin(),map.taxiinfo.end(),taxi_struct(id,h1,m1,s1),cmp) - map.taxiinfo.begin();
	int t = std::upper_bound(map.taxiinfo.begin(),map.taxiinfo.end(),taxi_struct(id,h2,m2,s2),cmp) - map.taxiinfo.begin();
	static std::vector<point> total_path; total_path.clear();
	//printf("[DEBUG] s = %d t = %d\n", s,t);
	for(int i = s; i < t; i++)
	{
		total_path.push_back(point(map.taxiinfo[i].lat,map.taxiinfo[i].lon));
		if(queryvec.size() && ++cnt == queryvec[queryvec.size() - 1])
		{
			map.taxiinfo[i].print();
			queryvec.pop_back();
		}
	}
	Time = clock() - Time;
	printf("[Query No.%u Taxi from %02hd:%02hd:%02hd to %02hd:%02hd:%02hd]%u points get!(%fms used)\n",
		   id,h1,m1,s1,h2,m2,s2,total_path.size(),(float)Time / CLOCKS_PER_SEC * 1000);
	cv::Mat taxipath = map.PlotPath(total_path);
	imwrite("taxipath.png", taxipath);
}

void YWMap::cmd_shortesttest()
{
	int t;
	scanf("%d", &t);
	srand(time(NULL));
	double SPFATime=0, dijkstraTime=0, AStarTime=0, SPFADist=0, dijkstraDist=0, AStarDist=0;
	clock_t Time;
	int cnt1=0, cnt2=0;
	for(int i=0;i<t;i++)
	{
		unsigned index1=random() % map.nodevec.size(), index2=random() % map.nodevec.size();
		unsigned id1 = map.getNodeIdByIndex(map.nearest(map.nodevec[index1].p)[0]);
		unsigned id2 = map.getNodeIdByIndex(map.nearest(map.nodevec[index2].p)[0]);
		printf("==%u=====%u==\n", id1,id2);
		std::set<unsigned> slowset;
		map.SPFA(id1,id2,Time);
		if((float)Time / CLOCKS_PER_SEC > 15){printf("\n");continue;}
		SPFADist += (float)Time / CLOCKS_PER_SEC;
		map.AStarDist(id1,id2,Time); AStarDist += (float)Time / CLOCKS_PER_SEC;
		map.dijkstraDist(id1,id2,Time); dijkstraDist += (float)Time / CLOCKS_PER_SEC;
		cnt1++;

		map.SPFATime(id1,id2,Time,slowset);
		if((float)Time / CLOCKS_PER_SEC > 15){printf("\n");continue;}
		SPFATime += (float)Time / CLOCKS_PER_SEC;
		map.AStarTime(id1,id2,Time,slowset); AStarTime += (float)Time / CLOCKS_PER_SEC;
		map.dijkstraTime(id1,id2,Time,slowset); dijkstraTime += (float)Time / CLOCKS_PER_SEC;
		printf("\n");
		cnt2++;
	}
	printf("[SPFADist] %fms average\n", 1000*SPFADist/cnt1);
	printf("[DijkDist] %fms average\n", 1000*dijkstraDist/cnt1);
	printf("[A\*  Dist] %fms average\n", 1000*AStarDist/cnt1);
	printf("[SPFATime] %fms average\n", 1000*SPFATime/cnt2);
	printf("[DijkTIme] %fms average\n", 1000*dijkstraTime/cnt2);
	printf("[A\*  Time] %fms average\n", 1000*AStarTime/cnt2);
}

void YWMap::cmd_insertpoint()
{
	double lat,lon;
	scanf("%lf%lf",&lat,&lon);
	map.way_node_tree.insert(std::make_pair(point(lat,lon),0));
}

void YWMap::cmd_queryNearestTaxi()
{
	point p;
	double r,time1,time2,x,y;
	int hh,mm,ss;
	scanf("%lf %lf %lf %d:%d:%d %lf", &x, &y, &r, &hh, &mm, &ss, &time2);
	time1 = 3600.0 * hh + 60.0 * mm + ss;
	time2 += time1;
	p = point(x,y);
	map.queryNearestTaxi(p,r,time1,time2);
}

void YWMap::cmd_help()
{
	printf("显示地图：\n");
	printf("	语法：showmap lat lon level\n");
	printf("	解释：level是缩放级别，取值为11到18\n");
	printf("	举例：\n");
	printf("		showmap 31.30186 121.51159 13\n");
	printf("\n");

	printf("查最短路：\n");
	printf("	语法1：shortestpath id1 id2 n ...\n");
	printf("	解释1：id1 id2为两个node的id，n为拥堵路的数量，后面跟n个数代表拥堵路的id\n");
	printf("	举例1：shortestpath 1008057714 1813887498 2 227962891 11635095\n");
	printf("	语法2：shortestpath s1 s2 n ...\n");
	printf("	解释2：s1 s2是点的名称，其它同上\n");
	printf("	举例2：shortestpath 邯郸路 张江校区 2 8886548 8886576\n");
	printf("\n");

	printf("通过名字找坐标和id：\n");
	printf("	语法：queryname name\n");
	printf("	举例：queryname 复旦\n");
	printf("\n");

	printf("查询道路：\n");
	printf("	语法：queryway name/id\n");
	printf("	举例：queryway 肇嘉浜路\n");
	printf("		queryway 世纪大道\n");
	printf("		queryway 272231909\n");
	printf("\n");

	printf("范围兴趣点查询：\n");
	printf("	语法：querynode maxlat minlon minlat maxlon\n");
	printf("	举例：querynode 31.2449 121.4886 31.2349 121.5073\n");
	printf("\n");

	printf("k邻近兴趣点查询：\n");
	printf("	语法：querynearest lat lon k\n");
	printf("	举例：querynearest 31.2392 121.4956 100\n");
	printf("\n");

	printf("最短路性能测试：\n");
	printf("	语法：shortesttest n\n");
	printf("	说明：随机生成起点和终点，测试n次不同最短路算法\n");
	printf("	举例：shortesttest 20\n");
	printf("\n");

	printf("插入兴趣点：\n");
	printf("	语法：insertpoint lat lon\n");
	printf("	举例：\n");
	printf("		querynode 31.2449 121.4886 31.24 121.49\n");
	printf("		insertpoint 31.2440 121.489\n");
	printf("		querynode 31.2449 121.4886 31.24 121.49\n");
	printf("\n");

	printf("以下为出租车相关，在使用以下功能前先输入loadtaxi载入出租车轨迹信息\n\n");

	printf("查询一辆出租车轨迹：\n");
	printf("	语法：querytaxi taxiid starttime endtime n ...\n");
	printf("	说明：n为要查询的点的个数，后面n个数字，代表查询第几个点\n");
	printf("	举例：querytaxi 10944 0:0:1 23:59:59 2 2 3(最后三个数字代表，输出两个点的信息，分别是第二个点和第三个点)\n");
	printf("\n");

	printf("查询去哪里载客：\n");
	printf("	语法：queryneartaxi lat lon r time duration\n");
	printf("	说明：r是搜索半径 time是当前时间 duration是设置搜索范围的时间长度\n");
	printf("	举例：queryneartaxi 31.2363 121.4690 0.01 21:0:0 3600\n");
	printf("	上例为当前时间为21点，所以搜索范围为20点半到21点半\n");
	printf("\n");

	printf("输出得图片可以在同一文件夹中找到对应或相似的文件名\n");
}
