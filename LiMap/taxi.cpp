#include "ywmap.h"
#include "kdtree.h"

KDTree kdt;
extern YWMap map;
void YWMap::loadtaxi()
{
	clock_t Time = clock();
	//freopen("/Users/SpaceQ/iNut/151/Data Structure/PJ/shanghai_taxi_20150401_min.csv","r",stdin);
	FILE *fp=fopen("shanghai_taxi_20150401.csv","r");
	char buf[120];
	int cnt=0;
	double minlat = 180,maxlat = 0,minlon = 180,maxlon = 0;
	while(fgets(buf,sizeof(buf),fp))
	{
		if(strlen(buf) < 10) {
			cnt ++;
			continue;
		}
		taxi_struct tmp;
		tmp.read(buf);
		taxiinfo.push_back(tmp);
		minlat = fmin(minlat, tmp.lat);
		maxlat = fmax(maxlat, tmp.lat);
		minlon = fmin(minlon, tmp.lon);
		maxlon = fmax(maxlon, tmp.lon);
		//tmp.print();
	}
	printf("Taxi Data Load Finished\n");
	printf("Taxi Number = %d\n",cnt);
	printf("Range:");
	printf("lat(%f~%f) lon(%f~%f)\n", minlat, maxlat, minlon, maxlon);
	Time = clock() - Time;
	printf("It takes %fs\n", (float)Time / CLOCKS_PER_SEC);

	/*static std::vector<Node> nodes; nodes.clear();
	static std::vector<std::pair<point,unsigned>> points; points.clear();
	for(int i=1;i<taxiinfo.size(); i++)
		if(taxiinfo[i].time() != taxiinfo[i-1].time() && taxiinfo[i-1].empty && !taxiinfo[i].empty)
		{
			points.push_back(std::make_pair(point(taxiinfo[i].lat,taxiinfo[i].lon),i));
			nodes.push_back(Node(i,taxiinfo[i].lat,taxiinfo[i].lon,double(taxiinfo[i].time())));
		}
	printf("taxi begin number = %d\n", nodes.size());
	cv::Mat test = YWMap::PlotPointInBox(box(point(31.2193-0.5,121.4751-0.5),point(31.2193+0.5,121.4751+0.5)),points);
	imwrite("test.png", test);*/
	int nodesize = 0;
	static Node nodes[55000];
	for(int i=1; i < taxiinfo.size(); i++)
		if(taxiinfo[i].time() != taxiinfo[i-1].time() && taxiinfo[i-1].empty && !taxiinfo[i].empty)
		{
			nodes[nodesize++] = Node(i,taxiinfo[i].lat,taxiinfo[i].lon,double(taxiinfo[i].time()));
		}
	kdt.Build(nodes,0,nodesize-1);
	printf("Build Finished\n");
}

void YWMap::queryNearestTaxi(point p,double r,double time1, double time2)
{
	time1 -= time2 / 2;
	std::vector<int> ret = kdt.CylinderFinder(point3(p.get<0>(),p.get<1>(),time2), time2 - time1, r);
	printf("%d\n", ret.size());
	/*static std::vector<point> points; points.clear();
	for(int i=0;i<ret.size();i++) points.push_back(point(taxiinfo[ret[i]].lat,taxiinfo[ret[i]].lon));
	cv::Mat test = YWMap::PlotPoints(points);
	imwrite("test.png", test);*/
	std::vector<int> cnt;cnt.clear();
	double average=0, maxnum = 0;
	for(int i=0;i<ret.size();i++)
	{
		std::vector<int> tmp;tmp.clear();
		tmp = kdt.CylinderFinder(point3(taxiinfo[ret[i]].lat,taxiinfo[ret[i]].lon,time2), time2 - time1, 0.001);
		cnt.push_back(tmp.size());
	}
	for(int i=0;i<cnt.size();i++) average += cnt[i], maxnum = fmax(maxnum, cnt[i]);
	average /= cnt.size();
	std::vector<std::pair<point, cv::Scalar>> points;
	for(int i=0;i<ret.size(); i++)
	{
		double weights = cnt[i] > average ? (cnt[i] - 2 * average) / maxnum * 6 : -1e20;
		weights = 1.0 / (1 + exp(-weights));
		//printf("%f\n", weights);
		cv::Scalar color = cv::Scalar(255,255 - int(255 * weights),255);
		points.push_back(std::make_pair(point(taxiinfo[ret[i]].lat, taxiinfo[ret[i]].lon), color));
	}
	points.push_back(std::make_pair(p,cv::Scalar(0,0,0)));
	cv::Mat test = YWMap::PlotColorfulPoints(points);
	imwrite("taxirecommend.png", test);
}


int YWMap::timecmp(short h1,short m1,short s1,short h2,short m2,short s2)
{
	if(h1==h2 && m1==m2 && s1==s2)return 0;
	if (h1 < h2 || (h1 == h2 && m1 < m2) || (h1 == h2 && m1 == m2 && s1 < s2) ) return -1;
	return 1;
}
