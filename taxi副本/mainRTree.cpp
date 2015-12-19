#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/box.hpp>

#include<cstdio>
#include<cstring>
#include<assert.h>
#include<vector>
#define rep(i,n) for(int i=0;i<n;i++)
#define pb push_back
#define mp make_pair
#define x first
#define y secnod
using namespace std;
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef std::pair<point, unsigned> nodeinfo;

bgi::rtree< std::pair<point,unsigned> , bgi::quadratic<16> > rtree;

struct taxi_struct
{
	int id;
	short hh,mm,ss;
	short alert;
	double lon,lat;
	short empty;
	char light;
	short high, brake;
	double v,dir;
	short satellite;
	taxi_struct(){}
	taxi_struct(int id,short hh,short mm,short ss,short alert,double lon,double lat,
			short empty,char light,short high,short brake,double v,double dir, short satellite):
		id(id), hh(hh), mm(mm), ss(ss), alert(alert), lon(lon), lat(lat), empty(empty), light(light),
		high(high), brake(brake), v(v), dir(dir), satellite(satellite){}
	void read(char* buf)
	{
		char t[30],date[15],stime[15];
		sscanf(buf,"%d,%[^,],%hd,%lf,%lf,%hd,%c,%hd,%hd,%lf,%lf,%hd",
				   &id,t,&alert,&lon,&lat,&empty,&light,&high,&brake,&v,&dir,&satellite);
		sscanf(t,"%s%s",date,stime);
		sscanf(stime,"%hd:%hd:%hd", &hh,&mm,&ss);
	};
	void print()
	{
		printf("%d,%hd:%hd:%hd,%hd,%lf,%lf,%hd,%c,%hd,%hd,%lf,%lf,%hd\n",
				id,hh,mm,ss,alert,lon,lat,empty,light,high,brake,v,dir,satellite);
	}
	bool operator<(const taxi_struct& o) const
	{
		return lon < o.lon;
	}
};
vector<taxi_struct> taxiinfo;

void readin()
{
	//freopen("/Users/SpaceQ/iNut/151/Data Structure/PJ/shanghai_taxi_20150401_min.csv","r",stdin);
	FILE *fp=fopen("/Users/SpaceQ/iNut/151/Data Structure/PJ/shanghai_taxi_20150401.csv","r");
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
		minlat = min(minlat, tmp.lat);
		maxlat = max(maxlat, tmp.lat);
		minlon = min(minlon, tmp.lon);
		maxlon = max(maxlon, tmp.lon);
		//tmp.print();
	}
	printf("%d\n",cnt);
	printf("minlat = %f\nmaxlat = %f\nminlon = %f\nmaxlon = %f\n", 
			minlat, maxlat, minlon, maxlon);
}
int main()
{
	clock_t t = clock();
	readin();
	t = clock() - t;
	printf("read takes %fs\n", (float)t/ CLOCKS_PER_SEC);

	for(int i=0;i<taxiinfo.size();i++)
	{
		rtree.insert(mp(point(taxiinfo[i].lat,taxiinfo[i].lon),i));
		//if(i%100000==0)printf("%d points inserted\n", i);
	}
	t = clock() - t;
	printf("[RTree]Build Finished!(%fs used)\n", (float)t/CLOCKS_PER_SEC);

	double minlat,maxlat,minlon,maxlon;
	int cnt=0;
	freopen("data_10000.in","r",stdin);
	while(scanf("%lf%lf%lf%lf",&minlat,&maxlat,&minlon,&maxlon)!=EOF)
	{
		cnt ++;
		box query_box(point(minlat,minlon),point(maxlat,maxlon));
		static vector<pair<point,unsigned>> ans;
		ans.clear();
		rtree.query(bgi::intersects(query_box), std::back_inserter(ans));
		if(cnt%1000==0)printf("%d query responsed\n",cnt);
		//for(int i=0;i<ans.size();i++)
		//	printf("%u ",ans[i]);
		//printf("\n");
	}


	printf("[RTree]Query %d times takes %fs(average = %fµs)\n",
			cnt, (float)t/ CLOCKS_PER_SEC, (float)t/ CLOCKS_PER_SEC/ cnt * 1000000);
	
	return 0;
}
