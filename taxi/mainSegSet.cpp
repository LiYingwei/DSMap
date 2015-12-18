#include<cstdio>
#include<cstring>
#include<assert.h>
#include<vector>
#include<map>
#define rep(i,n) for(int i=0;i<n;i++)
#define pb push_back
#define mp make_pair
#define x first
#define y secnod
using namespace std;
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

struct segmentTreeNode
{
	segmentTreeNode *lch,*rch;
	double minlon, maxlon;
	//vector<pair<double,unsigned> > p; // lat -> index
	map<double,unsigned> p;
};


void pushup(segmentTreeNode &rt)
{
	assert(rt.lch || rt.rch);
	if(rt.lch==NULL)
	{
		rt.minlon = rt.rch -> minlon;
		rt.maxlon = rt.rch -> maxlon;
		rt.p = rt.rch -> p;
	}
	else if(rt.rch==NULL)
	{
		rt.minlon = rt.lch -> minlon;
		rt.maxlon = rt.lch -> maxlon;
		rt.p = rt.lch -> p;
	}
	else
	{
		rt.minlon = min(rt.lch -> minlon, rt.rch -> minlon);
		rt.maxlon = max(rt.lch -> maxlon, rt.rch -> maxlon);
		for(auto it = rt.lch->p.begin(); it != rt.lch -> p.end(); it++) rt.p.insert(*it);
		for(auto it = rt.rch->p.begin(); it != rt.rch -> p.end(); it++) rt.p.insert(*it);
	}
}
int nodenum=0;
void build(int l,int r,segmentTreeNode &rt)
{
	nodenum ++;
	if(l==r)
	{
		rt.lch = NULL;
		rt.rch = NULL;
		rt.minlon = rt.maxlon = taxiinfo[l].lon;
		rt.p.clear();
		rt.p.insert(make_pair(taxiinfo[l].lat, l));
		return;
	}
	int m = (l + r) >> 1;
	if(l<=m)
	{
		rt.lch = new segmentTreeNode();
		build(l,m,*rt.lch);
	}
	if(m<r)
	{
		rt.rch = new segmentTreeNode();
		build(m+1,r,*rt.rch);
	}
	pushup(rt);
}
void query(double minlat,double maxlat,double minlon,double maxlon,const segmentTreeNode &rt, vector<unsigned>& ansvec)
{
	if(minlon <= rt.minlon && rt.maxlon <= maxlon)
	{
		//printf("%f %f\n",rt.minlon, rt.maxlon);
		//for(int i=Lpt;i<=min((int)rt.p.size()-1,Rpt);i++)
		for(auto it=rt.p.lower_bound(minlat); (it!=rt.p.end()) && (it->first <= maxlat); it++)
		{
			/*if(it->first <= maxlat && it->first >= minlat)*/ ansvec.push_back(it->second);
		}
		return;
	}
	//printf("-%f %f\n",rt.minlon, rt.maxlon);
	if(rt.lch!=NULL && minlon <= rt.lch->maxlon)query(minlat,maxlat,minlon,maxlon, *rt.lch, ansvec);
	if(rt.rch!=NULL && maxlon >= rt.rch->minlon)query(minlat,maxlat,minlon,maxlon, *rt.rch, ansvec);
}



vector<unsigned> Query(double minlat,double maxlat,double minlon,double maxlon,const segmentTreeNode &rt)
{
	static vector<unsigned> ret; ret.clear();
	//auto cmp = [] (const pair<double,unsigned> &a,const pair<double,unsigned> &b) -> bool{return a.first < b.first;};
	//int Lpt = lower_bound(rt.p.begin(),rt.p.end(),mp(minlat,0U)) - rt.p.begin();
	//int Rpt = lower_bound(rt.p.begin(),rt.p.end(),mp(maxlat,0U)) - rt.p.begin();
	query(minlat,maxlat,minlon,maxlon,rt,ret);
	return ret;
}



	
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
	t = clock();

	sort(taxiinfo.begin(),taxiinfo.end());
	segmentTreeNode root;
	build(0,taxiinfo.size()-1,root);
	printf("build Finsihed\nnodenum=%d\n",nodenum);
	t = clock() - t;
	printf("Build takes %fs\n", (float)t/ CLOCKS_PER_SEC);
	double minlat,maxlat,minlon,maxlon;
	int cnt=0;
	freopen("data_1000.in","r",stdin);
	t = clock();
	while(scanf("%lf%lf%lf%lf",&minlat,&maxlat,&minlon,&maxlon)!=EOF)
	{
		cnt ++;
		static vector<unsigned> ans;
		ans.clear();
		ans = Query(minlat,maxlat,minlon,maxlon,root);
		//for(int i=0;i<ans.size();i++)
		//	printf("%u ",ans[i]);
		//printf("\n");
	}
	t = clock() - t;
	printf("Query %d times takes %fs(average = %fµs)\n",
			cnt, (float)t/ CLOCKS_PER_SEC, (float)t/ CLOCKS_PER_SEC/ cnt * 1000000);
	
	return 0;
}
