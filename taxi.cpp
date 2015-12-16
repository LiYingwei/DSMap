#include "ywmap.h"

void YWMap::loadtaxi()
{
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
	printf("minlat = %f maxlat = %f minlon = %f maxlon = %f\n", minlat, maxlat, minlon, maxlon);
}

int YWMap::timecmp(short h1,short m1,short s1,short h2,short m2,short s2)
{
	if(h1==h2 && m1==m2 && s1==s2)return 0;
	if (h1 < h2 || (h1 == h2 && m1 < m2) || (h1 == h2 && m1 == m2 && s1 < s2) ) return -1;
	return 1;
}
