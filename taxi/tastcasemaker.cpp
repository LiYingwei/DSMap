#include<cstdio>
#include<cstring>
#include<vector>
#include<time.h>
#include<cstdlib>
using namespace std;
double minlat = 30.000367,maxlat = 31.912750,minlon = 120.130592,maxlon = 122.236892;
int main()
{
	srand(time(NULL));
	int n;
	scanf("%d",&n);
	for(int i=0;i<n;i++)
	{
		double r1 = ((double) rand() / (RAND_MAX)) ;
		double r2 = ((double) rand() / (RAND_MAX)) ;
		double r3 = ((double) rand() / (RAND_MAX)) ;
		double r4 = ((double) rand() / (RAND_MAX)) ;
		if(r1 > r2) swap(r1,r2);
		if(r3 > r4) swap(r3,r4);
		printf("%f %f %f %f\n",
				minlat + (maxlat - minlat) * r1,
				minlat + (maxlat - minlat) * r2,
				minlon + (maxlon - minlon) * r3,
				minlon + (maxlon - minlon) * r4);
	}
	return 0;
}

	

