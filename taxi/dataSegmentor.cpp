#include<cstdio>
using namespace std;
int main()
{
	char buf[120];
	int n;
	scanf("%d",&n);
	freopen("shanghai_taxi_20150401.csv","r");
	freopen("shanghai_taxi_20150401_min5.csv","w");
	for(int i=0;i<n;i++)
	{
		fgets(buf,sizeof(buf),stdin);
		printf("%s",buf);
	}
	return 0;
}
