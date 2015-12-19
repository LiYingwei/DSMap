#include "ywmap.h"
void YWMap::build_sa()
{
	static int t[500000], t2[500000], c[500000];
	int&n = SA_n; n = 0;
	int m = 256;
	s[n++] = '\n';
	for(int i=0;i<nameList.size(); i++)
	{
		for(int j=0;j<nameList[i].first.size(); j++)
		{
			s[n] = nameList[i].first[j];
			index[n] = i;
			n++;
		}
	}

	//printf("%s", s);

	int i, *x = t, *y = t2;
	for(i = 0; i < m; i++) c[i] = 0;
	for(i = 0; i < n; i++) c[x[i] = s[i]+128]++;
	for(i = 1; i < m; i++) c[i] += c[i-1];
	for(i = n - 1; i >= 0; i--) sa[--c[x[i]]] = i;
	for(int k = 1; k <= n; k<<=1)
	{
		int p = 0;
		for(i = n-k; i < n; i++) y[p++] = i;
		for(i = 0; i < n; i++) if(sa[i] >= k) y[p++] = sa[i] - k;
		for(i = 0; i < m; i++) c[i] = 0;
		for(i = 0; i < n; i++)
		{
			c[x[y[i]]]++;
		}
		for(i = 1; i < m; i++) c[i] += c[i-1];
		for(i = n-1; i >= 0; i--) sa[--c[x[y[i]]]] = y[i];
		std::swap(x,y);
		p = 1; x[sa[0]] = 0;
		for(i = 1; i < n; i++)
			x[sa[i]] = y[sa[i-1]]==y[sa[i]] && y[sa[i-1] + k]==y[sa[i]+k] ? p-1 : p++;
		if(p>=n)break;
		m = p;
	}
	//for(int i=0;i<n;i++)printf("%d %d\n", s[sa[i]],s[sa[i]+1]);
}

void YWMap::build_saWay()
{
	static int t[500000], t2[500000], c[500000];
	int&n = SA_nWay; n = 0;
	int m = 256;
	sWay[n++] = '\n';
	for(int i=0;i<nameListWay.size(); i++)
	{
		for(int j=0;j<nameListWay[i].first.size(); j++)
		{
			sWay[n] = nameListWay[i].first[j];
			indexWay[n] = i;
			n++;
		}
	}

	//printf("%s", s);

	int i, *x = t, *y = t2;
	for(i = 0; i < m; i++) c[i] = 0;
	for(i = 0; i < n; i++) c[x[i] = sWay[i]+128]++;
	for(i = 1; i < m; i++) c[i] += c[i-1];
	for(i = n - 1; i >= 0; i--) saWay[--c[x[i]]] = i;
	for(int k = 1; k <= n; k<<=1)
	{
		int p = 0;
		for(i = n-k; i < n; i++) y[p++] = i;
		for(i = 0; i < n; i++) if(saWay[i] >= k) y[p++] = sa[i] - k;
		for(i = 0; i < m; i++) c[i] = 0;
		for(i = 0; i < n; i++)
		{
			c[x[y[i]]]++;
		}
		for(i = 1; i < m; i++) c[i] += c[i-1];
		for(i = n-1; i >= 0; i--) saWay[--c[x[y[i]]]] = y[i];
		std::swap(x,y);
		p = 1; x[sa[0]] = 0;
		for(i = 1; i < n; i++)
			x[saWay[i]] = y[saWay[i-1]]==y[saWay[i]] && y[saWay[i-1] + k]==y[saWay[i]+k] ? p-1 : p++;
		if(p>=n)break;
		m = p;
	}
	//for(int i=0;i<n;i++)printf("%d %d\n", s[sa[i]],s[sa[i]+1]);
}


int YWMap::cmp_suffix(char *pattern, int p, int m)
{
	//printf("%.*s\n%.*s\n%d\n", m, pattern, m, s+sa[p], strncmp(pattern, (s+sa[p]), m));
	return strncmp(pattern, (s+sa[p]), m);
}

int YWMap::cmp_suffixWay(char *pattern, int p, int m)
{
	//printf("%.*s\n%.*s\n%d\n", m, pattern, m, s+sa[p], strncmp(pattern, (s+sa[p]), m));
	return strncmp(pattern, (sWay+saWay[p]), m);
}


std::vector<std::pair<std::string,point>> YWMap::queryName(char *P)
{
	clock_t t = clock();
	std::vector<std::pair<std::string,point>> ret;
	ret.clear();
	int m = strlen(P);
	const int& n = SA_n;
	//printf("%d %d %d %d\n",m, P[0], P[1], P[2]);
	//if(cmp_suffix(P, 0, m) < 0) return ret;
	//if(cmp_suffix(P, n-1, m) > 0) return ret;
	int L = 0, R = n-1;
	while(R>L)
	{
		//printf("L = %d R = %d\n", L, R);
		int M = (L + R) / 2;
		int res = cmp_suffix(P, M, m);
		if(res <= 0) R = M;
		else L = M + 1;
	}
	while(cmp_suffix(P,L,m) == 0)
	{
		ret.push_back(nameList[index[sa[L]]]);
		//std::cout << nameList[index[sa[L]]].first << std::endl;
		L++;
	}
	t = clock() - t;
	printf("[Suffix Array]%fms used\n", (float)t/CLOCKS_PER_SEC * 1000);
	return ret;
}

std::vector<std::pair<std::string,unsigned>> YWMap::queryNameWay(char *P)
{
	clock_t t = clock();
	std::vector<std::pair<std::string,unsigned>> ret;
	ret.clear();
	int m = strlen(P);
	const int& n = SA_nWay;
	//printf("%d %d %d %d\n",m, P[0], P[1], P[2]);
	//if(cmp_suffix(P, 0, m) < 0) return ret;
	//if(cmp_suffix(P, n-1, m) > 0) return ret;
	int L = 0, R = n-1;
	while(R>L)
	{
		//printf("L = %d R = %d\n", L, R);
		int M = (L + R) / 2;
		int res = cmp_suffix(P, M, m);
		if(res <= 0) R = M;
		else L = M + 1;
	}
	while(cmp_suffix(P,L,m) == 0)
	{
		ret.push_back(nameListWay[indexWay[saWay[L]]]);
		//std::cout << nameList[index[sa[L]]].first << std::endl;
		L++;
	}
	t = clock() - t;
	printf("[Suffix Array]%fms used\n", (float)t/CLOCKS_PER_SEC * 1000);
	return ret;
}
