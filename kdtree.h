#ifndef KDTREE_H
#define KDTREE_H
#include<vector>
#include<algorithm>
#include<cmath>
struct point3
{
	double x,y,z;
	point3(){}
	point3(double x,double y, double z):x(x),y(y),z(z){}
};

struct Node
{
	int id;
	point3 p;
	int type;
	double x() const{return this->p.x;}
	double y() const{return this->p.y;}
	double z() const{return this->p.z;}
	Node(int id=0, double x=0, double y=0, double z=0):id(id),p(x,y,z){}
};

struct KDNode
{
	int id;
	KDNode *l,*r,*fa;
	point3 p1,p2,p;
	int sp;
	KDNode(){}
	KDNode(int id, point3 p1, point3 p2,point3 p):id(id),p1(p1),p2(p2),p(p){}
	void cylinderFinder(std::vector<int> &ret, const point3 &o,const double &h,const double &r);
};

class KDTree
{
public:
	KDTree();
	void Build(Node *node, int l, int r);
	std::vector<int> CylinderFinder(const point3 &o,const double &h,const double &r);
	static bool cylinderIntersection(const point3 &o,const double &h,const double &r,const point3 &p1, const point3 &p2);
	static double disxy(const point3 &p1, const point3 &p2);
	static double disxy(const point3 &o, const point3 &p1, const point3 &p2);
	static bool pointincylinder(const point3&o, const double&h, const double&r, const point3&p);
private:
	KDNode *root;
	KDNode *build(Node *node, int l,int r);

};

#endif // KDTREE_H
