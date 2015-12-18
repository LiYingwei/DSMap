#include "kdtree.h"

KDTree::KDTree()
{

}

void KDTree::Build(Node *node, int l, int r)
{
	root = build(node, l, r);
}

KDNode* KDTree::build(Node *node, int l, int r)
{
	if(l==r) return new KDNode(node[l].id, node[l].p, node[l].p, node[l].p);
	else if(r<l) return nullptr;
	int mid = (l+r)>>1;
	double minx = 1000, miny = 1000, minz = 1000, maxx = -1000, maxy = -1000, maxz = -1000;
	double varx = 0, vary = 0, varz = 0, avgx = 0, avgy = 0, avgz=0;
	for (int i = l; i <= r; i++) {
			minx = fmin(minx, node[i].x());
			miny = fmin(miny, node[i].y());
			minz = fmin(minz, node[i].z());
			maxx = fmax(maxx, node[i].x());
			maxy = fmax(maxy, node[i].y());
			maxz = fmax(maxz, node[i].z());
			avgx += node[i].x();
			avgy += node[i].y();
			avgz += node[i].z();
		}
	avgx /= r - l + 1;
	avgy /= r - l + 1;
	avgz /= r - l + 1;
	for(int i=l;i<=r;i++)
	{
		varx = pow(node[i].x() - avgx, 2);
		vary = pow(node[i].y() - avgy, 2);
		varz = pow(node[i].z() - avgz, 2);
	}
	int sp=2;
	if(varx >= vary && varx >= varz) //sp = 0, x
	{
		sp=0;
		std::nth_element(node+l,node+mid,node+r+1,[](const Node& a, const Node&b) -> bool {return a.x() < b.x();});
	}
	else if(vary >= varx && vary >= varz) // sp == 1, y
	{
		sp=1;
		std::nth_element(node+l,node+mid,node+r+1,[](const Node& a, const Node&b) -> bool {return a.y() < b.y();});
	}
	else std::nth_element(node+l,node+mid,node+r+1,[](const Node& a, const Node&b) -> bool {return a.z() < b.z();});

	KDNode* ret = new KDNode(node[mid].id, point3(minx,miny,minz), point3(maxx,maxy,maxz), node[mid].p);
	ret->sp = sp;
	ret->l = build(node, l, mid - 1);
	ret->r = build(node, mid+1, r);
	if(ret->l != nullptr) ret->l->fa = ret;
	if(ret->r != nullptr) ret->r->fa = ret;
	return ret;
}

std::vector<int> KDTree::CylinderFinder(const point3 &o,const double &h,const double &r)
{
	static std::vector<int> ret;
	ret.clear();
	root->cylinderFinder(ret,o,h,r);
	return ret;
}

void KDNode::cylinderFinder(std::vector<int> &ret, const point3 &o, const double &h, const double &r)
{
	if(!KDTree::cylinderIntersection(o,h,r,this->p1, this->p2))return;
	if(KDTree::pointincylinder(o,h,r,this->p)) ret.push_back(this->id);
	if(this->l != nullptr) this->l->cylinderFinder(ret, o, h, r);
	if(this->r != nullptr) this->r->cylinderFinder(ret, o, h, r);
}

bool KDTree::cylinderIntersection(const point3 &o, const double &h, const double &r, const point3 &p1, const point3 &p2)
{
	double t2 = o.z, t1 = t2 - h, T2 = p2.z, T1 = p1.z;
	return (disxy(o,p1,p2) < r && (t2 - t1 + T2 - T1) > (fmax(T2,t2) - fmin(T1,t1)));
}

double KDTree::disxy(const point3 &p1, const point3 &p2)
{
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double KDTree::disxy(const point3 &o, const point3 &p1, const point3 &p2)
{
	if(o.x >= p1.x && o.x <= p2.x && o.y>=p1.y && o.y<=p2.y) return 0;
	if(o.x >= p1.x && o.x <= p2.x) return fmin(fabs(o.y - p1.y), fabs(o.y - p2.y));
	if(o.y >= p1.y && o.y <= p2.y) return fmin(fabs(o.x - p1.x) ,fabs(o.x - p2.x));
	return fmin(fmin(disxy(o,p1),disxy(o,p2)),
				fmin(disxy(o,point3(p1.x,p2.y,0)),disxy(o,point3(p2.x,p1.y,0))));
}

bool KDTree::pointincylinder(const point3&o, const double&h, const double&r, const point3&p)
{
	return disxy(p,o) < r && (p.z >= o.z - h && p.z <= o.z);
}
