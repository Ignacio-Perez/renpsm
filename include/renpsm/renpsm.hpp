#ifndef _RENPSM_HPP_
#define _RENPSM_HPP_

#include <random>
#include <vector>
#include <list>
#include <chrono>
#include <assert.h>

namespace renpsm
{

struct Point
{
	int x;
	int y;

};

class Obstacles
{
public:
	Obstacles() {}
	virtual ~Obstacles() {}
	virtual double getDistanceToNearestObstacle(int i, int j) = 0;
};



class Membrane
{
public:
	Membrane(Membrane *parent, int label, int position_x, int position_y);
	~Membrane();
	double distance(int x, int y) {return (y1-x)*(y1-x) + (y2-y)*(y2-y);}

	int childsSize() const {return childs.size();}
	Membrane *getChild(int i) const {return childs[i];}

	int getX() {return y1;}
	int getY() {return y2;}
	Membrane *getParent() {return parent;}

private:
	int label;
	int y1; // position x
	int y2; // position y

	Membrane *parent;
	
	std::vector<Membrane*> childs;
};


class BidirectionalRRT
{
public:
	BidirectionalRRT();
	~BidirectionalRRT();

	void start(int x_init, int y_init, int x_end, int y_end, int x_max, int y_max, double d_min, double delta_t, Obstacles *obstacles, double robotRadius);

	void step();

	Membrane *getRootA() {return root_a;}
	Membrane *getRootB() {return root_b;}

	int getSeed() {return seed;}
	void setSeed(int seed) {BidirectionalRRT::seed = seed;gen.seed(seed);}

	bool isStarted() {return started;}

	std::list<Point>& getPath() {return path;}

private:

	int encode(int i, int j);
	void decode(int label, int &i, int &j);	


	Membrane *getClosestMembrane(std::vector<Membrane *> mu, int x, int y, double &min_distance);
	
	Membrane *createChild(Membrane *parent, int x, int y, double distance);

	double d_min;
	double delta_t;

	int p; 
	int q;  
	
	Membrane *root_a;
	Membrane *root_b;

	std::vector<Membrane*> mu_a;
	std::vector<Membrane*> mu_b;
	int seed;
	std::mt19937 gen;

	bool started;

	Obstacles *obstacles;

	double robotRadius;

	std::list<Point> path;
	
	

	
};


Membrane::Membrane(Membrane *parent, int label, int position_x, int position_y)
: label(label),
  y1(position_x),
  y2(position_y),
  parent(parent)
{
	if (parent!=NULL) {
		parent->childs.push_back(this);
	}
}

Membrane::~Membrane()
{
	for (unsigned i=0;i<childs.size();i++) {
		delete childs[i];
	}
}


BidirectionalRRT::BidirectionalRRT()
 :
  root_a(NULL),
  root_b(NULL),
  seed(std::chrono::system_clock::now().time_since_epoch().count()),
  gen(seed),
  started(false)
{
	
}

BidirectionalRRT::~BidirectionalRRT()
{
	delete root_a;
	delete root_b;
}


int BidirectionalRRT::encode(int i, int j)
{
	return i*q + j;

}

void BidirectionalRRT::start(int x_init, int y_init, int x_end, int y_end, int x_max, int y_max, double d_min, double delta_t, Obstacles *obstacles, double robotRadius)
{
	//std::cout<<x_init<<" "<<y_init<<" "<<x_end<<" "<<y_end<<" "<<x_max<<" "<<y_max<<" "<<d_min<<" "<<delta_t<<std::endl;
	BidirectionalRRT::d_min = d_min;
	BidirectionalRRT::delta_t = delta_t;
	BidirectionalRRT::p = x_max;
	BidirectionalRRT::q = y_max;
	BidirectionalRRT::obstacles = obstacles;
	BidirectionalRRT::robotRadius = robotRadius;
	delete root_a;
	delete root_b;
	mu_a.clear();
	mu_b.clear();
	path.clear();

	root_a = new Membrane(NULL,encode(x_init,y_init),x_init,y_init);
	root_b = new Membrane(NULL,encode(x_end,y_end),x_end,y_end);

	mu_a.push_back(root_a);
	mu_b.push_back(root_b);

	started=true;
}



Membrane *BidirectionalRRT::getClosestMembrane(std::vector<Membrane *> mu, int x, int y, double &min_distance)
{
	min_distance = 999999999;
	Membrane *closest = NULL;
	for (unsigned i=0;i<mu.size();i++) {
		double distance = mu[i]->distance(x,y);
		if (distance < min_distance) {
			min_distance = distance;
			closest = mu[i];
		}

	}
	return closest;
	
}

Membrane *BidirectionalRRT::createChild(Membrane *parent, int x, int y, double distance)
{
	assert(parent!=NULL);
	// If the distance is less than a thresshold, Flag_{mem} is activated, no child created
	if (distance<d_min) {
		return NULL;
	}

	// Compute the U vector
	
	
	double u1 = (x - parent->getX())/distance;
	double u2 = (y - parent->getY())/distance;
	

	// Compute the position of the new membrane
	double x1 = parent->getX() + u1 * delta_t;
	double y1 = parent->getY() + u2 * delta_t;

	
	int i = std::round(x1);
	int j = std::round(y1);

	if (i<0 || i>=p || j<0 || j>=q) {
		return NULL;
	}


	double x0 = parent->getX();
	double y0 = parent->getY();
	double inc_x = (u1 * delta_t)/10.0;
	double inc_y = (u2 * delta_t)/10.0;	

	for (int i=0;i<10;i++) {
		
		if (obstacles->getDistanceToNearestObstacle(x0,y0)<=robotRadius) {
			return NULL;
		}
		x0 += inc_x;
		y0+= inc_y;
	}

	
	

	// Create new membrane
	return new Membrane(parent,encode(i,j),i,j);


	
}



void BidirectionalRRT::step()
{
	if (!started) {
		return;
	}

	// Step 1: Four random numbers are generated in the shared memory
	std::uniform_int_distribution<int> dist1(0,p-1);
	std::uniform_int_distribution<int> dist2(0,q-1);

	int x1 = dist1(gen);
	int x2 = dist2(gen);
	
	int x3 = dist1(gen);
	int x4 = dist2(gen);
	
	// Step 2-6: Each membrane computes the distance to the generated positions
	//		The closest membranes are obtained, that is Near_a and Near_b

	
	double min_distance_a=9999999999;
	double min_distance_b=9999999999;
	Membrane *closest_a=NULL;	
	Membrane *closest_b=NULL;	

	
	closest_a=getClosestMembrane(mu_a,x1,x2,min_distance_a);
	closest_b=getClosestMembrane(mu_b,x3,x4,min_distance_b);

	

	// Step 7-13: New child membranes are created in mu_a and mu_b
	Membrane *new_a = createChild(closest_a,x1,x2,std::sqrt(min_distance_a));
	Membrane *new_b = createChild(closest_b,x3,x4,std::sqrt(min_distance_b));
	
	if (new_a!=NULL) { 
		mu_a.push_back(new_a);
		min_distance_b=9999999999;
		closest_b = getClosestMembrane(mu_b,new_a->getX(),new_a->getY(),min_distance_b);
		if (min_distance_b<d_min) {
			Point p;
			while (new_a!=NULL) {
				p.x = new_a->getX();
				p.y = new_a->getY();
				path.push_front(p);
				new_a = new_a->getParent();
			}
			while (closest_b!=NULL) {
				p.x = closest_b->getX();
				p.y = closest_b->getY();
				path.push_back(p);
				closest_b = closest_b->getParent();
			}
			started=false;
		}
	}

	if (new_b!=NULL && started) {
		mu_b.push_back(new_b);
		min_distance_a=9999999999;
		closest_a = getClosestMembrane(mu_a,new_b->getX(),new_b->getY(),min_distance_a);
		if (min_distance_a<d_min) {
			Point p;
			while (closest_a!=NULL) {
				p.x = closest_a->getX();
				p.y = closest_a->getY();
				path.push_front(p);
				closest_a = closest_a->getParent();
			}
			while (new_b!=NULL) {
				p.x = new_b->getX();
				p.y = new_b->getY();
				path.push_back(p);
				new_b = new_b->getParent();
			}

			started=false;
		}
	}
	//if (!started) {
	//	std::cout<<"FINISH"<<std::endl;
	//}



}


}




#endif
