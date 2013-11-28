#pragma once

//#include <CGAL/wlop_simplify_and_regularize_point_set_test_AABB_tree.h>
#include <vector>
#include <time.h>
#include <iostream>

#include "GlobalFunction.h"
#include "PointCloudAlgorithm.h"
#include "normal_extrapolation.h"

//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/wlop_simplify_and_regularize_point_set.h>
//#include <CGAL/Timer.h>
//#include <CGAL/tags.h>
//#include <CGAL/Simple_cartesian.h>


// types
//typedef CGAL::Simple_cartesian<double> Kernel;
//typedef Kernel::Point_3 Point;

using namespace std;
using namespace vcg;

// better code is going to be in CGAL 
class WLOP : public PointCloudAlgorithm
{
public:
	WLOP(RichParameterSet* para);
	~WLOP(void);
public:

	void run();
	void setInput(DataMgr* pData);
	RichParameterSet* getParameterSet(){ return para; }
	void setParameterSet(RichParameterSet* _para){para = _para;}
	void clear();

	void setFirstIterate();
  int getIterateNum(){ return nTimeIterated; }
	double getErrorX(){return error_x;}


protected:
	WLOP(){}

private:
	void input(CMesh* _samples, CMesh* _original);
	void initVertexes();

	double iterate();
	void computeAverageTerm(CMesh* samples, CMesh* original);
	void computeRepulsionTerm(CMesh* samples);

	void computeDensity(bool isOriginal, double radius);
	void recomputePCA_Normal();


private:
	RichParameterSet* para;

private:
	CMesh* samples;
	CMesh* original;

	Box3f box;
	int nTimeIterated;
	double error_x;

	vector<double> samples_density;
	vector<double> original_density;

	vector<Point3f> repulsion;
	vector<double>  repulsion_weight_sum;

	vector<Point3f> average;
	vector<double>  average_weight_sum;

	vector<CVertex> mesh_temp;
};
