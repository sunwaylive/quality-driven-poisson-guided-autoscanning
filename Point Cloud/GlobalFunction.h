

#pragma once
#include <vector>
#include "CMesh.h"
#include "grid.h"

#include "TriMesh.h"
#include "TriMesh_algo.h"
#include "ICP.h"
//#include "LAP_Others/eigen.h"
#include <fstream>
#include <float.h>
#include <QString>
#include <iostream>
#include <time.h>
#include <string>
#include <ctime>
#include <algorithm>
#include <math.h>
#include "ANN/ANN.h"

#define EIGEN_DEFAULT_TO_ROW_MAJOR
#define EIGEN_EXCEPTIONS
const double PI = 3.1415926;

const double EPI = 1e-6;
const double EPI_BOX = 1e-6;

const double BIG = 100000;
//#include <Eigen/Dense>

using namespace std;
using namespace vcg;

//typedef Eigen::MatrixXd Matrix;

#define MyMax(a,b) (((a) > (b)) ? (a) : (b))  
#define MyMin(a,b) (((a) < (b)) ? (a) : (b))  


namespace GlobalFun
{
  struct DesityAndIndex{
    int index;
    double density;
  };

  bool cmp(DesityAndIndex &a, DesityAndIndex &b);

	void computeKnnNeigbhors(vector<CVertex> &datapts, vector<CVertex> &querypts, int numKnn, bool need_self_included, QString purpose);
	void computeEigen(CMesh* _samples);
	void computeEigenIgnoreBranchedPoints(CMesh* _samples);
	void computeEigenWithTheta(CMesh* _samples, double radius);

	void computeAnnNeigbhors(vector<CVertex> &datapts, vector<CVertex> &querypts, int numKnn, bool need_self_included, QString purpose);
	//mesh0: search set, mesh1: 
  void computeBallNeighbors(CMesh* mesh0, CMesh* mesh1, double radius, vcg::Box3f& box);
  double estimateKnnSize(CMesh* mesh0, CMesh* mesh1, double radius, vcg::Box3f& box);

	void static  __cdecl self_neighbors(CGrid::iterator start, CGrid::iterator end, double radius);
	void static  __cdecl other_neighbors(CGrid::iterator starta, CGrid::iterator enda, 
	CGrid::iterator startb, CGrid::iterator endb, double radius);
	void static __cdecl find_original_neighbors(CGrid::iterator starta, CGrid::iterator enda, 
	CGrid::iterator startb, CGrid::iterator endb, double radius); 

	double computeEulerDist(Point3f& p1, Point3f& p2);
	double computeEulerDistSquare(Point3f& p1, Point3f& p2);
	double computeProjDist(Point3f& p1, Point3f& p2, Point3f& normal_of_p1);
	double computeProjDistSquare(Point3f& p1, Point3f& p2, Point3f& normal_of_p1);
	double computePerpendicularDistSquare(Point3f& p1, Point3f& p2, Point3f& normal_of_p1);
	double computePerpendicularDist(Point3f& p1, Point3f& p2, Point3f& normal_of_p1);
	double computeProjPlusPerpenDist(Point3f& p1, Point3f& p2, Point3f& normal_of_p1);
	double getDoubleMAXIMUM();
	vector<int> GetRandomCards(int Max);

	double computeRealAngleOfTwoVertor(Point3f v0, Point3f v1);
	bool isTwoPoint3fTheSame(Point3f& v0, Point3f& v1);
	bool isTwoPoint3fOpposite(Point3f& v0, Point3f& v1);

  double computeTriangleArea_3(Point3f& v0, Point3f& v1, Point3f& v2);
  bool isPointInTriangle_3(Point3f& v0, Point3f& v1, Point3f& v2, Point3f& p);
  double computeMeshLineIntersectPoint(CMesh *target, Point3f& p, Point3f& line_dir, Point3f& result);

  void removeOutliers(CMesh *mesh, double radius, double remove_percent);
  void removeOutliers(CMesh *mesh, double radius, int remove_num);
  void computeICP(CMesh *dst, CMesh *src);
  void downSample(CMesh *dst, CMesh *src, double sample_ratio, bool use_random_downsample = true);
  void clearCMesh(CMesh &mesh);

  void deleteIgnore(CMesh* mesh);
  void recoverIgnore(CMesh* mesh);

  void cutPointSelfSlice(CMesh* mesh, Point3f anchor, Point3f direction, double width);
}

class Timer
{
public:

	void start(const string& str)
	{
		cout << endl;
		starttime = clock();
		mid_start = clock();
		cout << "@@@@@ Time Count Start For: " << str << endl;

		_str = str;
	}

	void insert(const string& str)
	{
		mid_end = clock();
		timeused = mid_end - mid_start;
		cout << "##" << str << "  time used:  " << timeused / double(CLOCKS_PER_SEC) << " seconds." << endl;
		mid_start = clock();
	}

	void end()
	{
		stoptime = clock();
		timeused = stoptime - starttime;
		cout <<  "@@@@ finish	" << _str << "  time used:  " << timeused / double(CLOCKS_PER_SEC) << " seconds." << endl;
		cout << endl;
	}

private:
	int starttime, mid_start, mid_end, stoptime, timeused;
	string _str;
};


class Slice
{
public:
  Slice()
  {
    res = 0;
  }

  //void build_slice(Point3f min, Point3f max, float cell_size);
  //void build_slice(Point3f a, Point3f b, Point3f c, float cell_length);
  vector<CVertex> getSliceNodes(){ return slice_nodes; }

public:

  vector<CVertex> slice_nodes;
  int res;
};

typedef vector<Slice> Slices;

/* Useful code template

(1)
for(int i = 0; i < samples->vert.size(); i++)
{
CVertex& v = samples->vert[i];

for (int j = 0; j < v.neighbors.size(); j++)
{
CVertex& t = samples->vert[v.neighbors[j]];
}
}

(2)
int count = 0;
time.start("Test 2");
CMesh::VertexIterator vi;
Point3f p0 = Point3f(0,0,0);
for(vi = original->vert.begin(); vi != original->vert.end(); ++vi)
{
count += GlobalFun::computeEulerDistSquare(p0, vi->P());
}
cout << count << endl;
time.end();


time.start("Test 1");
for(int i = 0; i < original->vert.size(); i++)
{
CVertex& v = original->vert[i];
count += (p0 - v.P()).SquaredNorm();
}
cout << count << endl;
time.end();
*/

