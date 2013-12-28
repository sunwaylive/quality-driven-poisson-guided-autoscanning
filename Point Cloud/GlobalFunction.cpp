#include <iostream>
#include <fstream>
#include <algorithm>
#include <stdlib.h>
#include <assert.h>
#include "tbb/parallel_for.h"

#include "grid.h"
#include "GlobalFunction.h"

#define LINKED_WITH_TBB
//#ifdef LINKED_WITH_TBB
//#undef  LINKED_WITH_TBB
//#endif

using namespace vcg;
using namespace std;
using namespace tri;


void GlobalFun::find_original_neighbors(CGrid::iterator starta, CGrid::iterator enda, 
	CGrid::iterator startb, CGrid::iterator endb, double radius) 
{	

	double radius2 = radius*radius;
	double iradius16 = -4/radius2;
	//const double PI = 3.1415926;

	for(CGrid::iterator dest = starta; dest != enda; dest++) 
	{
		CVertex &v = *(*dest);

		Point3f &p = v.P();
		for(CGrid::iterator origin = startb; origin != endb; origin++)
		{
			CVertex &t = *(*origin);

			Point3f &q = t.P();
			Point3f diff = p-q;

			double dist2 = diff.SquaredNorm();

			if(dist2 < radius2) 
			{                          
				v.original_neighbors.push_back((*origin)->m_index);
			}
		}
	}
}

// get neighbors
void GlobalFun::self_neighbors(CGrid::iterator start, CGrid::iterator end, double radius)
{
	double radius2 = radius*radius;
	for(CGrid::iterator dest = start; dest != end; dest++)
	{
		CVertex &v = *(*dest);
		Point3f &p = v.P();


		for(CGrid::iterator origin = dest+1; origin != end; origin++)
		{
			CVertex &t = *(*origin);
			Point3f &q = t.P();
			Point3f diff = p-q;
			double dist2 = diff.SquaredNorm();
			if(dist2 < radius2) 
			{   
				v.neighbors.push_back((*origin)->m_index);
				t.neighbors.push_back((*dest)->m_index);
			}
		}
	}
}

void GlobalFun::other_neighbors(CGrid::iterator starta, CGrid::iterator enda, 
	CGrid::iterator startb, CGrid::iterator endb, double radius)
{
	double radius2 = radius*radius;
	for(CGrid::iterator dest = starta; dest != enda; dest++)
	{
		CVertex &v = *(*dest);
		Point3f &p = v.P();

		for(CGrid::iterator origin = startb; origin != endb; origin++)
		{
			CVertex &t = *(*origin);
			Point3f &q = t.P();
			Point3f diff = p-q;
			double dist2 = diff.SquaredNorm();
			if(dist2 < radius2) 
			{   
				v.neighbors.push_back((*origin)->m_index);
				t.neighbors.push_back((*dest)->m_index);
			}
		}
	}
}


void GlobalFun::computeBallNeighbors(CMesh* mesh0, CMesh* mesh1, double radius, vcg::Box3f& box)
{
	if (radius < 0.0001)
	{
		cout << "too small grid!!" << endl; 
		return;
	}
	//mesh1 should be original

	//cout << "compute_Bll_Neighbors" << endl;
	//cout << "radius: " << radius << endl;

	CGrid samples_grid;
	samples_grid.init(mesh0->vert, box, radius);
	//cout << "finished init" << endl;

	if (mesh1 != NULL)
	{
		for (int i = 0; i < mesh0->vn; i++)
		{
			mesh0->vert[i].original_neighbors.clear();
		}

		CGrid original_grid;
		original_grid.init(mesh1->vert, box, radius); // This can be speed up
		samples_grid.sample(original_grid, find_original_neighbors);
	}
	else
	{
		for (int i = 0; i < mesh0->vn; i++)
		{
			mesh0->vert[i].neighbors.clear();
		}

		samples_grid.iterate(self_neighbors, other_neighbors);
	}

}

double GlobalFun::estimateKnnSize(CMesh* samples, CMesh* original, double radius, vcg::Box3f& box)
{
  computeBallNeighbors(samples, original, radius, box);

  double sum_neighbor = 0;
  for(int i = 0; i < samples->vert.size(); i++)
  {
    CVertex& v = samples->vert[i];

    sum_neighbor += v.original_neighbors.size();
  }

  cout << "estimated original KNN: " << sum_neighbor / samples->vert.size() << endl;
  return sum_neighbor / samples->vert.size();
}

void GlobalFun::computeAnnNeigbhors(vector<CVertex> &datapts, vector<CVertex> &querypts, int knn, bool need_self_included = false, QString purpose = "?_?")
{
	cout << endl <<"Compute ANN for: " << purpose.toStdString() << endl;
	int numKnn = knn + 1;

	if (querypts.size() <= numKnn+2)
	{
		vector<CVertex>::iterator vi;
		for(vi = datapts.begin(); vi != datapts.end(); ++vi)
			for(int j = 0; j < 3; j++)
				vi->neighbors.clear();

		return;
	}

	int					    nPts;			 // actual number of data points
	ANNpointArray		dataPts;	 // data points
  ANNpoint			  queryPt;	 // query point
  ANNidxArray			nnIdx;		 // near neighbor indices
  ANNdistArray		dists;		 // near neighbor distances
  ANNkd_tree*			kdTree;		 // search structure
  int			k				= numKnn;			      // number of nearest neighbors
  int			dim			= 3;			          // dimension
  double	eps			= 0;			          // error bound
	int			maxPts	= numKnn + 3000000;	// maximum number of data points

	if (datapts.size() >= maxPts)
	{
		cout << "Too many data" << endl;
		return;
	}

	queryPt = annAllocPt(dim);					// allocate query point
	dataPts = annAllocPts(maxPts, dim);	// allocate data points
	nnIdx   = new ANNidx[k];						// allocate near neigh indices
	dists   = new ANNdist[k];						// allocate near neighbor dists

	vector<CVertex>::iterator vi;
	int index = 0;
	for(vi = datapts.begin(); vi != datapts.end(); ++vi)
	{
		for(int j = 0; j < 3; j++)
			dataPts[index][j] = double(vi->P()[j]); 

		index++;
	}

  nPts = datapts.size();	 // read data points
	kdTree = new ANNkd_tree( // build search structure
		dataPts,					     // the data points
		nPts,						       // number of points
		dim);						       // dimension of space

	for (vi = querypts.begin(); vi != querypts.end(); ++vi) 
	{
		vi->neighbors.clear();
		for (int j = 0; j < 3; j++) 
			queryPt[j] = vi->P()[j];

		kdTree->annkSearch( // search
			queryPt,					// query point
			k,								// number of near neighbors
			nnIdx,						// nearest neighbors (returned)
			dists,						// distance (returned)
			eps);							// error bound

		for (int k = 1; k < numKnn; k++)
			vi->neighbors.push_back(nnIdx[k]);
	}

	delete [] nnIdx; // clean things up
	delete [] dists;
	delete kdTree;
	annClose();			 // done with ANN
}


void GlobalFun::computeKnnNeigbhors(vector<CVertex> &datapts, vector<CVertex> &querypts, int numKnn, bool need_self_included = false, QString purpose = "?_?")
{
	if (querypts.size() <= numKnn+1)
	{
		vector<CVertex>::iterator vi;
		for(vi = datapts.begin(); vi != datapts.end(); ++vi)
		{
			vi->neighbors.clear();	
		}
		return;
	}

	bool isComputingOriginalNeighbor = false;
	//if (!datapts.empty() && datapts[0].is_original)
	//{
	//	isComputingOriginalNeighbor = true;
	//}

	int starttime, stoptime, timeused;
	starttime = clock();

	cout << endl;
	cout << "compute KNN Neighbors for: " << purpose.toStdString() << endl;


	ofstream outfile1;
	ofstream outfile2;
	float val;

	outfile1.open("point_cloud.txt", ofstream::binary);
	outfile2.open("query.txt", ofstream::binary);

	val = datapts.size();
	outfile1.write((char *)(&val), sizeof(float));
	val = querypts.size();
	outfile2.write((char *)(&val), sizeof(float));
	val = 3;
	outfile1.write((char *)(&val), sizeof(float));
	val = 4;
	outfile2.write((char *)(&val), sizeof(float));


	vector<CVertex>::iterator vi;
	for(vi = datapts.begin(); vi != datapts.end(); ++vi)
	{
		for(int j = 0; j < 3; j++)
		{
			val = vi->P()[j];
			outfile1.write((char *)(&val), sizeof(float));
		}
	}


	for (vi = querypts.begin(); vi != querypts.end(); ++vi) 
	{
		for (int j = 0; j < 3; j++) 
		{
			val = vi->P()[j];
			outfile2.write((char *)(&val), sizeof(float));
		}
		val = 0;
		outfile2.write((char *)(&val), sizeof(float));
	}

	outfile1.close();
	outfile2.close();

	char mycmd[100];
	sprintf(mycmd, "RG_NearestNeighbors.exe point_cloud.txt query.txt result.txt %d", numKnn+1);
	//sprintf(mycmd, "RG_NearestNeighbors.exe point_cloud.txt query.txt result.txt", numKnn+1);

	//cout << mycmd;

	system(mycmd); 

	//cout << "knn_neighbor file saved\n";

	//clean querypts 
	for (vi = querypts.begin(); vi != querypts.end(); ++vi)
	{
		if (isComputingOriginalNeighbor)
		{
			vi->original_neighbors.clear();
		}
		else
		{
			vi->neighbors.clear();
		}
	}

	ifstream infile;
	float size[2];
	int row,col;
	float *data;

	infile.open ("result.txt", ifstream::binary);
	infile.read((char*)size, 2*sizeof(float));
	row = (int)size[0];
	col = (int)size[1];
	data = new float [row*col];
	infile.read((char*)data,row*col*sizeof(float));
	infile.close();

	for (int idx = 0; idx < row; idx++)
	{

		CVertex &v = querypts[(int)data[idx*col+1]-1];
		if (isComputingOriginalNeighbor)
		{
			v.original_neighbors.push_back((int)data[idx*col]-1);
		}
		else
		{
			v.neighbors.push_back((int)data[idx*col]-1);
		}
	}

	if (!need_self_included)// slow solution...
	{
		for(int i = 0; i < querypts.size(); i++)
		{
			CVertex& v = querypts[i];
			v.neighbors.erase(v.neighbors.begin());
		}
	}


	delete[] data;
	//cout << "compute_knn_neighbor end." << endl << endl;

	stoptime = clock();
	timeused = stoptime - starttime;
	cout << "KNN time used:  " << timeused/double(CLOCKS_PER_SEC) << " seconds." << endl;
	cout << endl;
}		


vector<int> GlobalFun::GetRandomCards(int Max)
{
	vector<int> nCard(Max, 0);
	srand(time(NULL));
	for(int i=0; i < Max; i++)
	{
		nCard[i] = i;
	}
	random_shuffle(nCard.begin(), nCard.begin() + Max);


	return nCard;
}


void GlobalFun::computeEigenIgnoreBranchedPoints(CMesh* _samples)
{
	vector<vector<int> > neighborMap;

	typedef vector<CVertex>::iterator VertexIterator;

	VertexIterator begin = _samples->vert.begin();
	VertexIterator end = _samples->vert.end();

	neighborMap.assign(end - begin, vector<int>());

	int curr_index = 0;
	for (VertexIterator iter=begin; iter!=end; ++iter, curr_index++)
	{
    if(iter->neighbors.size() <= 3)
    {
      iter->eigen_confidence = 0.5;
      continue;
    }

		//neighborMap[curr_index].push_back(curr_index);
		for(int j = 0; j < iter->neighbors.size(); j++)
		{
			CVertex& t = _samples->vert[iter->neighbors[j]];
			if (t.is_skel_branch || t.is_ignore)
			{
				continue;
			}
			neighborMap[curr_index].push_back(iter->neighbors[j]);
		}
	}

	int currIndex = 0;
	for (VertexIterator iter=begin; iter!=end; iter++, currIndex++)
	{
		int neighbor_size = neighborMap[currIndex].size();

		if (neighbor_size < 3)
		{
			iter->eigen_confidence = 0.95;
			iter->eigen_vector0 = Point3f(0, 0, 0);

			continue;
		}

		Matrix33d covariance_matrix;
		Point3f diff;
		covariance_matrix.SetZero();
		int neighborIndex = -1;

		for (unsigned int n=0; n<neighbor_size; n++)
		{
			neighborIndex = neighborMap[currIndex][n];
			if(neighborIndex < 0)
				break;
			VertexIterator neighborIter = begin + neighborIndex;

			diff = iter->P() - neighborIter->P();

			for (int i=0; i<3; i++)
				for (int j=0; j<3; j++)
					covariance_matrix[i][j] += diff[i]*diff[j];
		}

		Point3f   eigenvalues;
		Matrix33d	eigenvectors;
		int required_rotations;
		vcg::Jacobi< Matrix33d, Point3f >(covariance_matrix, eigenvalues, eigenvectors, required_rotations);
		vcg::SortEigenvaluesAndEigenvectors< Matrix33d, Point3f >(eigenvalues, eigenvectors);

		double sum_eigen_value = (eigenvalues[0] + eigenvalues[1] + eigenvalues[2]);
		iter->eigen_confidence = eigenvalues[0] / sum_eigen_value;

		for (int d=0; d<3; d++)
			iter->eigen_vector0[d] = eigenvectors[d][0];
		for (int d=0; d<3; d++)
			iter->eigen_vector1[d] = eigenvectors[d][1];
		for (int d=0; d<3; d++)
			iter->N()[d] = eigenvectors[d][2];

		iter->eigen_vector0.Normalize();
		iter->eigen_vector1.Normalize();
		iter->N().Normalize();
	}
}

void GlobalFun::computeEigen(CMesh* _samples)
{
	vector<vector<int> > neighborMap;

	typedef vector<CVertex>::iterator VertexIterator;

	VertexIterator begin = _samples->vert.begin();
	VertexIterator end = _samples->vert.end();

	int curr_index = 0;

	int currIndex = 0;
	for (VertexIterator iter=begin; iter!=end; iter++, currIndex++)
	{
		Matrix33d covariance_matrix;
		Point3f diff;
		covariance_matrix.SetZero();
		int neighbor_size = iter->neighbors.size();
		for (unsigned int n=0; n<neighbor_size; n++)
		{
			Point3f& tP =_samples->vert[iter->neighbors[n]].P();
			diff = iter->P() - tP;

			for (int i=0; i<3; i++)
				for (int j=0; j<3; j++)
					covariance_matrix[i][j] += diff[i]*diff[j];
		}


		Point3f   eigenvalues;
		Matrix33d	eigenvectors;
		int required_rotations;
		vcg::Jacobi< Matrix33d, Point3f >(covariance_matrix, eigenvalues, eigenvectors, required_rotations);
		vcg::SortEigenvaluesAndEigenvectors< Matrix33d, Point3f >(eigenvalues, eigenvectors);


		double sum_eigen_value = (eigenvalues[0] + eigenvalues[1] + eigenvalues[2]);


		iter->eigen_confidence = eigenvalues[0] / sum_eigen_value;

		for (int d=0; d<3; d++)
			iter->eigen_vector0[d] = eigenvectors[d][0];
		for (int d=0; d<3; d++)
			iter->eigen_vector1[d] = eigenvectors[d][1];
		for (int d=0; d<3; d++)
			iter->N()[d] = eigenvectors[d][2];

		iter->eigen_vector0.Normalize();
		iter->eigen_vector1.Normalize();
		iter->N().Normalize();
	}

}




void GlobalFun::computeEigenWithTheta(CMesh* _samples, double radius)
{
	vector<vector<int> > neighborMap;

	typedef vector<CVertex>::iterator VertexIterator;

	VertexIterator begin = _samples->vert.begin();
	VertexIterator end = _samples->vert.end();

	neighborMap.assign(end - begin, vector<int>());

	int curr_index = 0;

	for (VertexIterator iter=begin; iter!=end; iter++, curr_index++)
	{
		if(iter->neighbors.size() <= 3)
		{
			iter->eigen_confidence = 0.5;
			continue;
		}

		for(int j = 0; j < iter->neighbors.size(); j++)
		{
			neighborMap[curr_index].push_back(iter->neighbors[j]);
		}
	}

	double radius2 = radius*radius;
	double iradius16 = -1/radius2; 

	int currIndex = 0;
	for (VertexIterator iter=begin; iter!=end; iter++, currIndex++)
	{
    if(iter->neighbors.size() <= 3)
    {
      iter->eigen_confidence = 0.5;
      continue;
    }

		Matrix33d covariance_matrix;
		Point3f diff;
		covariance_matrix.SetZero();
		int neighborIndex = -1;
		int neighbor_size = iter->neighbors.size();
		for (unsigned int n=0; n<neighbor_size; n++)
		{
			neighborIndex = neighborMap[currIndex][n];
			if(neighborIndex < 0)
				break;
			VertexIterator neighborIter = begin + neighborIndex;

			diff = iter->P() - neighborIter->P();

			Point3f vm = iter->N();
			Point3f tm = neighborIter->N();
			double dist2 = diff.SquaredNorm();
			double theta = exp(dist2*iradius16);

			for (int i=0; i<3; i++)
				for (int j=0; j<3; j++)
					covariance_matrix[i][j] += diff[i]*diff[j] * theta;
		}

		Point3f   eigenvalues;
		Matrix33d	eigenvectors;
		int required_rotations;
		vcg::Jacobi< Matrix33d, Point3f >(covariance_matrix, eigenvalues, eigenvectors, required_rotations);
		vcg::SortEigenvaluesAndEigenvectors< Matrix33d, Point3f >(eigenvalues, eigenvectors);


		double sum_eigen_value = (eigenvalues[0] + eigenvalues[1] + eigenvalues[2]);

		iter->eigen_confidence = eigenvalues[0] / sum_eigen_value;

		for (int d=0; d<3; d++)
			iter->eigen_vector0[d] = eigenvectors[d][0];
		for (int d=0; d<3; d++)
			iter->eigen_vector1[d] = eigenvectors[d][1];
		for (int d=0; d<3; d++)
			iter->N()[d] = eigenvectors[d][2];

		iter->eigen_vector0.Normalize();
		iter->eigen_vector1.Normalize();
		iter->N().Normalize();
	}
}



double GlobalFun::computeEulerDist(Point3f& p1, Point3f& p2)
{
	double dist2 = (p1-p2).SquaredNorm();
	if (dist2 < 1e-8 || dist2 > 1e8)
	{
		return 0;
	}
	return sqrt(dist2);
}

double GlobalFun::computeEulerDistSquare(Point3f& p1, Point3f& p2)
{
	return (p1-p2).SquaredNorm();
}

double GlobalFun::computeProjDist(Point3f& p1, Point3f& p2, Point3f& normal_of_p1)
{
	return (p2-p1) * normal_of_p1.Normalize();
}



double GlobalFun::computeProjDistSquare(Point3f& p1, Point3f& p2, Point3f& normal_of_p1)
{
	double proj_dist = computeProjDist(p1, p2, normal_of_p1);
	return proj_dist * proj_dist;
}

double GlobalFun::computePerpendicularDistSquare(Point3f& p1, Point3f& p2, Point3f& normal_of_p1)
{
	//Point3f v_p2_p1 = p1-p2;
	//double proj_dist = computeProjDist(p1, p2, normal_of_p1);
	//Point3f v_proj = /*p1 + */normal_of_p1 * proj_dist;
	//   return (v_p2_p1 + v_proj).SquaredNorm();
	double proj_dist = computeProjDist(p1, p2, normal_of_p1);
	Point3f proj_p = p1 + normal_of_p1 * proj_dist;
	return (proj_p - p2).SquaredNorm();
}



double GlobalFun::computePerpendicularDist(Point3f& p1, Point3f& p2, Point3f& normal_of_p1)
{
	return sqrt(computePerpendicularDistSquare(p1, p2, normal_of_p1));
}

double GlobalFun::computeProjPlusPerpenDist(Point3f& p1, Point3f& p2, Point3f& normal_of_p1)
{
	normal_of_p1.Normalize();
	double proj_dist = GlobalFun::computeProjDist(p1, p2, normal_of_p1);

	if (proj_dist <= 0)
	{
		return -1.;
	}

	Point3f proj_p = p1 + normal_of_p1 * proj_dist;
	double perpend_dist = sqrt((proj_p - p2).SquaredNorm());
	double eular_dist = GlobalFun::computeEulerDist(p1, p2);
	return eular_dist + perpend_dist;
	/*return proj_dist  * 0.5 + perpend_dist;*/
}



double GlobalFun::getDoubleMAXIMUM()
{  
	return (numeric_limits<double>::max)();
}






bool GlobalFun::isTwoPoint3fTheSame(Point3f& v0, Point3f& v1)
{
	if (abs(v0[0] - v1[0]) < 1e-7 &&  
		abs(v0[1] - v1[1]) < 1e-7 && 
		abs(v0[2] - v1[2]) < 1e-7)
	{
		return true;
	}

	return false;

}

bool GlobalFun::isTwoPoint3fOpposite(Point3f& v0, Point3f& v1)
{
	if (abs(-v0[0] - v1[0]) < 1e-7 &&  
		abs(-v0[1] - v1[1]) < 1e-7 && 
		abs(-v0[2] - v1[2]) < 1e-7)
	{
		return true;
	}

	return false;
}


double GlobalFun::computeRealAngleOfTwoVertor(Point3f v0, Point3f v1)
{
	v0.Normalize();
	v1.Normalize();


	if (isTwoPoint3fTheSame(v0, v1))
	{
		return 0;
	}

	if (isTwoPoint3fOpposite(v0, v1))
	{
		return 180;
	}

	double angle_cos = v0 * v1;
	if (angle_cos > 1)
	{
		angle_cos = 0.99;
	}
	if (angle_cos < -1)
	{
		angle_cos = -0.99;
	}
	if (angle_cos > 0 && angle_cos < 1e-8)
	{
		return 90;
	}

	double angle = acos(angle_cos) * 180. / 3.1415926 ;

	if (angle < 0 || angle > 180)
	{
		cout << "compute angle wrong!!" << endl;
		//system("Pause");
		return -1;
	}

	return angle;
}

double GlobalFun::computeTriangleArea_3(Point3f& v0, Point3f& v1, Point3f& v2)
{
  Point3f AB = v1 - v0;  //vector v0v1
  Point3f AC = v2 - v0;  //vector v0v2
  Point3f AP = AB ^ AC;  
  return AP.Norm() / 2.0f;
}

bool GlobalFun::isPointInTriangle_3(Point3f& v0, Point3f& v1, Point3f& v2, Point3f& p)
{
  double area1 = GlobalFun::computeTriangleArea_3(v0, v1, p);
  double area2 = GlobalFun::computeTriangleArea_3(v0, v2, p);
  double area3 = GlobalFun::computeTriangleArea_3(v1, v2, p);
  double area  = GlobalFun::computeTriangleArea_3(v0, v1, v2);
  if (fabs(area - (area1 + area2 + area3)) < EPI_BOX) return true;
  else return false;
}

double GlobalFun::computeMeshLineIntersectPoint( CMesh *target, Point3f& p, Point3f& line_dir, Point3f& result )
{
  //compute the intersecting point between the ray and the mesh
  int n_face = target->face.size();
  double min_dist = BIG;
  
#ifdef LINKED_WITH_TBB
  tbb::parallel_for(tbb::blocked_range<size_t>(0, n_face), 
    [&](const tbb::blocked_range<size_t>& r)
  {
    for (size_t f = r.begin(); f < r.end(); ++f)
    {
      Point3f& v0 = target->face[f].V(0)->P();
      Point3f& v1 = target->face[f].V(1)->P();
      Point3f& v2 = target->face[f].V(2)->P();
      Point3f e1 = v0 - v1;
      Point3f e2 = v1 - v2;
      Point3f face_norm = (e1 ^ e2).Normalize();
      //if the face can't be seen, then continue
      if(face_norm * line_dir > 0) continue;
      //the line cross the point: pos, and line vector is viewray_iter 
      double t = ( (v0.X() - p.X()) * face_norm.X() 
        + (v0.Y() - p.Y()) * face_norm.Y() 
        + (v0.Z() - p.Z()) * face_norm.Z() ) 
        / ( face_norm.X() * line_dir.X() + face_norm.Y() * line_dir.Y() + face_norm.Z() * line_dir.Z() ) ;

      Point3f intersect_point = p + line_dir * t;

      if(GlobalFun::isPointInTriangle_3(v0, v1, v2, intersect_point)) 
      {
        Point3f d = intersect_point - p;
        double dist_temp = d.SquaredNorm();
        //get the visible point
        if (dist_temp < min_dist)
        {
          min_dist = dist_temp;
          result = intersect_point;
        }
      }else continue;
    }
  });
#else
  for (int f = 0; f < n_face; ++f)
  {
    Point3f& v0 = target->face[f].V(0)->P();
    Point3f& v1 = target->face[f].V(1)->P();
    Point3f& v2 = target->face[f].V(2)->P();
    Point3f e1 = v0 - v1;
    Point3f e2 = v1 - v2;
    Point3f face_norm = (e1 ^ e2).Normalize();
    //if the face can't be seen, then continue
    if(face_norm * line_dir > 0) continue;

    //just choose one point and calculate the distance,if the triangle is too far from the line, then continue
    /*double A = line_dir.Y();
    double B = line_dir.Z() - line_dir.X();
    double C = -line_dir.Y();
    double D = line_dir.Y() * p.Z() + line_dir.X() * p.Y() - line_dir.Y() * p.X() - line_dir.Z() * p.Y();
    double proj_dist = abs(A * v0.X() + B * v0.Y() + C * v0.Z() + D) / sqrt(A * A + B * B + C * C);
    if (proj_dist < )
    {
    }*/

    //the line cross the point: pos, and line vector is viewray_iter 
    double t = ( (v0.X() - p.X()) * face_norm.X() 
      + (v0.Y() - p.Y()) * face_norm.Y() 
      + (v0.Z() - p.Z()) * face_norm.Z() ) 
      / ( face_norm.X() * line_dir.X() + face_norm.Y() * line_dir.Y() + face_norm.Z() * line_dir.Z() ) ;

    Point3f intersect_point = p + line_dir * t;

    if(GlobalFun::isPointInTriangle_3(v0, v1, v2, intersect_point)) 
    {
      Point3f d = intersect_point - p;
      double dist_temp = d.SquaredNorm();
      //get the visible point
      if (dist_temp < min_dist)
      {
        min_dist = dist_temp;
        result = intersect_point;
      }
    }else continue;
  }
#endif
  
  return sqrt(min_dist);
}

bool
GlobalFun::cmp(DesityAndIndex &a, DesityAndIndex &b)
{
  return a.density < b.density;
}

void
GlobalFun::removeOutliers(CMesh *mesh, double radius, double remove_percent)
{
  if (NULL == mesh) 
  { 
    cout<<"Empty Mesh, When RemoveOutliers!"<<endl;
    return;
  }

  double radius2 = radius * radius; 
  double iradius16 = .0f - 4.0f / radius2;
  computeBallNeighbors(mesh, NULL, radius, mesh->bbox);

  vector<DesityAndIndex> mesh_density;
  for (int i = 0; i < mesh->vert.size(); ++i)
  {
    CVertex &v = mesh->vert[i];
    DesityAndIndex dai;
    dai.index = i;
    dai.density = 1.0f;

    vector<int>* neighbors = & v.neighbors;
    for (int j = 0; j < neighbors->size(); ++j)
    {
      CVertex &nei = mesh->vert[(*neighbors)[j]];
      double dist2 = (v.P() - nei.P()).SquaredNorm();
      double den = exp(dist2 * iradius16);

      dai.density += den;
    }
    mesh_density.push_back(dai);
  }

  //sort the density and remove low ones
  sort(mesh_density.begin(), mesh_density.end(), cmp);
  int remove_num = mesh_density.size() * remove_percent;
  //set those who are removed, ignored = false
  for (int i = 0; i < remove_num; ++i)
  {
    mesh->vert[mesh_density[i].index].is_ignore = true;
  }

  //wsh truly remove points
  vector<CVertex> temp_vert;
  for (int i = 0; i < mesh->vert.size(); i++)
  {
    CVertex& v = mesh->vert[i];
    if (!v.is_ignore)
    {
      temp_vert.push_back(v);
    }
  }

  mesh->vert.clear();
  for (int i = 0; i < temp_vert.size(); i++)
  {
    CVertex& v = temp_vert[i];
    v.m_index = i;
    mesh->vert.push_back(v);
  }
  mesh->vn = mesh->vert.size();
}

void
GlobalFun::removeOutliers(CMesh *mesh, double radius, int remove_num)
{
  if (NULL == mesh) 
  { 
    cout<<"Empty Mesh, When RemoveOutliers!"<<endl;
    return;
  }

  double remove_percent = remove_num / mesh->vert.size();

  GlobalFun::removeOutliers(mesh, radius, remove_percent);
}

void
GlobalFun::computeICP(CMesh *dst, CMesh *src)
{
  if (dst->vert.empty() || src->vert.empty())
  {
    cout<< "compute ICP Error : " <<"Empty meshes!" <<endl;
    return;
  }

  int verbose = 0;
  bool do_scale = false;
  bool do_affine = false;
  bool bulkmode = false;

  int c;

  princeton::TriMesh::set_verbose(verbose);

  //******************** for test *****************************
  //const char *filename1 = "1.ply", *filename2 = "2.ply";
  //princeton::TriMesh *raw1 = princeton::TriMesh::read(filename1);
  //princeton::TriMesh *raw2 = princeton::TriMesh::read(filename2);
 
  //CMesh *cm1 = new CMesh;
  //CMesh *cm2 = new CMesh;
  //int index = 0;
  //for (int i = 0; i < raw1->vertices.size(); ++i)
  //{
  //  point &p = raw1->vertices[i];
  //  CVertex t;
  //  t.m_index = index++;
  //  t.P()[0] = p[0];
  //  t.P()[1] = p[1];
  //  t.P()[2] = p[2];
  //  cm1->vert.push_back(t);
  //  cm1->bbox.Add(t.P());
  //}
  //cm1->vn = cm1->vert.size();

  //for (int j = 0; j < raw2->vertices.size(); ++j)
  //{
  //  point &p = raw2->vertices[j];
  //  CVertex t;
  //  t.P()[0] = p[0];
  //  t.P()[1] = p[1];
  //  t.P()[2] = p[2];
  //  cm2->vert.push_back(t);
  //  cm2->bbox.Add(t.P());
  //}
  //cm2->vn = cm2->vert.size();
  //*********************************************************

  princeton::TriMesh *mesh1 = new princeton::TriMesh;
  princeton::TriMesh *mesh2 = new princeton::TriMesh;

  //set two tri-meshes
  for (int i = 0; i < dst->vert.size(); ++i)
  {
    CVertex &v = dst->vert[i];
    point p(v.P()[0], v.P()[1], v.P()[2]);
    mesh1->vertices.push_back(p);
  }

  for (int j = 0; j < src->vert.size(); ++j)
  {
    CVertex &v = src->vert[j];
    point p(v.P()[0], v.P()[1], v.P()[2]);
    mesh2->vertices.push_back(p);
  }

  xform xf1, xf2;
  
  KDtree *kd1 = new KDtree(mesh1->vertices);
  KDtree *kd2 = new KDtree(mesh2->vertices);
  vector<float> weights1, weights2;

  if (bulkmode) {
    float area1 = mesh1->stat(princeton::TriMesh::STAT_TOTAL, princeton::TriMesh::STAT_FACEAREA);
    float area2 = mesh2->stat(princeton::TriMesh::STAT_TOTAL, princeton::TriMesh::STAT_FACEAREA);
    float overlap_area, overlap_dist;
    find_overlap(mesh1, mesh2, xf1, xf2, kd1, kd2,
      overlap_area, overlap_dist);
    float frac_overlap = overlap_area / min(area1, area2);
    if (frac_overlap < 0.1f) {
      printf("Insufficient overlap\n");
      exit(1);
    } else {
      printf("%.1f%% overlap\n",
        frac_overlap * 100.0);
    }
  }

  float err = ICP(mesh1, mesh2, xf1, xf2, kd1, kd2, weights1, weights2,
    verbose, do_scale, do_affine);
  if (err >= 0.0f)
    err = ICP(mesh1, mesh2, xf1, xf2, kd1, kd2, weights1, weights2,
    verbose, do_scale, do_affine);
  
  if (err < 0.0f) {
    printf("ICP failed\n");
    exit(1);
  }

  printf("ICP succeeded - distance = %f\n", err);
  
  //add new points to dst
  int index = (dst->vert.back()).m_index;
  for (int i = 0; i < mesh2->vertices.size(); ++i)
  {
    point p = xf2 * mesh2->vertices[i];
    CVertex t;
    t.m_index = ++index;
    t.P()[0] = p[0];
    t.P()[1] = p[1];
    t.P()[2] = p[2];
    dst->vert.push_back(t);
    dst->bbox.Add(t.P());
  }
  dst->vn = dst->vert.size();
}

void
GlobalFun::downSample(CMesh *dst, CMesh *src, double sample_ratio, bool use_random_downsample)
{
  if (NULL == src || src->vert.size() <= 0)
  {
    cout<<"Down sample Error: Empty src points!" <<endl;
    return;
  }

  int want_sample_num = static_cast<int>(src->vert.size() * sample_ratio);
  dst->vn = want_sample_num;

  vector<int> nCard = GlobalFun::GetRandomCards(src->vert.size());
  for ( int i = 0; i < dst->vn; ++i)
  {
    int index = nCard[i];
    if (!use_random_downsample)
    {
      index = i;
    }

    CVertex &v = src->vert[index];
    dst->vert.push_back(v);
    dst->bbox.Add(v.P());
  }

  CMesh::VertexIterator vi;
  for(vi = dst->vert.begin(); vi != dst->vert.end(); ++vi)
    vi->is_original = false;

}

void 
GlobalFun::clearCMesh(CMesh &mesh)
{
  mesh.face.clear();
  mesh.fn = 0;
  mesh.vert.clear();
  mesh.vn = 0;
  mesh.bbox = Box3f();
}

void GlobalFun::deleteIgnore(CMesh* mesh)
{
  vector<CVertex> temp_vert;
  for (int i = 0; i < mesh->vert.size(); i++)
  {
    CVertex& v = mesh->vert[i];
    if (!v.is_ignore)
    {
      temp_vert.push_back(v);
    }
  }

  mesh->vert.clear();
  for (int i = 0; i < temp_vert.size(); i++)
  {
    CVertex& v = temp_vert[i];
    v.m_index = i;
    mesh->vert.push_back(v);
  }
  mesh->vn = mesh->vert.size();
}

void GlobalFun::recoverIgnore(CMesh* mesh)
{
  for (int i = 0; i < mesh->vert.size(); i++)
  {
    CVertex& v = mesh->vert[i];
    v.is_ignore = false;
  }
}

void GlobalFun::cutPointSelfSlice(CMesh* mesh, Point3f anchor, Point3f direction, double width)
{
  //mesh->vert.clear();
  cout << "cut anchor point: " << anchor[0] << ", " << anchor[1] << ", " << anchor[2] << endl;
  cout << "direction: " << direction[0] << ", " << direction[1] << ", " << direction[2] << endl;

  double width2 = width * width;
  for (int i = 0; i < mesh->vert.size(); i++)
  {
    CVertex& v = mesh->vert[i];
    //double perpend_dist2 = GlobalFun::computePerpendicularDistSquare(anchor, v.P(), direction);
    double proj_dist = GlobalFun::computeProjDist(anchor, v.P(), direction);
    double proj_dist2 = proj_dist * proj_dist;

    //if (i < 100)
    //{
    //   cout << perpend_dist2 << " " << width2 << endl;
    //}

    if (proj_dist2 > width2)
    {
      v.is_ignore = true;
      continue;
    }
    
    //cout << "Not Ignore!!!!" << endl;

    Point3f new_p = v.P() - direction * proj_dist;
    v.P() = new_p;
  }

}

//void Slice::build_slice(Point3f a, Point3f b, Point3f c, float c_length)
//{
//  cell_length = c_length;
//
//  Point3f origin = a;
//  Point3f row_axis = b-a;
//  Point3f col_axis = c-a;
//
//  float row_length = sqrt(row_axis.SquaredNorm());
//  float col_length = sqrt(col_axis.SquaredNorm());
//
//  int row_num = int(row_length / c_length) + 2;
//  int col_num = int(col_length / c_length) + 2;
//
//  row_axis.Normalize();
//  col_axis.Normalize();
//
//  slice_nodes.clear();
//
//  for (int i = 0; i < row_num; i++)
//  {
//    for (int j = 0; j < col_num; j++)
//    {
//      CVertex new_v;
//      new_v.P() = origin + row_axis * (c_length * i) + col_axis * (c_length * j);
//      slice_nodes.push_back(new_v);
//    }
//  }
//}
