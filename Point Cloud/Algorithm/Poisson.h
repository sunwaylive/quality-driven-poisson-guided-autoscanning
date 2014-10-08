#pragma once
#include "GlobalFunction.h"
#include <algorithm>
#include <iostream>
#include "PointCloudAlgorithm.h"
#include <fstream>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>

using namespace vcg;
using namespace std;

class Poisson : public PointCloudAlgorithm
{
public:
	Poisson(RichParameterSet* _para);
	~Poisson(void);

	void setInput(DataMgr* pData);
	void setParameterSet(RichParameterSet* _para){para = _para;}
	RichParameterSet* getParameterSet(){ return para; }
	void run();
	void clear(){samples = NULL; original = NULL; iso_points = NULL;}

	
protected:
	Poisson(){}

private:
	void input(CMesh* _mesh);
  void runOneKeyPoissonConfidence();

  void runPoisson();
  void runPoissonFieldAndExtractIsoPoints();
  void runPoissonFieldAndExtractIsoPoints_ByEXE();
  void runLabelISO();
  void runIsoSmooth();
  void runLabelBoundaryPoints();
  void runComputeViewCandidates();
  void runViewCandidatesClustering();
  void runSlice();
  void runComputeOriginalConfidence();
  void runComputeSampleConfidence();
  void runComputeIsoGradientConfidence();
  void runComputeIsoSmoothnessConfidence();
  void runComputeIsoHoleConfidence();
  void runSmoothGridConfidence();

  void runSlicePoints();

  void runBallPivotingReconstruction();

  void samplePointsFromMesh(CMesh& mesh, CMesh* points);

private:
	CMesh* samples;
	CMesh* original;
  CMesh* iso_points;
  CMesh* view_candidates;
  CMesh* field_points;
  CMesh* model;
  Slices* slices;
  CMesh tentative_mesh;
  
	RichParameterSet* para;
	Box3f m_box;
};
