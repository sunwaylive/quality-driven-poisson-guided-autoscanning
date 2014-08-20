#pragma once
#include <iostream>
#include <tbb/parallel_for.h>
#include <tbb/concurrent_vector.h>
#include <tbb/queuing_mutex.h>
#include "PointCloudAlgorithm.h"
#include "GlobalFunction.h"

using std::cout;
using std::endl;
using vcg::Point3f;

class NBV : public PointCloudAlgorithm
{
public:
  NBV(RichParameterSet* _para);
  ~NBV();

  void run();
  void setInput(DataMgr *pData);
  void setParameterSet(RichParameterSet *_para) { para = _para;}
  RichParameterSet * getParameterSet() { return para;}
  void clear();

private:
  void runOneKeyNBV();
  void buildGrid();
  void propagate();
  void viewExtraction();
  void viewExtractionIntoBins(int view_bin_each_axis);
  void viewPrune();
  void extractViewIntoBinsUsingDist();
  void viewClustering();
  void setIsoBottomConfidence();
  bool updateViewDirections();
  void runSmoothGridConfidence();
  void runComputeViewCandidateIndex();

  int    round(double x);
  void   setGridUnHit(vector<int>& hit_grids_idx);
  double computeLocalScores(CVertex& view_t, CVertex& iso_v, 
  double& optimal_D, double& half_D2, double& sigma_threshold);
  int    getIsoPointsViewBinIndex(Point3f& p, int which_axis);
  static bool cmp(const CVertex &v1, const CVertex &v2);

private:
  int                   view_bin_each_axis;
  RichParameterSet      *para;
  CMesh                 *model;
  CMesh                 *original;
  CMesh                 *iso_points;
  CMesh                 *view_grid_points;
  CMesh                 *nbv_candidates;
  vector<ScanCandidate> *scan_candidates;
  vector<ScanCandidate> *seletedViewCameras;
  CMesh                 *field_points;
  double                grid_step_size;
  Point3f               whole_space_box_max;
  Point3f               whole_space_box_min;
  int                   x_max; //max index num of x-axis 
  int                   y_max;
  int                   z_max;
  vector<float>         confidence_weight_sum;
  vector<double>        nbv_scores;
  Box3f*                whole_space_box;
};