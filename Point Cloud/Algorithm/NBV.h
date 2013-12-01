#pragma once
#include <iostream>
#include <tbb/parallel_for.h>
#include "PointCloudAlgorithm.h"
#include "GlobalFunction.h"
#include "NBVGrid.h"

using std::cout;
using std::endl;
using vcg::Point3f;

#define  LINKED_WITH_TBB

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
  void buildGrid();
  void propagate();

  void runOneKeyNBV();

  void viewExtraction();
  void viewExtractionIntoBins();
  void viewClustering();
  void updateViewDirections();

  void normalizeConfidence(vector<CVertex>& vertexes, float delta);
  double   getAbsMax(double x, double y, double z);
  int      round(double x);
  quadrant getQuadrantIdx(double a, double b); //two parameters deciding the quadrant
  void     setGridUnHit(vector<int>& hit_grids_idx);
  vector<float> confidence_weight_sum;

  double computeLocalScores(CVertex& view_t, CVertex& iso_v, 
                           double& optimal_D, double& half_D2, double& sigma_threshold);


private:
  RichParameterSet *para;
  CMesh            *model;
  CMesh            *original;
  CMesh            *iso_points;
  CMesh            *all_nbv_grid_centers;
  CMesh            *nbv_candidates;
  CMesh            *field_points;
  double           grid_resolution;
  Point3f          whole_space_box_max;
  Point3f          whole_space_box_min;
  int              x_max; //max index num of x-axis 
  int              y_max;
  int              z_max;
  std::vector<NBVGrid>  *all_nbv_grids; //grids in all the space

  static int       view_bins_each_axis;
};