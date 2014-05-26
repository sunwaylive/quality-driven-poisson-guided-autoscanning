#ifndef PVS_BASED_NBV_H
#define PVS_BASED_NBV_H

#include <iostream>
#include <math.h>
#include "PointCloudAlgorithm.h"
#include "GlobalFunction.h"
#include "vcg\complex\algorithms\update\topology.h"
#include "vcg\complex\complex.h"

#include <QtGui>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>

#include "RIMLS/mlsmarchingcube.h"
#include "RIMLS/rimls.h"
#include "RIMLS/apss.h"
#include "RIMLS/mlsutils.h"
#include "RIMLS/implicits.h"
#include "RIMLS/smallcomponentselection.h"
#include "Algorithm/RIMLS/balltree.h"
using namespace GaelMls;


typedef std::vector<MyBoarderEdge>::iterator MyEdgeIter;
typedef vcg::tri::UpdateFlags<CMesh>::FaceIterator FaceIter;

class PVSBasedNBV : public PointCloudAlgorithm
{
public:
  PVSBasedNBV(RichParameterSet* _para);
  ~PVSBasedNBV();

  void run();
  void setInput(DataMgr *pData);
  void setParameterSet(RichParameterSet* _para) {para = _para;}
  RichParameterSet* getParameterSet() {return para;}
  void clear();

public:
  void findBoarderPoints();
  void findBoarderPointsByBallpivoting();
  void searchNewBoundaries();
  Boundary searchOneBoundaryFromIndex(int begin_idx);
  Boundary searchOneBoundaryFromDirection(int begin_idx, Point3f direction);

  //IEEE sphere
  void buildSphereCandidatesIEEE();
  void computeScoreIEEE();
  double areaFactorIEEE(double percentage, double optimum_percentage);
  //distance should be normalized to 0~1
  double navigationFactorIEEE(double distance, double rho = 0.2);
  void updateIEEE();
  void selectCandidateIEEE();

private:
  void runSearchNewBoundaries();
  void runBuildPVS();
  void runUpdatePVS();
  void runSearchNewBoundariesByBallpivoting();
  void runComputeCandidates();
  void runSelectCandidate();
  void runPVSMerge();
  void runSphere();

  static bool cmp(const CVertex &v1, const CVertex &v2);
  
private:
  RichParameterSet      *para;
  double                optimalDist;
  CMesh                 *model;
  CMesh                 *sample;
  CMesh                 *original;
  CMesh                 *iso_points;
  CMesh                 *pvs;
  CMesh                 *detect_result;
  vector<Boundary>      *m_v_boundaries;
  CMesh                 *coarse_candidates;
  CMesh                 *nbv_candidates;
  vector<ScanCandidate> *scan_candidates;
  vector<ScanCandidate> *scan_history;
  vector<CMesh *>*      scanned_results;
  int                   *scan_count;
  Point3f               whole_space_box_min;
  Point3f               whole_space_box_max;
  int                   x_max;
  int                   y_max;
  int                   z_max;
};

#endif