#ifndef PVS_BASED_NBV_H
#define PVS_BASED_NBV_H

#include <iostream>
#include <math.h>
#include "PointCloudAlgorithm.h"
#include "GlobalFunction.h"
#include "vcg\complex\algorithms\update\topology.h"
#include "vcg\complex\complex.h"
#include "vcg/complex/algorithms/create/platonic.h" //for mesh copy

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
  void detectBoundary();

  void searchNewBoundaries();
  Boundary searchOneBoundaryFromIndex(int begin_idx);
  Boundary searchOneBoundaryFromDirection(int begin_idx, Point3f direction);

private:
  void runPVSDetectBoundary();
  void runSearchNewBoundaries();
  void runComputeCandidates();
  void runSelectCandidate();

  std::vector<Boundary> getBoundary(std::vector<MyBoarderEdge> &v_board_edge);

private:
  RichParameterSet      *para;
  double                optimalDist;
  CMesh                 *sample;
  CMesh                 *original;
  CMesh                 *nbv_candidates;
  CMesh                 *iso_points;
  vector<Boundary>      *m_v_boundaries;
  vector<ScanCandidate> *scan_candidates;
  vector<CMesh *>*      scanned_results;
};

#endif