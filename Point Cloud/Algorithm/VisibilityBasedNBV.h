#ifndef VISIBILITY_BASED_NBV_H
#define VISIBILITY_BASED_NBV_H

#pragma once
#include <iostream>
#include <math.h>
#include "PointCloudAlgorithm.h"
#include "GlobalFunction.h"

class VisibilityBasedNBV : public PointCloudAlgorithm
{
public:
  VisibilityBasedNBV(RichParameterSet* _para);
  ~VisibilityBasedNBV();

  void run();
  void setInput(DataMgr *pData);
  void setParameterSet(RichParameterSet* _para) {para = _para;}
  RichParameterSet* getParameterSet() {return para;}
  void clear();

private:
  void runVisibilityPropagate();
  void runVisibilityCandidatesCluster();

private:
  RichParameterSet      *para;
  double                optimalDist;
  CMesh                 *original;
  CMesh                 *nbv_candidates;
  vector<ScanCandidate> *scan_history;
  vector<ScanCandidate> *scan_candidates;
};
#endif