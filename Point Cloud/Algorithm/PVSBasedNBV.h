#ifndef PVS_BASED_NBV_H
#define PVS_BASED_NBV_H

#include <iostream>
#include <math.h>
#include "PointCloudAlgorithm.h"
#include "GlobalFunction.h"
#include "vcg\complex\trimesh\update\selection.h"
#include "vcg\complex\algorithms\update\topology.h"
#include "vcg\complex\complex.h"
#include "vcg/complex/algorithms/create/platonic.h" //for mesh copy

class PVSBasedNBV : public PointCloudAlgorithm
{
public:
  //tri::UpdateSelection<CMeshO>::VertexFromBorderFlag(m.cm);
  void detectBoundary();
  PVSBasedNBV(RichParameterSet* _para);
  ~PVSBasedNBV();

  void run();
  void setInput(DataMgr *pData);
  void setParameterSet(RichParameterSet* _para) {para = _para;}
  RichParameterSet* getParameterSet() {return para;}
  void clear();

private:
  void runPVSDetectBoundary();

private:
  RichParameterSet      *para;
  double                optimalDist;
  CMesh                 *sample;
  CMesh                 *original;
  CMesh                 *nbv_candidates;
  vector<ScanCandidate> *scan_candidates;
  vector<CMesh *>*      scanned_results;
};

#endif