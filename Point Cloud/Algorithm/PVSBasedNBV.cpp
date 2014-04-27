#include "PVSBasedNBV.h"

PVSBasedNBV::PVSBasedNBV(RichParameterSet* _para)
{
  std::cout<< "PVSBasedNBV constructed!" <<std::endl;
  para = _para;
}

PVSBasedNBV::~PVSBasedNBV()
{

}

void PVSBasedNBV::setInput(DataMgr *pData)
{
  if (!pData->getCurrentOriginal()->vert.empty())
    original = pData->getCurrentOriginal();
  else
    std::cout << "ERROR: PVSBasedNBV::setInput empty original points" <<std::endl;

  sample = pData->getCurrentSamples();

  scanned_results = pData->getScannedResults();
  optimalDist = (global_paraMgr.camera.getDouble("Camera Far Distance") +global_paraMgr.camera.getDouble("Camera Near Distance")) 
    / 2 / global_paraMgr.camera.getDouble("Predicted Model Size");
  nbv_candidates = pData->getNbvCandidates();
  scan_candidates = pData->getScanCandidates();
}

void PVSBasedNBV::run()
{
  if (para->getBool("Run PVS Detect Boundary"))
  {
    std::cout << "Run PVS Detect Boundary" <<std::endl;
    runPVSDetectBoundary();
  }
}

void PVSBasedNBV::clear()
{

}

void PVSBasedNBV::runPVSDetectBoundary()
{
  //copy point from original to sample
  GlobalFun::clearCMesh(*sample);  

  for (int i = 0; i < original->vert.size(); ++i)
  {
    CVertex v = original->vert[i];
    v.is_original = false;
    v.is_fixed_sample = true;
    sample->vert.push_back(v);
  }
  sample->vn = sample->vert.size();

  //detect boundary
}