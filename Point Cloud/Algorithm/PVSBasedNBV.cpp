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

  if (!pData->getCurrentSamples()->vert.empty())
    sample = pData->getCurrentSamples();
  else
    std::cout << "ERROR: PVSBasedNBV::setInput empty sample points" <<std::endl;

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
  std::cout << "detect boundary" <<std::endl;
}