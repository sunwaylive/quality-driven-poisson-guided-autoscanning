#include "VisibilityBasedNBV.h"

VisibilityBasedNBV::VisibilityBasedNBV(RichParameterSet* _para)
{
  std::cout<<"VisibilityBasedNBV constructed!"<<std::endl;
  para = _para;
}

VisibilityBasedNBV::~VisibilityBasedNBV()
{

}

void VisibilityBasedNBV::setInput(DataMgr *pData)
{
  optimalDist = global_paraMgr.camera.getDouble("Camera Dist To Model");

}

void VisibilityBasedNBV::run()
{

}

void VisibilityBasedNBV::clear()
{

}