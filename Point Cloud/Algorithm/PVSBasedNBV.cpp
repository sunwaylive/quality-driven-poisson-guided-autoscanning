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

  //calculate the topology of sample points
  GlobalFun::ballPivotingReconstruction(*sample);
  vcg::tri::UpdateTopology<CMesh>::VertexFace(*sample);

  //use vertex topology
  vcg::tri::UpdateFlags<CMesh>::VertexBorderFromNone(*sample); 
  std::cout<<"vertex line: " <<tri::UpdateSelection<CMesh>::VertexFromBorderFlag(*sample) << std::endl;
  for (int i = 0; (i < sample->vert.size()); ++i)
  {
    if(sample->vert[i].IsS())
      std::cout <<" point on border: "<< sample->vert[i].IsB() <<std::endl;
  }

  //using face topology
  /*vcg::tri::UpdateFlags<CMesh>::FaceBorderFromNone(*sample);
  std::cout<<"face line: " <<tri::UpdateSelection<CMesh>::FaceFromBorderFlag(*sample) <<std::endl;
  for (int i = 0; i < sample->face.size(); ++i)
  {
    if (sample->face[i].IsS())
    {
      std::cout<< "face is selected: " <<sample->face[i].IsS() <<std::endl;
      for (int j = 0; j < 3 ; ++j)
      {
        std::cout <<j <<"th point of face " << i << " on border: " << sample->face[i].IsB(j) <<std::endl;
      }
    }
  }*/
  
  //vcg::tri::UpdateFlags<CMesh>::FaceBorderFromFF(*sample);
  //
  //vcg::tri::UpdateFlags<CMesh>::VertexBorderFromNone(*sample);
  //vcg::tri::UpdateTopology<CMesh>::VertexFace(*sample);
  //update mesh topology
  /*vcg::tri::UpdateTopology<CMesh>::FaceFace(*sample);
  vcg::tri::UpdateTopology<CMesh>::VertexFace(*sample);
  vcg::tri::UpdateFlags<CMesh>::FaceBorderFromFF(*sample);
  vcg::tri::UpdateFlags<CMesh>::VertexBorderFromFace(*sample);*/
  //
  //std::cout<<"vertex line: " <<tri::UpdateSelection<CMesh>::VertexFromBorderFlag(*sample) << std::endl;    
}