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
  if (!pData->getCurrentOriginal()->vert.empty())
  {
    original = pData->getCurrentOriginal();
  }else
  {
    std::cout<<"ERROR: VisibilitBasedNBV::setInput empty original points"<<std::endl;
  }

  optimalDist = global_paraMgr.camera.getDouble("Camera Dist To Model")
    / global_paraMgr.camera.getDouble("Predicted Model Size");
  nbv_candidates = pData->getNbvCandidates();
}

void VisibilityBasedNBV::run()
{
  if (para->getBool("Run Visibility Propagate"))
  {
    runVisibilityPropagate();
    return;
  }
}

void VisibilityBasedNBV::clear()
{

}

void VisibilityBasedNBV::runVisibilityPropagate()
{
  //do a normalization for original points


  int index = 0;
  for (int i = 0; i < original->vert.size(); ++i)
  {
    CVertex &v = original->vert[i];
    if (v.is_barely_visible) 
    {
      CVertex p;
      p.P() = v.P() + v.N() * optimalDist;
      p.N() = -v.N();
      p.m_index = index++;
      nbv_candidates->vert.push_back(p);
    }
  }
  nbv_candidates->vn = nbv_candidates->vert.size();
  std::cout<<"candidate number:"<< nbv_candidates->vn <<std::endl;
}

void VisibilityBasedNBV::runNBVCandidatesCluster()
{
  double radius = para->getDouble("CGrid Radius"); 
  double radius2 = radius * radius;
  double iradius16 = -4/radius2;

  double sigma = 45;
  double cos_sigma = cos(sigma / 180.0 * 3.1415926);
  double sharpness_bandwidth = std::pow((std::max)(1e-8, 1 - cos_sigma), 2);

  GlobalFun::computeAnnNeigbhors(nbv_candidates->vert,
    nbv_candidates->vert, 
    15,
    false,
    "runViewCandidatesClustering");

  vector<CVertex> update_temp;
  for(int i = 0; i < nbv_candidates->vert.size(); i++)
  {
    CVertex& v = nbv_candidates->vert[i];

    //if (v.neighbors.size() <= 5)
    if (v.neighbors.empty())
    {
      //update_temp.push_back(v);
      continue;
    }

    Point3f average_positon = Point3f(0, 0, 0);
    Point3f average_normal = Point3f(0, 0, 0);
    double sum_weight = 0.0;

    for (int j = 0; j < v.neighbors.size(); j++)
    {
      CVertex& t = nbv_candidates->vert[v.neighbors[j]];

      Point3f diff = v.P() - t.P();
      double dist2  = diff.SquaredNorm();

      double dist_weight = exp(dist2 * iradius16);
      double normal_weight = exp(-std::pow(1 - v.N() * t.N(), 2));
      double weight = dist_weight;

      average_positon += t.P() * weight;
      average_normal += t.N() * weight;
      sum_weight += weight;
    }

    CVertex temp_v = v;
    temp_v.P() = average_positon / sum_weight;
    temp_v.N() = average_normal / sum_weight;
    update_temp.push_back(temp_v);
  }

  nbv_candidates->vert.clear();
  for (int i = 0; i < update_temp.size(); i++)
  {
    nbv_candidates->vert.push_back(update_temp[i]);
  }
  nbv_candidates->vn = nbv_candidates->vert.size(); 
}