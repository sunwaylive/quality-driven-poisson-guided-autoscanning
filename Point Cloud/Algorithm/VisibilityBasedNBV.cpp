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
    original = pData->getCurrentOriginal();
  else
    std::cout<<"ERROR: VisibilitBasedNBV::setInput empty original points"<<std::endl;

  model = pData->getCurrentModel();
  scanned_results = pData->getScannedResults();

  optimalDist = (global_paraMgr.camera.getDouble("Camera Far Distance") +global_paraMgr.camera.getDouble("Camera Near Distance")) 
    / 2 / global_paraMgr.camera.getDouble("Predicted Model Size");
  nbv_candidates = pData->getNbvCandidates();
  scan_candidates = pData->getScanCandidates();
  scan_history = pData->getScanHistory();
}

void VisibilityBasedNBV::run()
{
  if (para->getBool("Run Visibility Propagate"))
  {
    std::cout<<"Run Visibility Propagate" <<std::endl;
    runVisibilityPropagate();
    return;
  }
  if (para->getBool("Run Visibility Candidates Cluster"))
  {
    std::cout<<"Run Visibility Candidates Cluster" <<std::endl;
    runVisibilityCandidatesCluster();
    return;
  }
  if (para->getBool("Run Visibility Update"))
  {
    std::cout<<"Run VIsibility Update" <<std::endl;
    runVisibilityUpdate();
    return;
  }
  if (para->getBool("Run Visibility Merge"))
  {
    std::cout<<"Run Visibility Merge" <<std::endl;
    runVisibilityMerge();
    return;
  }
}

void VisibilityBasedNBV::clear()
{

}

void VisibilityBasedNBV::runVisibilityPropagate()
{
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

void VisibilityBasedNBV::runVisibilityCandidatesCluster()
{
  double radius = 0.5; //global_paraMgr.data.getDouble("CGrid Radius");
  double nbv_minimum_dist = radius;
  double radius2 = radius * radius;
  double iradius16 = -4/radius2;

  double sigma = 45;
  double cos_sigma = cos(sigma / 180.0 * 3.1415926);
  double sharpness_bandwidth = std::pow((std::max)(1e-8, 1 - cos_sigma), 2);

  double shift_dist_stop = 0.001;
  double current_shift = radius;

  do
  {
    GlobalFun::computeBallNeighbors(nbv_candidates, NULL, radius, nbv_candidates->bbox);

    vector<CVertex> update_temp;

    for(int i = 0; i < nbv_candidates->vert.size(); i++)
    {
      CVertex& v = nbv_candidates->vert[i];

      if (v.neighbors.empty())
        continue;

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

      double shift_dist = std::sqrt(static_cast<float>((temp_v.P() - v.P()).SquaredNorm()));
      current_shift = current_shift < shift_dist ? current_shift : shift_dist;
    }

    if (!update_temp.empty())
    {
      nbv_candidates->vert.clear();
      for (int k = 0; k < update_temp.size(); k++)
        nbv_candidates->vert.push_back(update_temp[k]);

      nbv_candidates->vn = nbv_candidates->vert.size(); 

      update_temp.clear();
    }
  }while(current_shift >= shift_dist_stop);

  //get selected NBV
  double nbv_select_radius = radius; 
  GlobalFun::computeBallNeighbors(nbv_candidates, NULL, nbv_select_radius, nbv_candidates->bbox);
  int max_neighbors_num = nbv_candidates->vert[0].neighbors.size();
  int vert_index = 0;
  for (int i=0; i < nbv_candidates->vert.size(); ++i)
  {
    if (nbv_candidates->vert[i].neighbors.size() > max_neighbors_num)
    {
      vert_index = i;
      max_neighbors_num = nbv_candidates->vert[i].neighbors.size();
      std::cout<< i <<std::endl;
    }
  }

  CVertex v = nbv_candidates->vert[vert_index];
  nbv_candidates->vert.clear();
  nbv_candidates->vert.push_back(v);

  //check the minimum distance and store them in scan_candidates
  scan_candidates->clear();
  for (int i = 0; i < nbv_candidates->vert.size(); ++i)
  {
    CVertex &c = nbv_candidates->vert[i];
    bool is_qualified = true;

    if (scan_history->empty())
    {
      ScanCandidate s =  make_pair(nbv_candidates->vert[i].P(), nbv_candidates->vert[i].N());
      scan_candidates->push_back(s);
      continue;
    }
    
    for (int j = 0; j < scan_history->size() && is_qualified; ++j)
      if (GlobalFun::computeEulerDist(c.P(), scan_history->at(j).first) < nbv_minimum_dist)
        is_qualified = false;

    if (is_qualified)
    {
      ScanCandidate s =  make_pair(nbv_candidates->vert[i].P(), nbv_candidates->vert[i].N());
      scan_candidates->push_back(s);
    }
  }

  std::cout<<"scan candidate size:" << scan_candidates->size() <<std::endl;
  //if no scan_candidate is found, then quit
  /* if (scan_candidates->empty())
  {
  QMessageBox msgBox;
  msgBox.setText("algorithm has completed!");
  msgBox.exec();
  }*/
}

void VisibilityBasedNBV::runVisibilityMerge()
{
  for (int i = 0; i < scanned_results->size(); ++i)
  {
    CMesh *sr = scanned_results->at(i);
    int index = original->vert.back().m_index;

    for (int j = 0; j < sr->vert.size(); ++j)
    {
      CVertex &v = sr->vert[j];
      v.m_index = ++index;
      v.is_scanned = false;
      v.is_original = true;
      original->vert.push_back(v);
    }
  }
  original->vn = original->vert.size();
}

void VisibilityBasedNBV::runVisibilityUpdate()
{
  double camera_fov = global_paraMgr.camera.getDouble("Camera FOV Angle");
  double camera_far_dist = global_paraMgr.camera.getDouble("Camera Far Distance");
  double camera_near_dist = global_paraMgr.camera.getDouble("Camera Near Distance");
  //CMesh *target_mesh = original;
  CMesh *target_mesh = model;
  //first we should reconstruct the target surface
  //GlobalFuns:ballPivotingReconstruction(*target_mesh);

  for (int i = 0; i < original->vert.size(); ++i)
  {
    CVertex &v = original->vert[i];

    for (int j = 0; j < scan_history->size() && v.is_barely_visible; ++j)
    {
      //two issues:
      //for each ray, we should compute the ray_dir and check whether it's inside the camera's FOV
      Point3f pos = scan_history->at(j).first;
      Point3f camera_dir = scan_history->at(j).second;
      Point3f ray_dir = v.P() - pos;
      
      double angle = GlobalFun::computeRealAngleOfTwoVertor(ray_dir, camera_dir);
      double d = GlobalFun::computeEulerDist(pos, v.P());

      //if (angle > camera_fov || d > camera_far_dist || d < camera_near_dist) //out of camera fov
      //  continue;

      bool is_wv = GlobalFun::isPointWellVisible(v, pos, ray_dir, target_mesh);
      if (is_wv)
      {
         std::cout<<"well see!: "<< is_wv <<std::endl;
      }
      v.is_barely_visible = (v.is_barely_visible && !is_wv);  //make sure all view point can't well-see v.
    }
  }
}