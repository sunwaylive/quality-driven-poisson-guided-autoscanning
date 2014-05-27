#include "PVSBasedNBV.h"

//#include <common/interfaces.h>
#include <vcg/complex/trimesh/clean.h>
#include <vcg/complex/trimesh/refine.h>
#include <vcg/complex/trimesh/refine_loop.h>
#include <vcg/complex/trimesh/append.h>
#include <vcg/complex/trimesh/create/advancing_front.h>
#include <vcg/complex/trimesh/create/marching_cubes.h>



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

  model = pData->getCurrentModel();
  sample = pData->getCurrentSamples();
  detect_result = pData->getRIMLS();

  scanned_results = pData->getScannedResults();
  optimalDist = (global_paraMgr.camera.getDouble("Camera Far Distance") +global_paraMgr.camera.getDouble("Camera Near Distance")) 
    / 2 / global_paraMgr.camera.getDouble("Predicted Model Size");
  nbv_candidates = pData->getNbvCandidates();
  scan_candidates = pData->getScanCandidates();
  scan_history = pData->getScanHistory();
  scan_count = pData->getScanCount();
  m_v_boundaries = pData->getBoundaries();
  iso_points = pData->getCurrentIsoPoints();
  pvs = pData->getPVS();
}

void PVSBasedNBV::run()
{
  if (para->getBool("Run PVS Search New Boundaries"))
  {
    std::cout<< "Run PVS Search New Boundaries!" <<std::endl;
    runSearchNewBoundaries();
  }
  if (para->getBool("Run PVS Search New Boundaries By Ballpivoting"))
  {
    std::cout<<" Run PVS Search New Boundaries By Ballpivoting "<<std::endl;
    runSearchNewBoundariesByBallpivoting();
  }
  if (para->getBool("Run PVS Compute Candidates"))
  {
    std::cout<<"Run PVS Compute Candidates " <<std::endl;
    runComputeCandidates();
  }
  if (para->getBool("Run PVS Select Candidate"))
  {
    std::cout<<"Run PVS Select Candidate"<< std::endl;
    runSelectCandidate();
  }
  if (para->getBool("Run Build PVS"))
  {
    std::cout<<"Run Build PVS" <<std::endl;
    runBuildPVS();
  }
  if (para->getBool("Run Update PVS"))
  {
    std::cout<<"Run Update PVS" <<std::endl;
    runUpdatePVS();
  }
  if (para->getBool("Run PVS Merge"))
  {
    std::cout<<"Run PVS Merge" <<std::endl;
    runPVSMerge();
  }
  if (para->getBool("Run Sphere"))
  {
    std::cout<<"Run Sphere" <<std::endl;
    runSphere();
  }
}

void PVSBasedNBV::clear()
{

}

void PVSBasedNBV::runBuildPVS()
{
  if (model->vert.empty())
  {
    std::cout<< " Model Empty!" <<std::endl;
    return;
  }

  GlobalFun::clearCMesh(*pvs);

  /*Point3f bbox_max = model->bbox.max;
  Point3f bbox_min = model->bbox.min;*/

  //TODO: for real gundum
  Point3f bbox_max = original->bbox.max;
  Point3f bbox_min = original->bbox.min;

  //get the whole 3D space that a camera may exist
  double camera_max_dist = global_paraMgr.camera.getDouble("Camera Far Distance") /
    global_paraMgr.camera.getDouble("Predicted Model Size");
  double pvs_resolution = global_paraMgr.pvsBasedNBV.getDouble("PVS Grid Resolution");
  assert(pvs_resolution > 2);
  double grid_step_size = (camera_max_dist*2.0 + 1.0) / (pvs_resolution - 1);

  float extend_pvs_box_size = camera_max_dist;
  whole_space_box_min = bbox_min - Point3f(extend_pvs_box_size, extend_pvs_box_size, extend_pvs_box_size);
  whole_space_box_max = bbox_max + Point3f(extend_pvs_box_size, extend_pvs_box_size, extend_pvs_box_size);
  Box3f whole_space_box;
  whole_space_box.SetNull();
  whole_space_box.Add(whole_space_box_min);
  whole_space_box.Add(whole_space_box_max);
  //compute the size of the 3D space
  Point3f dif = whole_space_box_max - whole_space_box_min;
  //change the whole space box into a cube
  double max_length = std::max(dif.X(), std::max(dif.Y(), dif.Z()));
  whole_space_box_max = whole_space_box_min + Point3f(max_length, max_length, max_length);
  dif = whole_space_box_max - whole_space_box_min;
  //divide the box into grid
  x_max = static_cast<int> (dif.X() / grid_step_size);
  y_max = static_cast<int> (dif.Y() / grid_step_size);
  z_max = static_cast<int> (dif.Z() / grid_step_size);

  int all_max = std::max(std::max(x_max, y_max), z_max);
  x_max = y_max =z_max = all_max+1; // wsh 12-11
  //preallocate the memory
  int max_index = x_max * y_max * z_max;
  pvs->vert.resize(max_index + 1);
  //increase from whole_space_box_min
  for (int i = 0; i < x_max; ++i)
  {
    for (int j = 0; j < y_max; ++j)
    {
      for (int k = 0; k < z_max; ++k)
      {
        //add the grid
        int index = i * y_max * z_max + j * z_max + k;
        //add the center point of the grid
        CVertex t;
        t.P()[0] = whole_space_box_min.X() + i * grid_step_size;
        t.P()[1] = whole_space_box_min.Y() + j * grid_step_size;
        t.P()[2] = whole_space_box_min.Z() + k * grid_step_size;

        pvs->vert[index].P() = t.P();
        pvs->vert[index].is_pvs = true;
        pvs->vert[index].m_index = index;
        pvs->bbox.Add(t.P());
      }
    }
  }
  pvs->vn = pvs->vert.size();
  cout << "all grid points: " << pvs->vn << endl;
  cout << "resolution: " << x_max << endl;
}

void PVSBasedNBV::runUpdatePVS()
{
  double resolution = global_paraMgr.camera.getDouble("Camera Resolution");
  double far_horizon_dist = global_paraMgr.camera.getDouble("Camera Horizon Dist") 
    / global_paraMgr.camera.getDouble("Predicted Model Size");
  double far_vertical_dist = global_paraMgr.camera.getDouble("Camera Vertical Dist")
    / global_paraMgr.camera.getDouble("Predicted Model Size");
  double camera_max_dist = global_paraMgr.camera.getDouble("Camera Far Distance") /
    global_paraMgr.camera.getDouble("Predicted Model Size");
  int grid_resolution = global_paraMgr.pvsBasedNBV.getDouble("PVS Grid Resolution");
  if (grid_resolution <= 2)
    return;
  double grid_step_size = (camera_max_dist*2.0 + 1.0) / (grid_resolution - 1);
  int max_steps = static_cast<int>(camera_max_dist / grid_step_size);
  int pvs_size = pvs->vert.size();

  //STEP1: From angle of acquired points: for new scanned mesh, we should update the occupied pvs grid value
  //compute and normalize density of the each pvs grid
  GlobalFun::computeBallNeighbors(pvs, detect_result, grid_step_size, detect_result->bbox);
  double density_max = -1;
  double density_min = BIG;
  double radius2 = grid_step_size * grid_step_size;
  double iradius16 = - global_paraMgr.wLop.getDouble("H Gaussian Para") / radius2;

  for (int i = 0; i < pvs_size; ++i)
  {
    CVertex &pvs_v = pvs->vert[i];
    //std::cout<<"pvs_v neighbor size: "<<pvs_v.neighbors.size() <<std::endl;
    for (int j = 0; j < pvs_v.original_neighbors.size(); ++j)
    {
      CVertex &n = detect_result->vert[pvs_v.original_neighbors[j]];
      double dist2 = (pvs_v.P() - n.P()).SquaredNorm();
      double den = exp(dist2 * iradius16);
      pvs_v.pvs_density += den;
    }

    density_max = density_max > pvs_v.pvs_density ? density_max : pvs_v.pvs_density;
    density_min = density_min < pvs_v.pvs_density ? density_min : pvs_v.pvs_density;
  }

  //normalize the pvs density
  double density_delta = density_max - density_min;
  if (abs(density_delta) < 1e-7)
  {
    std::cout<<"density delta == 0! ERROR: Divided by zero! " <<std::endl;
  }else
  {
    for (int i = 0; i < pvs_size; ++i)
      pvs->vert[i].pvs_density = (pvs->vert[i].pvs_density - density_min) / density_delta;
  }  

  vector<int> v_pvs_occupied_index;
  if (!detect_result->vert.empty())
  {
    for (int o_i = 0; o_i < detect_result->vert.size(); ++o_i)
    {
      CVertex &o_v = detect_result->vert[o_i];
      //get the x,y,z index of each scan result vertex
      int sc_v_indexX = static_cast<int>( ceil((o_v.P()[0] - whole_space_box_min.X()) / grid_step_size ));
      int sc_v_indexY = static_cast<int>( ceil((o_v.P()[1] - whole_space_box_min.Y()) / grid_step_size ));
      int sc_v_indexZ = static_cast<int>( ceil((o_v.P()[2] - whole_space_box_min.Z()) / grid_step_size ));
      int index = sc_v_indexX * y_max * z_max + sc_v_indexY * z_max + sc_v_indexZ;

      if (index >= pvs_size || index < 0)  break;

      pvs->vert[index].is_ray_stop = true;   //the grid is occupied by object, so the ray should stop
      pvs->vert[index].pvs_value = 0;        //0: pvs grid occupied
      pvs->vert[index].N() += o_v.N();

      //record the total points num and boarder points num
      pvs->vert[index].total_point_num += 1;
      if (o_v.is_boundary)
        pvs->vert[index].boarder_point_num += 1;

      v_pvs_occupied_index.push_back(index);
    }
  }

  //calculate ni of each pvs grid
  for (int p_i = 0; p_i < v_pvs_occupied_index.size(); ++p_i)
  {
    int idx = v_pvs_occupied_index[p_i];
    if (pvs->vert[idx].total_point_num == 0)
    {
      std::cout<<"pvs grid point num == 0!" <<std::endl;
      continue;
    }
    pvs->vert[idx].N() /= pvs->vert[idx].total_point_num;
    pvs->vert[idx].N().Normalize();
  }

  //STEP3: From angle of Scan Candidate: for newly visited points, update the pvs values
  Point3f x_axis(1.0f, 0.0f, 0.0f);
  Point3f z_axis(0.0f, 0.0f, 1.0f);

  if (scan_history->empty())
  {
    std::cout<<"Empty scan history in runUpdatePVS!" <<std::endl;
    return;
  }

  ScanCandidate sc = scan_history->back();

  Point3f pos = sc.first;
  Point3f viewray = sc.second.Normalize();
  Point3f up, right;
  //compute up and right direction of the candidates
  if (viewray.Z() > 0)
    up = viewray ^ x_axis;
  else if (fabs(viewray.Z()) < EPS_SUN)
    up = viewray ^ z_axis;
  else
    up = x_axis ^ viewray;

  right = viewray ^ up;
  up = up.Normalize();
  right = right.Normalize();

  vector<int> hit_grid_indexes;
  //compute the end of the ray
  Point3f viewray_end = pos + viewray * camera_max_dist;
  //get the x,y,z index of each nbv_candidate
  int t_indexX = static_cast<int>( ceil((pos[0] - whole_space_box_min.X()) / grid_step_size ));
  int t_indexY = static_cast<int>( ceil((pos[1] - whole_space_box_min.Y()) / grid_step_size ));
  int t_indexZ = static_cast<int>( ceil((pos[2] - whole_space_box_min.Z()) / grid_step_size ));
  //next point index along the ray, pay attention , index should be stored in double ,used in integer
  double n_indexX, n_indexY, n_indexZ;
  //for DDA algorithm
  double deltaX, deltaY, deltaZ; 
  double x = 0.0f, y = 0.f, z = 0.0f;
  double length = 0.0f;
  //sweep
  int n_point_hr_half  = static_cast<int>(0.5 * far_horizon_dist / resolution);
  int n_point_ver_half = static_cast<int>(0.5 * far_vertical_dist / resolution);
  std::cout<< n_point_hr_half <<" " << n_point_ver_half <<std::endl;

  for (int i = - n_point_hr_half; i < n_point_hr_half; ++i)
  {
    for (int j = - n_point_ver_half; j < n_point_ver_half; ++j)
    {
      Point3f viewray_end_iter = viewray_end + right * (i * resolution) + up * (j * resolution);
      Point3f viewray_iter = viewray_end_iter - pos;
      //line direction vector
      Point3f line_dir = viewray_iter.Normalize();

      x = line_dir.X(); y = line_dir.Y(); z = line_dir.Z();
      //reset the next grid indexes
      n_indexX = t_indexX; n_indexY = t_indexY; n_indexZ = t_indexZ;
      //2. compute the next grid indexes
      length = GlobalFun::getAbsMax(x, y, z);
      deltaX = x / length; 
      deltaY = y / length;
      deltaZ = z / length;

      for (int k = 0; k <= max_steps; ++k)
      {
        n_indexX = n_indexX + deltaX;
        n_indexY = n_indexY + deltaY;
        n_indexZ = n_indexZ + deltaZ;
        int index = round(n_indexX) * y_max * z_max + round(n_indexY) * z_max + round(n_indexZ);

        //if (n_indexX < 0 || n_indexY < 0 || n_indexZ < 0) break;//the index is out of pvs grids
        if (index >= pvs_size || index < 0)  break;
        //if the direction is into the model, or has been hit, then stop tracing
        if (pvs->vert[index].is_ray_stop) break;            
        if (pvs->vert[index].is_ray_hit)  continue;

        ////if the grid get first hit 
        pvs->vert[index].is_ray_hit = true;
        pvs->vert[index].pvs_value = 1;

        // record hit_grid center index
        hit_grid_indexes.push_back(index); 
      }// end for k
    }//end for j
  }//end for i

  if (hit_grid_indexes.size() > 0)
  {
    vector<int>::iterator it;
    for (it = hit_grid_indexes.begin(); it != hit_grid_indexes.end(); ++it)
      pvs->vert[*it].is_ray_hit = false;
    hit_grid_indexes.clear();
  }
}

void PVSBasedNBV::runSearchNewBoundaries()
{
  findBoarderPoints();
  //clear former boundaries and search for new ones
  m_v_boundaries->clear();
  searchNewBoundaries();

  //refine the board points
  for (int i = 0; i < m_v_boundaries->size(); ++i)
  {
    int curve_size = m_v_boundaries->at(i).curve.size();
    for (int j = 0; j < curve_size; ++j)
    {
      detect_result->vert[m_v_boundaries->at(i).curve.at(j).m_index].is_boundary = true;
    }
  }
}

void PVSBasedNBV::runSearchNewBoundariesByBallpivoting()
{
  findBoarderPointsByBallpivoting();
  //clear former boundaries and search for new ones
  m_v_boundaries->clear();
  searchNewBoundaries();
}

void PVSBasedNBV::runComputeCandidates()
{
  if (m_v_boundaries->empty())
  {
    global_paraMgr.pvsBasedNBV.setValue("Is PVS Stop", BoolValue(true));
    std::cout<<"No Boundary Found! Algorithm Finished! " <<std::endl;
    std::cout<<"Scan Count: " <<*scan_count <<std::endl;
    return;
  }
  
  double step_aside_size = 0.0; //global_paraMgr.wLop.getDouble("CGrid Radius");
  double camera_far_dist = global_paraMgr.camera.getDouble("Camera Far Distance") /
    global_paraMgr.camera.getDouble("Predicted Model Size");

  GlobalFun::clearCMesh(*nbv_candidates);
  for (int i = 0; i < m_v_boundaries->size(); ++i)
  {
    Boundary b = m_v_boundaries->at(i);
    int mid_curve_idx = b.curve.size() / 2;
    int cmesh_index = b.curve[mid_curve_idx].m_index;
    //corresponding point in CMesh
    CVertex &v = detect_result->vert[cmesh_index];
    //find one point which is not a board point
    CVertex *inside_point;
    for (int j = 0; j < v.neighbors.size(); ++j)
    {
      CVertex &n = detect_result->vert[v.neighbors[j]];
      if (n.IsB()) continue;
      else { inside_point = &n; break;}
    }
    
    Point3f boder_dir = b.curve[mid_curve_idx].P() - b.curve[mid_curve_idx - 1].P();
    Point3f mid_point_normal = b.curve[mid_curve_idx].N();
    Point3f step_aside_direction = (boder_dir ^ mid_point_normal).Normalize();//cross product, remember always normalize

    Point3f inside_direction = (*inside_point).P() - b.curve[mid_curve_idx].P();
    if (step_aside_direction * inside_direction > 0)
      step_aside_direction = -step_aside_direction;

    //clear candidates
    CVertex candidate = v;
    candidate.m_index = i;
    candidate.P() = v.P() + step_aside_direction * step_aside_size;
    GlobalFun::printPoint3(std::cout, v.P());
    GlobalFun::printPoint3(std::cout, candidate.P());
    GlobalFun::printPoint3(std::cout, step_aside_direction);

    nbv_candidates->vert.push_back(candidate);
    nbv_candidates->bbox.Add(candidate.P());
  }
  nbv_candidates->vn = nbv_candidates->vert.size();

  std::cout<< "nbv candidates size: "<< nbv_candidates->vert.size() <<std::endl;

  if (nbv_candidates->vert.empty())
  {
    global_paraMgr.pvsBasedNBV.setValue("Is PVS Stop", BoolValue(true));
    std::cout<<"No candidates found! Algorithm Finished Correctly !" <<std::endl;
    std::cout<<"Scan count: " <<*scan_count <<std::endl;
    return;
  }

  //adjust the candidates according to the poisson surface
  if (iso_points->vert.empty())
  {
    std::cout<<"ERROR: Iso points empty!" <<std::endl;
    return;
  }
  std::cout<<iso_points->vert.size() << std::endl;
  GlobalFun::computeAnnNeigbhors(iso_points->vert, nbv_candidates->vert, 5, false, "PVS search nearest point on poisson surface to the given coarse candidates");
  
  scan_candidates->clear();
  for (int i = 0; i < nbv_candidates->vert.size(); ++i)
  {
    CVertex &iso_nearest = iso_points->vert[nbv_candidates->vert[i].neighbors[0]];
    // move outward in the normal direction and reverse the normal to get the nbv direction
    nbv_candidates->vert[i].P() = iso_nearest.P() + iso_nearest.N() * optimalDist;
    nbv_candidates->vert[i].N() = -iso_nearest.N();
    scan_candidates->push_back(std::make_pair(nbv_candidates->vert[i].P(), nbv_candidates->vert[i].N()));
  }
}

void PVSBasedNBV::findBoarderPoints()
{
  //copy point from original to sample
  GlobalFun::clearCMesh(*sample); 
  sample->vert.EnableRadius();
  int idx = 0;
  for (int i = 0; i < original->vert.size(); ++i)
  {
    CVertex v = original->vert[i];
    v.m_index = idx++;
    v.is_original = false;
    v.is_fixed_sample = true;
    //clear the boarder flag
    v.is_boundary = false;
    sample->vert.push_back(v);
    sample->bbox.Add(v.P());
  }
  sample->vn = sample->vert.size();
 
  //2.another surface reconstruction method
  GaelMls::APSS<CMesh> mls_apss(*sample);
  mls_apss.computeVertexRaddi();

  GaelMls::MlsSurface<CMesh>* mls = 0;
  GaelMls::RIMLS<CMesh>* rimls = 0;
  mls = rimls = new RIMLS<CMesh>(*sample);

  mls->setFilterScale(2);
  mls->setMaxProjectionIters(15);
  mls->setProjectionAccuracy(0.0001);

  //marching cube
  typedef vcg::tri::MlsWalker<CMesh,MlsSurface<CMesh> > MlsWalker;
  typedef vcg::tri::MarchingCubes<CMesh, MlsWalker> MlsMarchingCubes;
  MlsWalker walker;
  walker.resolution = 200;
  //iso extraction
  MlsMarchingCubes mc(*detect_result, walker);
  walker.BuildMesh<MlsMarchingCubes>(*detect_result, *mls, mc);

  //update face topology
  detect_result->face.EnableFFAdjacency();
  vcg::tri::UpdateTopology<CMesh>::FaceFace(*detect_result);
  vcg::tri::UpdateFlags<CMesh>::FaceBorderFromFF(*detect_result);
  vcg::tri::SmallComponent<CMesh>::Select(*detect_result, 0.1f);
  vcg::tri::SmallComponent<CMesh>::DeleteFaceVert(*detect_result);
  vcg::tri::UpdateNormals<CMesh>::PerVertexPerFace(*detect_result);
  detect_result->face.DisableFFAdjacency();
  
  for (int i = 0; i < detect_result->vert.size(); ++i)
    detect_result->vert[i].m_index = i;
  //method2 ends


  vcg::tri::UpdateFlags<CMesh>::VertexBorderFromNone(*detect_result); 
  //select the border vertexes
  std::cout<<"selected boarder points: " <<tri::UpdateSelection<CMesh>::VertexFromBorderFlag(*detect_result) << std::endl;
  //print them out
  for (int i = 0; (i < detect_result->vert.size()); ++i)
    if(detect_result->vert[i].IsB())
      detect_result->vert[i].is_boundary = true;
}

void PVSBasedNBV::findBoarderPointsByBallpivoting()
{
  //copy point from original to sample
  GlobalFun::clearCMesh(*detect_result); 
  detect_result->vert.EnableRadius();
  int idx = 0;
  for (int i = 0; i < original->vert.size(); ++i)
  {
    CVertex v = original->vert[i];
    v.m_index = idx++;
    v.is_original = false;
    v.is_fixed_sample = true;
    //clear the boarder flag
    v.is_boundary = false;
    detect_result->vert.push_back(v);
    detect_result->bbox.Add(v.P());
  }
  detect_result->vn = detect_result->vert.size();

  //1. use BallPivoting to reconstruct surfaces
  double resolution = global_paraMgr.camera.getDouble("Camera Resolution");
  GlobalFun::ballPivotingReconstruction(*detect_result, 0.05);

  vcg::tri::UpdateFlags<CMesh>::VertexBorderFromNone(*detect_result); 
  //select the border vertexes
  std::cout<<"selected boarder points: " <<tri::UpdateSelection<CMesh>::VertexFromBorderFlag(*detect_result) << std::endl;
  //print them out
  for (int i = 0; (i < detect_result->vert.size()); ++i)
    if(detect_result->vert[i].IsB())
      detect_result->vert[i].is_boundary = true;
}

void PVSBasedNBV::searchNewBoundaries()
{
  int boundary_knn = para->getInt("Boundary Search KNN");
  GlobalFun::computeAnnNeigbhors(detect_result->vert, detect_result->vert, boundary_knn, false, "void PVSBasedNBV::search New Boundaies");
  
  std::cout<<"rimls result neighbors: " <<detect_result->vert[0].neighbors.size() <<std::endl;

  while (true)
  {
    //make sure boundary search begin at boundary point
    int begin_idx = -1;
    for (int i = 0; i < detect_result->vert.size(); ++i)
    {
      if (detect_result->vert[i].is_boundary) 
      {
        begin_idx = i;
        break;
      }
    }
    
    if (begin_idx < 0) break;//exit the while loop

    Boundary new_boundary = searchOneBoundaryFromIndex(begin_idx);
    int accept_boundary_size = para->getInt("Accept Boundary Size");
    if (new_boundary.getSize() >= accept_boundary_size)
    {
      for (int j = 0; j < new_boundary.getSize(); ++j)
      {
        detect_result->vert[new_boundary.curve[j].m_index].is_boundary = false;//mark the point have been handled
      }
      m_v_boundaries->push_back(new_boundary);
    }
  }

  std::cout<< m_v_boundaries->size() <<" boundaries detected!" <<std::endl;
}

Boundary PVSBasedNBV::searchOneBoundaryFromIndex(int begin_idx)
{
  Boundary new_boundary;
  CVertex& begin_vert = detect_result->vert[begin_idx];
  if (!begin_vert.is_boundary)
  {
    std::cout<< "start point is not a border one! " <<std::endl;
    return new_boundary;
  }
  
  if (begin_vert.neighbors.size() < 1)
  {
    std::cout<<"Empty neighbors of begin_vert!" <<std::endl;
    return new_boundary;
  }
  //we should alter the treated point property
  begin_vert.is_boundary = false;

  int nearest_idx = -1;
  for (int i = 0; i < begin_vert.neighbors.size(); ++i)
  {
    CVertex& t = detect_result->vert[begin_vert.neighbors[i]];
    if (t.is_boundary)
    {
      nearest_idx = begin_vert.neighbors[i];
      break;
    }
  }

  if (nearest_idx < 0) return new_boundary;

  CVertex &t = detect_result->vert[nearest_idx];
  Point3f head_direction = (t.P() - begin_vert.P()).Normalize();
  Boundary boundary_first_part = searchOneBoundaryFromDirection(begin_idx, head_direction);
  Boundary boundary_second_part = searchOneBoundaryFromDirection(begin_idx, -head_direction);
  
  //combine them
  Curve curve_first_part = boundary_first_part.curve;
  Curve curve_second_part = boundary_second_part.curve;

  Curve::reverse_iterator r_iter = curve_second_part.rbegin();

  for (int i = 0; i < curve_second_part.size()-1; ++i) 
  {
    new_boundary.pushBackCVertex(*r_iter);
    r_iter++;
  }

  for (int i = 0; i < curve_first_part.size(); ++i)
  {
    new_boundary.pushBackCVertex(curve_first_part[i]);
  }

  return new_boundary;
}

Boundary PVSBasedNBV::searchOneBoundaryFromDirection(int begin_idx, Point3f direction)
{
  Boundary new_boundary;
  int curr_idx = begin_idx;
  do 
  {
    CVertex curr_vertex = detect_result->vert[curr_idx];
    new_boundary.pushBackCVertex(curr_vertex);
    detect_result->vert[curr_idx].is_boundary = false;

    Point3f new_direction;
    int next_idx = -1;
    for (int i = 0; i < curr_vertex.neighbors.size(); ++i)
    {
      CVertex &t = detect_result->vert[curr_vertex.neighbors[i]];

      if (!t.is_boundary) continue;

      new_direction = (t.P() - curr_vertex.P()).Normalize();
      double angle = GlobalFun::computeRealAngleOfTwoVertor(direction, new_direction);
      //for test
      if (abs(angle - (-1)) < 1e-4)
      {
        GlobalFun::printPoint3(std::cout, new_direction);
        GlobalFun::printPoint3(std::cout, direction);
      }
      if (angle > para->getDouble("Boundary Search Angle"))  continue;

      next_idx = curr_vertex.neighbors[i];
      break;
    }

    if (next_idx < 0) break;

    direction = new_direction;
    curr_idx = next_idx;
  } while (true);

  return new_boundary;
}

void PVSBasedNBV::buildSphereCandidatesIEEE()
{
  //build voxels for IEEE Sphere
  runBuildPVS();
}

void PVSBasedNBV::computeScoreIEEE()
{
  double alpha_occplane = 0.8, beta_occupied = 0.2;
  double occplane_percentage = 0.0, occupied_percentage = 0.0;
  double f_area = 0.0, f_navigation = 0.0, f_quality = 0.0f, f_occlusion = 0.0f, f_final = 0.0f;

  double resolution = global_paraMgr.camera.getDouble("Camera Resolution");
  double far_horizon_dist = global_paraMgr.camera.getDouble("Camera Horizon Dist") 
    / global_paraMgr.camera.getDouble("Predicted Model Size");
  double far_vertical_dist = global_paraMgr.camera.getDouble("Camera Vertical Dist")
    / global_paraMgr.camera.getDouble("Predicted Model Size");
  double camera_max_dist = global_paraMgr.camera.getDouble("Camera Far Distance") /
    global_paraMgr.camera.getDouble("Predicted Model Size");
  int grid_resolution = global_paraMgr.pvsBasedNBV.getDouble("PVS Grid Resolution");
  if (grid_resolution <= 2)
    return;
  double grid_step_size = (camera_max_dist*2.0 + 1.0) / (grid_resolution - 1);
  int max_steps = static_cast<int>(camera_max_dist / grid_step_size);
  int pvs_size = pvs->vert.size();

  /*if (scan_history->empty())
  {
  std::cout<<"Empty scan history in compute score!"<< std::endl;
  }else
  {
  ScanCandidate sc = scan_history->back();
  }*/

  for (int i = 0; i < nbv_candidates->vert.size(); ++i)
  {
    CVertex &c = nbv_candidates->vert[i];

    //step1: Area Factor
    int total_voxels_num = 0;
    int occplane_num = 0, occupied_num = 0;
    Point3f x_axis(1.0f, 0.0f, 0.0f), z_axis(0.0f, 0.0f, 1.0f);
        
    Point3f pos = c.P();
    Point3f viewray = c.N().Normalize();
    Point3f up, right;
    //compute up and right direction of the candidates
    if (viewray.Z() > 0)
      up = viewray ^ x_axis;
    else if (fabs(viewray.Z()) < EPS_SUN)
      up = viewray ^ z_axis;
    else
      up = x_axis ^ viewray;

    right = viewray ^ up;
    up = up.Normalize();
    right = right.Normalize();

    vector<int> hit_grid_indexes;
    //compute the end of the ray
    Point3f viewray_end = pos + viewray * camera_max_dist;
    //get the x,y,z index of each nbv_candidate
    int t_indexX = static_cast<int>( ceil((pos[0] - whole_space_box_min.X()) / grid_step_size ));
    int t_indexY = static_cast<int>( ceil((pos[1] - whole_space_box_min.Y()) / grid_step_size ));
    int t_indexZ = static_cast<int>( ceil((pos[2] - whole_space_box_min.Z()) / grid_step_size ));
    //next point index along the ray, pay attention , index should be stored in double ,used in integer
    double n_indexX = 0.0f, n_indexY = 0.0f, n_indexZ = 0.0f;
    //for DDA algorithm
    double deltaX = 0.0, deltaY = 0.0f, deltaZ = 0.0f; 
    double x = 0.0f, y = 0.f, z = 0.0f;
    double length = 0.0f;
    //sweep
    int n_point_hr_half  = static_cast<int>(0.5 * far_horizon_dist / resolution);
    int n_point_ver_half = static_cast<int>(0.5 * far_vertical_dist / resolution);

    for (int i = - n_point_hr_half; i < n_point_hr_half; ++i)
    {
      for (int j = - n_point_ver_half; j < n_point_ver_half; ++j)
      {
        Point3f viewray_end_iter = viewray_end + right * (i * resolution) + up * (j * resolution);
        Point3f viewray_iter = viewray_end_iter - pos;
        //line direction vector
        Point3f line_dir = viewray_iter.Normalize();

        x = line_dir.X(); y = line_dir.Y(); z = line_dir.Z();
        //reset the next grid indexes
        n_indexX = t_indexX; n_indexY = t_indexY; n_indexZ = t_indexZ;
        //2. compute the next grid indexes
        length = GlobalFun::getAbsMax(x, y, z);
        deltaX = x / length; 
        deltaY = y / length;
        deltaZ = z / length;
        for (int k = 0; k <= max_steps; ++k)
        {
          n_indexX = n_indexX + deltaX;
          n_indexY = n_indexY + deltaY;
          n_indexZ = n_indexZ + deltaZ;
          int index = round(n_indexX) * y_max * z_max + round(n_indexY) * z_max + round(n_indexZ);
          
          if (index >= pvs_size || index < 0)  break;

          if (pvs->vert[index].is_ray_hit)  continue;
          ////if the grid get first hit 
          pvs->vert[index].is_ray_hit = true;
          // record hit_grid center index
          hit_grid_indexes.push_back(index); 
          
          //if the direction is into the model, or has been hit, then stop tracing
          if (pvs->vert[index].is_ray_stop)
          {
            total_voxels_num = total_voxels_num + 1;//(max_steps - k + 1);//TODO
            if (pvs->vert[index].is_boundary)
            {
              occplane_num++;
            }
            else
            {
              occupied_num++;
              //for compute quality factor
              if (abs(pvs->vert[index].N().Norm()) < 1e-10)
                continue;
              else
                f_quality += (-line_dir * pvs->vert[index].N()) / (line_dir.Norm() * pvs->vert[index].N().Norm());
            }
            break;
          }
        }// end for k
      }//end for j
    }//end for i

    if (hit_grid_indexes.size() > 0)
    {
      vector<int>::iterator it;
      for (it = hit_grid_indexes.begin(); it != hit_grid_indexes.end(); ++it)
        pvs->vert[*it].is_ray_hit = false;
      hit_grid_indexes.clear();
    }

    if (total_voxels_num > 0)
    {
      occplane_percentage = 1.0f * occplane_num / total_voxels_num;
      occupied_percentage = 1.0f * occupied_num / total_voxels_num;
      f_area = areaFactorIEEE(occplane_percentage, alpha_occplane) + areaFactorIEEE(occupied_percentage, beta_occupied);
    }

    //step2: navigation factor
    double dist = 0.0f;//c.P().Norm() * GlobalFun::computeRealAngleOfTwoVertor(c.P(), sc.first) / 180.0 * PI;
    f_navigation = navigationFactorIEEE(dist);
    
    //step3: Quality factor
    if (total_voxels_num > occplane_num)
      f_quality = f_quality / (total_voxels_num - occplane_num);

    //step4: occlusion factor
    f_occlusion = occplane_num / ( 4 * n_point_hr_half * n_point_ver_half);

    f_final = f_area * (f_quality + f_navigation + f_occlusion);
    c.eigen_confidence = f_final;
  }//for candidates
}

void PVSBasedNBV::updateIEEE()
{
  //find boarder points 
  //copy point from original to sample
  GlobalFun::clearCMesh(*detect_result);
  detect_result = original;
  vcg::tri::UpdateFlags<CMesh>::VertexBorderFromNone(*detect_result); 
  //select the border vertexes
  std::cout<<"selected boarder points: " <<tri::UpdateSelection<CMesh>::VertexFromBorderFlag(*detect_result) << std::endl;
  //print them out
  for (int i = 0; (i < detect_result->vert.size()); ++i)
    if(detect_result->vert[i].IsB())
      detect_result->vert[i].is_boundary = true;

  double resolution = global_paraMgr.camera.getDouble("Camera Resolution");
  double far_horizon_dist = global_paraMgr.camera.getDouble("Camera Horizon Dist") 
    / global_paraMgr.camera.getDouble("Predicted Model Size");
  double far_vertical_dist = global_paraMgr.camera.getDouble("Camera Vertical Dist")
    / global_paraMgr.camera.getDouble("Predicted Model Size");
  double camera_max_dist = global_paraMgr.camera.getDouble("Camera Far Distance") /
    global_paraMgr.camera.getDouble("Predicted Model Size");
  int grid_resolution = global_paraMgr.pvsBasedNBV.getDouble("PVS Grid Resolution");
  if (grid_resolution <= 2)
    return;
  double grid_step_size = (camera_max_dist*2.0 + 1.0) / (grid_resolution - 1);

  int pvs_size = pvs->vert.size();
  std::set<int> v_pvs_occupied_index;
  if (!detect_result->vert.empty())
  {
    for (int o_i = 0; o_i < detect_result->vert.size(); ++o_i)
    {
      CVertex &o_v = detect_result->vert[o_i];
      //get the x,y,z index of each scan result vertex
      int sc_v_indexX = static_cast<int>( ceil((o_v.P()[0] - whole_space_box_min.X()) / grid_step_size ));
      int sc_v_indexY = static_cast<int>( ceil((o_v.P()[1] - whole_space_box_min.Y()) / grid_step_size ));
      int sc_v_indexZ = static_cast<int>( ceil((o_v.P()[2] - whole_space_box_min.Z()) / grid_step_size ));
      int index = sc_v_indexX * y_max * z_max + sc_v_indexY * z_max + sc_v_indexZ;

      if (index >= pvs_size || index < 0)  break;
      
      pvs->vert[index].is_ray_stop = true;   //the grid is occupied by object, so the ray should stop
      pvs->vert[index].pvs_value = 1.0f; 

      //if the voxel contains a border point, we consider it a occplane voxel
      if (o_v.is_boundary)
      {
        pvs->vert[index].is_boundary = true;
        pvs->vert[index].pvs_value = 0.0f;
      }

      pvs->vert[index].N() += o_v.N();
      //record the total points num and boarder points num
      pvs->vert[index].total_point_num += 1;
      v_pvs_occupied_index.insert(index);
    }
  }

  //called "surface normal" in IEEE Sphere
  std::set<int>::iterator set_iter = v_pvs_occupied_index.begin();
  for (; set_iter != v_pvs_occupied_index.end(); ++set_iter)
  {
    int idx = *set_iter;
    if (pvs->vert[idx].total_point_num == 0)
    {
      std::cout<<"pvs grid point num == 0!" <<std::endl;
      continue;
    }
    pvs->vert[idx].N() /= pvs->vert[idx].total_point_num;
    pvs->vert[idx].N().Normalize();
  }
}

void PVSBasedNBV::selectCandidateIEEE()
{
  sort(nbv_candidates->vert.begin(), nbv_candidates->vert.end(), cmp);
  int topN = global_paraMgr.nbv.getInt("NBV Top N");

  scan_candidates->clear();
  for (int i = 0; i < nbv_candidates->vert.size() && i < topN; ++i)
  {
    CVertex &c = nbv_candidates->vert[i];
    std::cout<<"candidate confidence: "<<c.eigen_confidence <<std::endl;
    scan_candidates->push_back(make_pair(c.P(), c.N()));
  }
  std::cout<<"scan candidates size: " <<scan_candidates->size() <<std::endl;
  
  GlobalFun::clearCMesh(*nbv_candidates);
  for (int i = 0; i < scan_candidates->size(); ++i)
  {
    CVertex v;
    v.m_index = i;
    v.P() = scan_candidates->at(i).first;
    v.N() = scan_candidates->at(i).second;
    nbv_candidates->vert.push_back(v);
    nbv_candidates->bbox.Add(v.P());
  }
  nbv_candidates->vn = nbv_candidates->vert.size();
}

double PVSBasedNBV::areaFactorIEEE(double percentage, double optimum_percentage)
{
  double f = 0.0;
  if (percentage > optimum_percentage)
  {
    f = - 2 / (optimum_percentage * optimum_percentage * optimum_percentage) * percentage * percentage * percentage
            + 3 / (optimum_percentage * optimum_percentage) * percentage * percentage;
  }
  else
  {
    double cube = (optimum_percentage - 1) * (optimum_percentage - 1) * (optimum_percentage - 1);
    f = - 2 / cube * percentage * percentage * percentage + 3 * (optimum_percentage + 1) / cube * percentage * percentage
            - 6 * optimum_percentage / cube * percentage + (3 * optimum_percentage - 1) / cube;
  }
  return f;
}

double PVSBasedNBV::navigationFactorIEEE(double distance, double rho)
{
  return (1 - rho) * distance * distance + 1;
}

void PVSBasedNBV::runSelectCandidate()
{
  int max_score_candidate_idx = -1;
  double candidate_max_score = -1000;

  double resolution = global_paraMgr.camera.getDouble("Camera Resolution");
  double far_horizon_dist = global_paraMgr.camera.getDouble("Camera Horizon Dist") 
    / global_paraMgr.camera.getDouble("Predicted Model Size");
  double far_vertical_dist = global_paraMgr.camera.getDouble("Camera Vertical Dist")
    / global_paraMgr.camera.getDouble("Predicted Model Size");
  double camera_max_dist = global_paraMgr.camera.getDouble("Camera Far Distance") /
    global_paraMgr.camera.getDouble("Predicted Model Size");
  int grid_resolution = global_paraMgr.pvsBasedNBV.getDouble("PVS Grid Resolution");
  if (grid_resolution <= 2)
    return;
  double grid_step_size = (camera_max_dist*2.0 + 1.0) / (grid_resolution - 1);
  int max_steps = static_cast<int>(camera_max_dist / grid_step_size);

  Point3f x_axis(1.0f, 0.0f, 0.0f);
  Point3f z_axis(0.0f, 0.0f, 1.0f);

  std::cout<<"nbv candidates size: "<< nbv_candidates->vert.size() <<std::endl;
  for ( int c_i = 0; c_i < nbv_candidates->vert.size(); ++c_i)
  {
    //formula transformation
    double ev_sum = 0.0f;
    double qs_sum = 0.0f;
    double lambda_coeff = 0.7f;
    int    nq = 5;
    double w = ((*scan_count) / nq) / (1 + (*scan_count) / nq);
    int candidate_hit_count = 0;

    CVertex &v = nbv_candidates->vert[c_i];
    Point3f viewray = v.N().Normalize();
    Point3f pos = v.P();
    Point3f up, right;
    //compute up and right direction of the candidates

    vector<int> hit_grid_indexes;
    //compute the end of the ray
    Point3f viewray_end = pos + viewray * camera_max_dist;
    //get the x,y,z index of each nbv_candidate
    int t_indexX = static_cast<int>( ceil((v.P()[0] - whole_space_box_min.X()) / grid_step_size ));
    int t_indexY = static_cast<int>( ceil((v.P()[1] - whole_space_box_min.Y()) / grid_step_size ));
    int t_indexZ = static_cast<int>( ceil((v.P()[2] - whole_space_box_min.Z()) / grid_step_size ));
    //std::cout<< t_indexX<<" " << t_indexY<< " " << t_indexZ<< " " <<std::endl;
    //next point index along the ray, pay attention , index should be stored in double ,used in integer
    double n_indexX, n_indexY, n_indexZ;
    //for DDA algorithm
    double deltaX, deltaY, deltaZ; 
    double x = 0.0f, y = 0.f, z = 0.0f;
    double length = 0.0f;
    //sweep
    int n_point_hr_half  = static_cast<int>(0.5 * far_horizon_dist / resolution);
    int n_point_ver_half = static_cast<int>(0.5 * far_vertical_dist / resolution);

    for (int i = - n_point_hr_half; i < n_point_hr_half; ++i)
    {
      for (int j = - n_point_ver_half; j < n_point_ver_half; ++j)
      {
        Point3f viewray_end_iter = viewray_end + right * (i * resolution) + up * (j * resolution);
        Point3f viewray_iter = viewray_end_iter - pos;
        //line direction vector
        Point3f line_dir = viewray_iter.Normalize();

        x = line_dir.X(); y = line_dir.Y(); z = line_dir.Z();
        //reset the next grid indexes
        n_indexX = t_indexX; n_indexY = t_indexY; n_indexZ = t_indexZ;
        //2. compute the next grid indexes
        length = GlobalFun::getAbsMax(x, y, z);
        deltaX = x / length; 
        deltaY = y / length;
        deltaZ = z / length;

        for (int k = 0; k <= max_steps; ++k)
        {
          n_indexX = n_indexX + deltaX;
          n_indexY = n_indexY + deltaY;
          n_indexZ = n_indexZ + deltaZ;
          int index = round(n_indexX) * y_max * z_max + round(n_indexY) * z_max + round(n_indexZ);

          if (index >= pvs->vert.size())  break;
          //if the direction is into the model, or has been hit, then stop tracing
          //1. calculate qs: only these pvs grid points that own original points have this property
          if (pvs->vert[index].is_ray_stop)
          {
            candidate_hit_count++;
            //std::cout<<"candidate hit count: "<<candidate_hit_count <<std::endl;
            assert(pvs->vert[index].total_point_num != 0);
            if (pvs->vert[index].total_point_num == 0)
            {
              std::cout<<"pvs grid total points num 0!" <<std::endl;
              break;
            }

            double bi = pvs->vert[index].boarder_point_num / pvs->vert[index].total_point_num;
            double di = pvs->vert[index].pvs_density;
            double qi = 0.0;
            //angle threshold 70бу
            if (GlobalFun::computeRealAngleOfTwoVertor(-line_dir, pvs->vert[index].N()) < 70.)
            {
              qi = lambda_coeff * bi + (1 - lambda_coeff) * di;
              //std::cout<<"bi: " <<bi << " di: " <<di <<std::endl;
            }

            qs_sum += qi;
            break;  
          }          

          //if the grid has been handled
          if (pvs->vert[index].is_ray_hit)  continue;
          else{
            //if the grid is first hit 
            pvs->vert[index].is_ray_hit = true;
            candidate_hit_count++;
            //std::cout<<"candidate hit count: "<<candidate_hit_count <<std::endl;

            double pi = pvs->vert[index].pvs_value;
            //std::cout<<"pi: "<<pi <<std::endl;
            //1.calculate evi
            double evi;
            if (abs(pi) < 1e-6 || abs(pi -1 ) < 1e-6)
              evi = 0.0f;
            else
              evi = -(pi * log(pi) + (1 - pi) * log(1 - pi));
            
            ev_sum += evi;
            // record hit_grid center index
            hit_grid_indexes.push_back(index); 
          }
        }// end for k
      }//end for j
    }//end for i

    if (hit_grid_indexes.size() > 0)
    {
      vector<int>::iterator it;
      for (it = hit_grid_indexes.begin(); it != hit_grid_indexes.end(); ++it)
        pvs->vert[*it].is_ray_hit = false;
      hit_grid_indexes.clear();
    }

    //final calculation
    double f_utility = 0.0f;
    if (candidate_hit_count == 0)
    {
      std::cout<<"candidate hit grid count: " << candidate_hit_count <<std::endl;
      continue;
    }
    double ev = ev_sum / candidate_hit_count;
    double qs = qs_sum / candidate_hit_count;
    std::cout<<"ev: " <<ev <<" qs: " <<qs <<std::endl;
    f_utility = (1 - w) * ev + w * (1 - qs);
    std::cout<<"f_utility: " <<f_utility <<std::endl;
    if (f_utility > candidate_max_score)
    {
      candidate_max_score = f_utility;
      max_score_candidate_idx = c_i;
    }
    std::cout<<"max_score: " <<candidate_max_score <<std::endl;
    std::cout<<"max_idx: " <<max_score_candidate_idx <<std::endl;
  }//end for nbv_candidates

  //find real max score candidate index, store the highest nbv
  scan_candidates->clear();
  if (max_score_candidate_idx >= 0)
  {
    scan_candidates->push_back(make_pair(nbv_candidates->vert[max_score_candidate_idx].P(),
      nbv_candidates->vert[max_score_candidate_idx].N()));
  }else
  {
    global_paraMgr.pvsBasedNBV.addParam(new RichBool("Is PVS Stop", false));
    std::cout<<"No scan candidates found! Something went wrong in PVSBasedNBV::runSelectCandidate!" <<std::endl;
  }
}

void PVSBasedNBV::runPVSMerge()
{
  double resolution = global_paraMgr.camera.getDouble("Camera Resolution");
  double Rr = resolution * 0.6;

  for ( int i = 0; i < scanned_results->size(); ++i)
  {
    CMesh *sr = scanned_results->at(i);
    assert(!original->vert.empty());
    int index = original->vert.back().m_index + 1;
    GlobalFun::computeBallNeighbors(sr, original, Rr, original->bbox);

    for (int j = 0; j < sr->vert.size(); ++j)
    {
      CVertex &v = sr->vert[j];
      //if (v.original_neighbors.size() > 0 ) continue;

      v.m_index = index++;      
      v.is_scanned = true;
      v.is_original = true;
      original->vert.push_back(v);
    }
  }
  original->vn = original->vert.size();
}

void PVSBasedNBV::runSphere()
{
  buildSphereCandidatesIEEE();

  updateIEEE();

  computeScoreIEEE();

  selectCandidateIEEE();
}

bool PVSBasedNBV::cmp(const CVertex &v1, const CVertex &v2)
{
  //in ascending order
  return v1.eigen_confidence > v2.eigen_confidence;
}