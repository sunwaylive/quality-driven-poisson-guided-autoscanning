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

  model = pData->getCurrentModel();
  sample = pData->getCurrentSamples();

  scanned_results = pData->getScannedResults();
  optimalDist = (global_paraMgr.camera.getDouble("Camera Far Distance") +global_paraMgr.camera.getDouble("Camera Near Distance")) 
    / 2 / global_paraMgr.camera.getDouble("Predicted Model Size");
  nbv_candidates = pData->getNbvCandidates();
  scan_candidates = pData->getScanCandidates();
  m_v_boundaries = pData->getBoundaries();
  iso_points = pData->getCurrentIsoPoints();
  pvs = pData->getPVS();
}

void PVSBasedNBV::run()
{
  if (para->getBool("Run PVS Detect Boundary"))
  {
    std::cout << "Run PVS Detect Boundary" <<std::endl;
    runPVSDetectBoundary();
  }
  if (para->getBool("Run PVS Search New Boundaries"))
  {
    std::cout<< "Run PVS Search New Boundaries!" <<std::endl;
    runSearchNewBoundaries();
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
}

void PVSBasedNBV::clear()
{

}

void PVSBasedNBV::runPVSDetectBoundary()
{
  //copy point from original to sample
  GlobalFun::clearCMesh(*sample); 
  int idx = 0;
  for (int i = 0; i < original->vert.size(); ++i)
  {
    CVertex v = original->vert[i];
    v.m_index = idx++;
    v.is_original = false;
    v.is_fixed_sample = true;
    sample->vert.push_back(v);
    sample->bbox.Add(v.P());
  }
  sample->vn = sample->vert.size();

  //compute the topology of sample points
  GlobalFun::ballPivotingReconstruction(*sample,global_paraMgr.wLop.getDouble("CGrid Radius"));
  //update the relation between vertexes and faces

  //use vertex topology
  //mark the border vertexes
  vcg::tri::UpdateFlags<CMesh>::VertexBorderFromNone(*sample); 
  //select the border vertexes
  std::cout<<"vertex line: " <<tri::UpdateSelection<CMesh>::VertexFromBorderFlag(*sample) << std::endl;
  //print them out
  for (int i = 0; (i < sample->vert.size()); ++i)
  {
    if(sample->vert[i].IsS())
      std::cout <<" point on border: "<< sample->vert[i].IsB() <<std::endl;
  }

  //vcg::tri::UpdateTopology<CMesh>::VertexFace(*sample);
  //update board vertex of faces
  //vcg::tri::UpdateFlags<CMesh>::FaceBorderFromNone(*sample);
  //select the board faces according to the vertex board property
  
  //traverse all faces and get board edges
  if (sample->face.size() == 0) 
  {
    std::cout<< "Error: face num 0" <<std::endl;
    return;
  }

  std::vector<MyBoarderEdge> v_edge;
  std::vector<MyBoarderEdge> v_border_edge;

  FaceIter fi;
  int n_edges = 0;
  for (fi = sample->face.begin(); fi != sample->face.end(); ++fi)
    if (!(*fi).IsD())
      n_edges += (*fi).VN();

  v_edge.resize(n_edges);

  MyEdgeIter edge_iter = v_edge.begin();
  FaceIter face_iter = sample->face.begin();
  FaceIter face_end = sample->face.end();
  for (; face_iter != face_end; ++face_iter)
  {
    if (!(*face_iter).IsD())
    {
      for (int j = 0; j < (*face_iter).VN(); ++j)
      {
        (*edge_iter).Set(&(*face_iter), j);
        (*face_iter).ClearB(j);
        ++edge_iter;
      }
    }
  }
  assert(edge_iter == v_edge.end());
  sort(v_edge.begin(), v_edge.end());
  
  //core part, find the board edges
  MyEdgeIter e_start, e_end;
  e_start = v_edge.begin();
  e_end = v_edge.begin();
  do 
  {
    if (e_end == v_edge.end() || *e_end != *e_start)
    {
      if (e_end - e_start == 1)
      {
        e_start->f->SetB(e_start->z);//found a board edge
        v_border_edge.push_back(*e_start);
      }else if (e_end - e_start != 2)
      {
        for (; e_start != e_end; ++e_start)
        {
          e_start->f->SetB(e_start->z);//found a board edge, set the z-th point of the face of e_start as board point
          v_border_edge.push_back(*e_start);
        }
      }
      e_start = e_end;
    }
    if (e_end == v_edge.end()) break;
    ++e_end;
  } while (true);

  std::cout<<"board face num: " <<tri::UpdateSelection<CMesh>::FaceFromBorderFlag(*sample) <<std::endl;
  std::cout<<"total edge size: " << v_edge.size() <<std::endl;
  std::cout<<"board edge num: " <<v_border_edge.size() << std::endl;

  sort(v_border_edge.begin(), v_border_edge.end());
  std::vector<Boundary> v_boundary = getBoundary(v_border_edge);

  //way2: using face topology
  /*
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

void PVSBasedNBV::runSearchNewBoundaries()
{
  //copy point from original to sample
  GlobalFun::clearCMesh(*sample); 
  int idx = 0;
  for (int i = 0; i < original->vert.size(); ++i)
  {
    CVertex v = original->vert[i];
    v.m_index = idx++;
    v.is_original = false;
    //v.is_fixed_sample = true;
    sample->vert.push_back(v);
    sample->bbox.Add(v.P());
  }
  sample->vn = sample->vert.size();

  //compute the topology of sample points
  GlobalFun::ballPivotingReconstruction(*sample);
  //use vertex topology
  //mark the border vertexes
  vcg::tri::UpdateFlags<CMesh>::VertexBorderFromNone(*sample); 
  //select the border vertexes
  std::cout<<"selected boarder points: " <<tri::UpdateSelection<CMesh>::VertexFromBorderFlag(*sample) << std::endl;
  //print them out
  for (int i = 0; (i < sample->vert.size()); ++i)
    if(sample->vert[i].IsB())
      sample->vert[i].is_boundary = true;

  //clear former boundaries and search for new ones
  m_v_boundaries->clear();
  searchNewBoundaries();

  for (int i = 0; i < m_v_boundaries->size(); ++i)
    std::cout<<"boundaries " <<i <<" "<< (*m_v_boundaries)[i].getSize() <<std::endl;
}

void PVSBasedNBV::runComputeCandidates()
{
  double step_aside_size = 0.2; //global_paraMgr.wLop.getDouble("CGrid Radius");
  double camera_max_dist = global_paraMgr.camera.getDouble("Camera Far Distance") /
    global_paraMgr.camera.getDouble("Predicted Model Size");

  GlobalFun::clearCMesh(*nbv_candidates);
  for (int i = 0; i < m_v_boundaries->size(); ++i)
  {
    Boundary b = m_v_boundaries->at(i);
    int mid_curve_idx = b.curve.size() / 2;
    int cmesh_index = b.curve[mid_curve_idx].m_index;
    //corresponding point in CMesh
    CVertex &v = sample->vert[cmesh_index];
    //find one point which is not a board point
    CVertex inside_point;
    for (int j = 0; j < v.neighbors.size(); ++j)
    {
      CVertex &n = sample->vert[v.neighbors[j]];
      if (n.IsB()) continue;
      else { inside_point = n; break;}
    }
    
    Point3f boder_dir = b.curve[mid_curve_idx].P() - b.curve[mid_curve_idx - 1].P();
    Point3f mid_point_normal = b.curve[mid_curve_idx].N();
    Point3f step_aside_direction = (boder_dir ^ mid_point_normal).Normalize();//cross product, remember always normalize

    Point3f inside_direction = inside_point.P() - b.curve[mid_curve_idx].P();
    if (step_aside_direction * inside_direction > 0)
      step_aside_direction = -step_aside_direction;

    //clear candidates
    CVertex candidate = v;
    candidate.m_index = i;
    candidate.P() = v.P() + step_aside_direction * step_aside_size;
    nbv_candidates->vert.push_back(candidate);
    nbv_candidates->bbox.Add(candidate.P());
  }
  nbv_candidates->vn = nbv_candidates->vert.size();

  std::cout<< "nbv candidates size: "<< nbv_candidates->vert.size() <<std::endl;

  //adjust the candidates according to the poisson surface
  if (iso_points->vert.empty())
  {
    std::cout<<"ERROR: Iso points empty!" <<std::endl;
    return;
  }
  std::cout<<iso_points->vert.size() << std::endl;
  GlobalFun::computeAnnNeigbhors(iso_points->vert, nbv_candidates->vert, 5, false, "PVS search nearest point on poisson surface to the given coarse candidates");
  
  for (int i = 0; i < nbv_candidates->vert.size(); ++i)
  {
    nbv_candidates->vert[i] = iso_points->vert[nbv_candidates->vert[i].neighbors[0]];
    // move outward in the normal direction and reverse the normal to get the nbv direction
    nbv_candidates->vert[i].P() = nbv_candidates->vert[i].P() + nbv_candidates->vert[i].N() * camera_max_dist;
    nbv_candidates->vert[i].N() = -nbv_candidates->vert[i].N();
  }

}

void PVSBasedNBV::runSelectCandidate()
{
  
}

std::vector<Boundary> PVSBasedNBV::getBoundary(std::vector<MyBoarderEdge> &v_border_edge)
{
  // store all boundaries;
  std::vector<Boundary> v_boundary;
  std::vector<MyBoarderEdge>::iterator iter = v_border_edge.begin();
  for (; iter != v_border_edge.end(); ++iter)
  {
    /* Boundary b;
    MyEdge e = *iter;
    b.v_board_edges.push_back(e);*/
    //std::vector<MyEdge>::iterator iter2 = ++iter;
    //for (; iter2 != v_border_edge.end(); ++iter2)
    //{
    //  //if they are connected
    //  if (iter->v[0]->P() == iter2->v[0]->P()
    //    ||iter->v[0]->P() == iter2->v[1]->P()
    //    ||iter->v[1]->P() == iter2->v[0]->P()
    //    ||iter->v[1]->P() == iter2->v[1]->P())
    //  {

    //  }
    //This is important: ways to access board points
    std::cout<< (iter->v[0]->P()[0])<<" " << (iter->v[0]->P()[1]) <<" " << (iter->v[0]->P()[2]) <<"     ";
    std::cout<< (iter->v[1]->P()[0])<<" " << (iter->v[1]->P()[1]) <<" " << (iter->v[1]->P()[2]) <<std::endl;
  }

  return v_boundary;
}

void PVSBasedNBV::searchNewBoundaries()
{
  int boundary_knn = para->getInt("Boundary Search KNN");
  GlobalFun::computeAnnNeigbhors(sample->vert, sample->vert, boundary_knn, false, "void PVSBasedNBV::search New Boundaies");
  
  std::cout<<"sample neighbors: " <<sample->vert[0].neighbors.size() <<std::endl;

  while (true)
  {
    //make sure boundary search begin at boundary point
    int begin_idx = -1;
    for (int i = 0; i < sample->vert.size(); ++i)
    {
      if (sample->vert[i].is_boundary) 
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
        sample->vert[new_boundary.curve[j].m_index].is_boundary = false;//mark the point have been handled
      }
      m_v_boundaries->push_back(new_boundary);
    }
  }

  std::cout<< m_v_boundaries->size() <<" boundaries detected!" <<std::endl;
}

Boundary PVSBasedNBV::searchOneBoundaryFromIndex(int begin_idx)
{
  Boundary new_boundary;
  CVertex& begin_vert = sample->vert[begin_idx];
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
    CVertex& t = sample->vert[begin_vert.neighbors[i]];
    if (t.is_boundary)
    {
      nearest_idx = begin_vert.neighbors[i];
      break;
    }
  }

  if (nearest_idx < 0) return new_boundary;

  CVertex &t = sample->vert[nearest_idx];
  Point3f head_direction = (t.P() - begin_vert.P()).Normalize();
  Boundary boundary_first_part = searchOneBoundaryFromDirection(begin_idx, head_direction);
  Boundary boundary_second_part = searchOneBoundaryFromDirection(begin_idx, -head_direction);

  std::cout<<"first half boundary size: "<< boundary_first_part.getSize() << std::endl;
  std::cout<<"second half boundary size: " << boundary_second_part.getSize() <<std::endl;

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
    CVertex curr_vertex = sample->vert[curr_idx];
    new_boundary.pushBackCVertex(curr_vertex);
    sample->vert[curr_idx].is_boundary = false;

    Point3f new_direction;
    int next_idx = -1;
    for (int i = 0; i < curr_vertex.neighbors.size(); ++i)
    {
      CVertex &t = sample->vert[curr_vertex.neighbors[i]];

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

void PVSBasedNBV::runBuildPVS()
{
  if (model->vert.empty())
  {
    std::cout<< " Model Empty!" <<std::endl;
    return;
  }

  GlobalFun::clearCMesh(*pvs);
  Point3f bbox_max = model->bbox.max;
  Point3f bbox_min = model->bbox.min;
  //get the whole 3D space that a camera may exist
  double camera_max_dist = global_paraMgr.camera.getDouble("Camera Far Distance") /
    global_paraMgr.camera.getDouble("Predicted Model Size");
  double pvs_resolution = global_paraMgr.pvsBasedNBV.getDouble("PVS Grid Resolution");
  assert(pvs_resolution > 2);
  double grid_step_size = (camera_max_dist*2.0 + 1.0) / (pvs_resolution - 1);

  float pvs_box_size = camera_max_dist + 0.5;
  Point3f whole_space_box_min = Point3f(-pvs_box_size, -pvs_box_size, -pvs_box_size);
  Point3f whole_space_box_max = Point3f(pvs_box_size, pvs_box_size, pvs_box_size);
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
  int x_max = static_cast<int> (dif.X() / grid_step_size);
  int y_max = static_cast<int> (dif.Y() / grid_step_size);
  int z_max = static_cast<int> (dif.Z() / grid_step_size);

  int all_max = std::max(std::max(x_max, y_max), z_max);
  x_max = y_max =z_max = all_max+1; // wsh 12-11
  //preallocate the memory
  int max_index = x_max * y_max * z_max;
  pvs->vert.resize(max_index);
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
        t.m_index = index;
        t.is_pvs = true;
        pvs->vert[index] = t;
        pvs->bbox.Add(t.P());
      }
    }
  }
  pvs->vn = max_index;
  cout << "all grid points: " << max_index << endl;
  cout << "resolution: " << x_max << endl;
}