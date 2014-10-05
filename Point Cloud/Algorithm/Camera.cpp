#include "Camera.h"

vcc::Camera::Camera(RichParameterSet* _para)
{
  para = _para;
}

void vcc::Camera::setInput(DataMgr* pData)
{
  if (!pData->isModelEmpty())
  {
    target = pData->getCurrentModel();
    original = pData->getCurrentOriginal();
    //scan candidates for initialing
    scan_count = pData->getScanCount();
    init_scan_candidates = pData->getInitCameraScanCandidates();
    visibility_first_scan_candidates = pData->getVisibilityFirstScanCandidates();
    pvs_first_scan_candidates = pData->getPVSFirstScanCandidates();
    //candidates for nbv computing
    scan_candidates = pData->getScanCandidates();
    scan_history = pData->getScanHistory();
    current_scanned_mesh = pData->getCurrentScannedMesh();
    scanned_results = pData->getScannedResults();
    nbv_candidates = pData->getNbvCandidates();

    far_horizon_dist = global_paraMgr.camera.getDouble("Camera Horizon Dist") 
      / global_paraMgr.camera.getDouble("Predicted Model Size");
    far_vertical_dist = global_paraMgr.camera.getDouble("Camera Vertical Dist")
      / global_paraMgr.camera.getDouble("Predicted Model Size");

    far_distance = global_paraMgr.camera.getDouble("Camera Far Distance") 
      / global_paraMgr.camera.getDouble("Predicted Model Size");
    near_distance = global_paraMgr.camera.getDouble("Camera Near Distance")
      / global_paraMgr.camera.getDouble("Predicted Model Size");

    dist_to_model = global_paraMgr.camera.getDouble("Camera Dist To Model")
      / global_paraMgr.camera.getDouble("Predicted Model Size");

    resolution = global_paraMgr.camera.getDouble("Camera Resolution");
  }else
  {
    cout<<"ERROR: Camera::setInput empty!!" << endl;
    return;
  }
}

void vcc::Camera::run()
{
  if (para->getBool("Run Virtual Scan"))
  {
    runVirtualScan();
    return ;
  }
  if (para->getBool("Run Initial Scan"))
  {
    runInitialScan();
    return ;
  }
  if (para->getBool("Run NBV Scan"))
  {
    runNBVScan();
    return;
  }
  if (para->getBool("Run One Key NewScans"))
  {
    runOneKeyNewScan();
    return;
  }
  if (para->getBool("Run Visibility First Scan"))
  {
    runVisibilityFirstScan();
    return;
  }
  if (para->getBool("Run PVS First Scan"))
  {
    runPVSFirstScan();
    return;
  }
}

void vcc::Camera::runVirtualScan()
{
  //point current_scanned_mesh to a new address
  current_scanned_mesh = new CMesh;
  double max_displacement = resolution * 0.0f; //8.0f;//global_paraMgr.nbv.getDouble("Max Displacement"); //resolution * 2; //for adding noise
  computeUpAndRight();
  Point3f viewray = direction.Normalize();
  //compute the end point of viewray
  Point3f viewray_end = pos + viewray * far_distance;

  //sweep and scan
  int n_point_hr_half  = static_cast<int>(0.5 * far_horizon_dist / resolution);
  int n_point_ver_half = static_cast<int>(0.5 * far_vertical_dist / resolution);
  int index = 0; 
  for (int i = - n_point_hr_half; i < n_point_hr_half; ++i)
  {
    double i_res = i * resolution;
    for (int j = - n_point_ver_half; j < n_point_ver_half; ++j)
    {
      Point3f viewray_end_iter = viewray_end + right * i_res + up * (j * resolution);
      Point3f viewray_iter = viewray_end_iter - pos;
      //line direction vector
      Point3f line_dir = viewray_iter.Normalize();
      Point3f intersect_point;
      Point3f intersect_point_normal;
      bool is_barely_visible = false;
      double dist = GlobalFun::computeMeshLineIntersectPoint(target, pos, line_dir, intersect_point, intersect_point_normal, is_barely_visible);
      if ( dist <= far_distance && dist >= near_distance)
      {
        //add some random noise
        //srand(time(NULL)); 
        double rndax = (double(2.0f * rand()) / RAND_MAX - 1.0f ) * max_displacement;
        double rnday = (double(2.0f * rand()) / RAND_MAX - 1.0f ) * max_displacement;
        double rndaz = (double(2.0f * rand()) / RAND_MAX - 1.0f ) * max_displacement;

        CVertex t;
        t.is_scanned = true;
        t.is_barely_visible= is_barely_visible;
         
        t.m_index = index++;
        t.P() = intersect_point + Point3f(rndax, rnday, rndaz);//noise 1
        t.N() = intersect_point_normal; //set out direction as approximate normal
        current_scanned_mesh->vert.push_back(t);
        current_scanned_mesh->bbox.Add(t.P());
      }
    }
  }
  current_scanned_mesh->vn = current_scanned_mesh->vert.size();
  //remove outlier
  double outlier_percentage = 0.02f;//global_paraMgr.wLop.getDouble("Outlier Percentage");
  std::cout<<"Outlier percentage: " <<outlier_percentage <<endl;
  GlobalFun::removeOutliers(current_scanned_mesh, 0.03, outlier_percentage);
  cout<<"Has removed samples outliers."<<endl;
  //noise 2 from trimesh2
  //double noise_size = global_paraMgr.camera.getDouble("Camera Resolution") * 1.0f;
  //GlobalFun::addNoise(current_scanned_mesh, noise_size);

  //noise 3 from benchmark
  GlobalFun::addBenchmarkNoise(current_scanned_mesh, this->pos, viewray, 0.75f);

  //increase the scan count;
  (*scan_count)++;
  std::cout<<"scan count right after virtual scan: "<<*scan_count <<std::endl;
}

void vcc::Camera::runInitialScan()
{
  //clear original points
  GlobalFun::clearCMesh(*original);

  //release scanned_result
  vector<CMesh* >::iterator it_scanned_result = scanned_results->begin();
  for (; it_scanned_result != scanned_results->end(); ++it_scanned_result)
  {
    if ( (*it_scanned_result) != NULL)
    {
      delete (*it_scanned_result);
      (*it_scanned_result) = NULL;
    }
  }
  scanned_results->clear();

  //release scan history
  scan_history->clear();

  //run initial scan
  vector<ScanCandidate>::iterator it = init_scan_candidates->begin();
  int i = 1;
  for (; it != init_scan_candidates->end(); ++it)
  {
    pos = it->first;
    direction = it->second;
    /******* call runVirtualScan() *******/
    cout<<i << "th initial scan begin" <<endl;
    runVirtualScan();
    cout<<i++ <<"th initial scan done!" <<endl;

    scan_history->push_back(*it);

    //merge scanned mesh with original
    int index = 0;
    if (!original->vert.empty()) index = original->vert.back().m_index + 1;

    for (int i = 0; i < current_scanned_mesh->vert.size(); ++i)
    {
      CVertex& v = current_scanned_mesh->vert[i];
      CVertex t = v;
      t.m_index = index++;
      t.is_original = true;
      original->vert.push_back(t);
      original->bbox.Add(t.P());
    }
    original->vn = original->vert.size();
  }

}

void vcc::Camera::runNBVScan()
{
  //release scanned_result
  vector< CMesh* >::iterator it_scanned_result = scanned_results->begin();
  for (; it_scanned_result != scanned_results->end(); ++it_scanned_result)
  {
    if ( (*it_scanned_result) != NULL)
    {
      delete (*it_scanned_result);
      (*it_scanned_result) = NULL;
    }
  }
  scanned_results->clear();

  //traverse the scan_candidates and do virtual scan
  vector<ScanCandidate>::iterator it = scan_candidates->begin();
  cout<<"scan candidates size: " <<scan_candidates->size() <<endl;
  int i = 1;
  for (; it != scan_candidates->end(); ++it)
  {
    pos = it->first;
    direction = it->second;
    /********* call runVirtualScan() *******/
    cout<< i << "th candidate Begins!" <<endl;
    runVirtualScan();
    cout<< i++ << "th candidate Ends!" <<endl;

    scan_history->push_back(*it);
    scanned_results->push_back(current_scanned_mesh);
    cout << "scanned points:  " << current_scanned_mesh->vert.size() << endl;
  }
}

void vcc::Camera::runOneKeyNewScan()
{
  runNBVScan();
}

void vcc::Camera::computeUpAndRight()
{
  Point3f x_axis(1.0f, 0.0f, 0.0f);
  Point3f z_axis(0.0f, 0.0f, 1.0f);

  Point3f viewray = direction.Normalize();
  if (viewray.Z() > 0)
  {
    up = viewray ^ x_axis;
  }else if (fabs(viewray.Z()) < EPS_SUN)
  {
    up = viewray ^ z_axis;
  }else
  {
    up = x_axis ^ viewray;
  }
  //compute the right vector
  right = viewray ^ up;

  up = up.Normalize();
  right = right.Normalize();
}

void vcc::Camera::runVisibilityFirstScan()
{
  //clear original points
  GlobalFun::clearCMesh(*original);

  //release scanned_result
  vector<CMesh* >::iterator it_scanned_result = scanned_results->begin();
  for (; it_scanned_result != scanned_results->end(); ++it_scanned_result)
  {
    if ( (*it_scanned_result) != NULL)
    {
      delete (*it_scanned_result);
      (*it_scanned_result) = NULL;
    }
  }
  scanned_results->clear();

  //release scan history
  scan_history->clear();

  //run visibility first scan
  vector<ScanCandidate>::iterator it = visibility_first_scan_candidates->begin();
  int i = 1;
  for (; it != visibility_first_scan_candidates->end(); ++it)
  {
    pos = it->first;
    direction = it->second;
    /******* call runVirtualScan() *******/
    cout<<i << "th initial scan begin" <<endl;
    runVirtualScan();
    cout<<i <<"th initial scan done!" <<endl;

    //add to scan history
    scan_history->push_back(*it);

    //merge scanned mesh with original
    int index = 0;
    if (!original->vert.empty()) index = original->vert.back().m_index + 1;

    std::cout<< i++ <<"th initial scan points num: " <<current_scanned_mesh->vert.size() << std::endl;
    for (int j = 0; j < current_scanned_mesh->vert.size(); ++j)
    {
      CVertex& v = current_scanned_mesh->vert[j];
      CVertex t = v;
      t.m_index = index++;
      t.is_original = true;
      original->vert.push_back(t);
      original->bbox.Add(t.P());
    }
    original->vn = original->vert.size();
  }
  
  int knn = global_paraMgr.norSmooth.getInt("PCA KNN");
  GlobalFun::computePCANormal(original, knn);
}

void vcc::Camera::runPVSFirstScan()
{
  //clear original points
  GlobalFun::clearCMesh(*original);

  //release scanned_result
  vector<CMesh* >::iterator it_scanned_result = scanned_results->begin();
  for (; it_scanned_result != scanned_results->end(); ++it_scanned_result)
  {
    if ( (*it_scanned_result) != NULL)
    {
      delete (*it_scanned_result);
      (*it_scanned_result) = NULL;
    }
  }
  scanned_results->clear();

  //release scan history
  scan_history->clear();

  vector<ScanCandidate>::iterator it = pvs_first_scan_candidates->begin();
  int i = 1;
  for (; it != pvs_first_scan_candidates->end(); ++it)
  {
    pos = it->first;
    direction = it->second;
    /******* call runVirtualScan() *******/
    cout<<i << "th initial scan begin" <<endl;
    runVirtualScan();
    cout<<i <<"th initial scan done!" <<endl;
    
    //add to scan history
    scan_history->push_back(*it);
    //merge scanned mesh with original
    int index = 0;
    if (!original->vert.empty()) index = original->vert.back().m_index + 1;

    std::cout<< i++ <<"th initial scan points num: " <<current_scanned_mesh->vert.size() << std::endl;
    for (int j = 0; j < current_scanned_mesh->vert.size(); ++j)
    {
      CVertex t = current_scanned_mesh->vert[j];
      t.m_index = index++;
      t.is_original = true;
      original->vert.push_back(t);
      original->bbox.Add(t.P());
    }
    original->vn = original->vert.size();
    //add scanned points to scanned_resultd for computing pvs grid value
    scanned_results->push_back(current_scanned_mesh);
  }
}