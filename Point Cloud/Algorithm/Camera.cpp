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
     init_scan_candidates = pData->getInitCameraScanCandidates();
     //candidates for nbv computing
     scan_candidates = pData->getScanCandidates();
     current_scanned_mesh = pData->getCurrentScannedMesh();
     scanned_results = pData->getScannedResults();
	   nbv_candidates = pData->getNbvCandidates();

     far_horizon_dist = global_paraMgr.camera.getDouble("Camera Horizon Dist");
     far_vertical_dist = global_paraMgr.camera.getDouble("Camera Vertical Dist");

     far_distance = global_paraMgr.camera.getDouble("Camera Far Distance") /
                    global_paraMgr.camera.getDouble("Predicted Model Size");
     near_distance = global_paraMgr.camera.getDouble("Camera Near Distance");

     dist_to_model = global_paraMgr.camera.getDouble("Camera Dist To Model");
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
}

void vcc::Camera::runVirtualScan()
{
  //point current_scanned_mesh to a new address
  current_scanned_mesh = new CMesh;
  double max_displacement = resolution / 2.0f; //for adding noise
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
    for (int j = - n_point_ver_half; j < n_point_ver_half; ++j)
    {
      Point3f viewray_end_iter = viewray_end + right * (i * resolution) + up * (j * resolution);
      Point3f viewray_iter = viewray_end_iter - pos;
      //line direction vector
      Point3f line_dir = viewray_iter.Normalize();
      Point3f intersect_point;
      if (GlobalFun::computeMeshLineIntersectPoint(target, pos, line_dir, intersect_point) <= far_distance)
      {        
        //add some random noise
        srand(time(NULL)); 
        double rndax = (double(2.0f * rand()) / RAND_MAX - 1.0f ) * max_displacement;
        double rnday = (double(2.0f * rand()) / RAND_MAX - 1.0f ) * max_displacement;
        double rndaz = (double(2.0f * rand()) / RAND_MAX - 1.0f ) * max_displacement;
        
        CVertex t;
        t.is_scanned = true;
        t.m_index = index++;
        t.P()[0] = intersect_point.X() + rndax;
        t.P()[1] = intersect_point.Y() + rnday;
        t.P()[2] = intersect_point.Z() + rndaz;
        current_scanned_mesh->vert.push_back(t);
        current_scanned_mesh->bbox.Add(t.P());
      }
    }
  current_scanned_mesh->vn = current_scanned_mesh->vert.size();
  }
}

void vcc::Camera::runInitialScan()
{
  //clear original points
  original->face.clear();
  original->fn = 0;
  original->vert.clear();
  original->vn = 0;
  original->bbox = Box3f();

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

  //run initial scan
  vector<ScanCandidate>::iterator it = init_scan_candidates->begin();
  for (; it != init_scan_candidates->end(); ++it)
  {
    //init scan should consider dist to model
    pos = it->first * dist_to_model;
    direction = it->second;
    /********* call runVirtualScan() *******/
    runVirtualScan();

    //merge scanned mesh with original
    int index = original->vert.size();
    for (int i = 0; i < current_scanned_mesh->vert.size(); ++i)
    {
      CVertex& v = current_scanned_mesh->vert[i];
      CVertex t;
      t.m_index = ++index;
      t.is_original = true;
      t.P()[0] = v.P()[0];
      t.P()[1] = v.P()[1];
      t.P()[2] = v.P()[2];
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
  for (; it != scan_candidates->end(); ++it)
  {
    pos = it->first;
    direction = it->second;
    /********* call runVirtualScan() *******/
    runVirtualScan();

    scanned_results->push_back(current_scanned_mesh);
  }
}

void vcc::Camera::runOneKeyNewScan()
{
	runNBVScan();

	//merge with original points
  int index = original->vert.back().m_index;
	vector<CMesh*>::iterator it_scan_result = scanned_results->begin();
	for (; it_scan_result != scanned_results->end(); ++it_scan_result)
	{
		CMesh *r = *it_scan_result;
		for (int i = 0; i < r->vert.size(); ++i)
		{
      r->vert[i].is_original = true;
      r->vert[i].m_index = ++index;
			original->vert.push_back(r->vert[i]);
		}
	}
}

void vcc::Camera::computeUpAndRight()
{
  Point3f x_axis(1.0f, 0.0f, 0.0f);
  Point3f z_axis(0.0f, 0.0f, 1.0f);

  Point3f viewray = direction.Normalize();
  if (viewray.Z() > 0)
  {
    up = viewray ^ x_axis;
  }else if (fabs(viewray.Z()) < EPI)
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