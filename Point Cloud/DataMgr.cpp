#include "DataMgr.h"


DataMgr::DataMgr(RichParameterSet* _para)
{
	para = _para;
  camera_pos = Point3f(0.0f, 0.0f, 1.0f);
  camera_direction = Point3f(0.0f, 0.0f, -1.0f);

  initDefaultScanCamera();

  whole_space_box.Add(Point3f(2.0, 2.0, 2.0));
  whole_space_box.Add(Point3f(-2.0, -2.0, -2.0));

  loadCommonTransform();
  slices.assign(3, Slice());
}

DataMgr::~DataMgr(void)
{

}

void DataMgr::clearCMesh(CMesh& mesh)
{
	mesh.face.clear();
	mesh.fn = 0;
	mesh.vert.clear();
	mesh.vn = 0;
	mesh.bbox = Box3f();
}

void DataMgr::initDefaultScanCamera()
{
  //default cameras for initial scanning, pair<pos, direction>
  //x axis
  init_scan_candidates.push_back(make_pair(Point3f(1.0f, 0.0f, 0.0f), Point3f(-1.0f, 0.0f, 0.0f)));
  init_scan_candidates.push_back(make_pair(Point3f(-1.0f, 0.0f, 0.0f), Point3f(1.0f, 0.0f, 0.0f)));
  //y axis
  init_scan_candidates.push_back(make_pair(Point3f(0.0f, 0.0f, 1.0f), Point3f(0.0f, 0.0f, -1.0f)));
  init_scan_candidates.push_back(make_pair(Point3f(0.0f, 0.0f, -1.0f), Point3f(0.0f, 0.0f, 1.0f)));
  //z axis
  /*init_scan_candidates.push_back(make_pair(Point3f(0.0f, 0.0f, 1.0f), Point3f(0.0f, 0.0f, -1.0f)));
  init_scan_candidates.push_back(make_pair(Point3f(0.0f, 0.0f, -1.0f), Point3f(0.0f, 0.0f, 1.0f)));*/

  //this should be deleted, for UI debug
  //for test
  scan_candidates.push_back(make_pair(Point3f(0.0f, 0.0f, 1.0f), Point3f(0.0f, 0.0f, -1.0f)));
  //x axis
  scan_candidates.push_back(make_pair(Point3f(1.0f, 0.0f, 0.0f), Point3f(-1.0f, 0.0f, 0.0f)));
  //scan_candidates.push_back(make_pair(Point3f(-1.0f, 0.0f, 0.0f), Point3f(1.0f, 0.0f, 0.0f)));
  ////y axis
  //scan_candidates.push_back(make_pair(Point3f(0.0f, 1.0f, 0.0f), Point3f(0.0f, -1.0f, 0.0f)));
  //scan_candidates.push_back(make_pair(Point3f(0.0f, -1.0f, 0.0f), Point3f(0.0f, 1.0f, 0.0f)));
  ////z axis
  //scan_candidates.push_back(make_pair(Point3f(0.0f, 0.0f, 1.0f), Point3f(0.0f, 0.0f, -1.0f)));
  //scan_candidates.push_back(make_pair(Point3f(0.0f, 0.0f, -1.0f), Point3f(0.0f, 0.0f, 1.0f)));
}

bool DataMgr::isSamplesEmpty()
{
	return samples.vert.empty();
}

bool DataMgr::isModelEmpty()
{
  return model.vert.empty();
}

bool DataMgr::isOriginalEmpty()
{
	return original.vert.empty();
}

bool DataMgr::isSkeletonEmpty()
{
  return skeleton.isEmpty();
}

bool DataMgr::isIsoPointsEmpty()
{
  return iso_points.vert.empty();
}

bool DataMgr::isFieldPointsEmpty()
{
  return field_points.vert.empty();
}

bool DataMgr::isScannedMeshEmpty()
{
  return current_scanned_mesh.vert.empty();
}

bool DataMgr::isScannedResultsEmpty()
{
  return scanned_results.empty();
}

bool DataMgr::isViewGridsEmpty()
{
  return view_grid_points.vert.empty();
}

bool DataMgr::isNBVCandidatesEmpty()
{
  return nbv_candidates.vert.empty();
}

void DataMgr::loadPlyToModel(QString fileName)
{
  clearCMesh(model);
  curr_file_name = fileName;

  int mask = tri::io::Mask::IOM_ALL;
  int err = tri::io::Importer<CMesh>::Open(model, curr_file_name.toAscii().data(), mask);
  if (err)
  {
    cout<<"Failed to read model: "<< err <<"\n";
    return;
  }
  cout<<"object model loaded \n";

  CMesh::VertexIterator vi;
  int idx = 0;
  for (vi = model.vert.begin(); vi != model.vert.end(); ++vi)
  {
    vi->is_model = true;
    vi->m_index = idx++;
    model.bbox.Add(vi->P());
  }
  model.vn = model.vert.size();
}

void DataMgr::loadPlyToOriginal(QString fileName)
{
	clearCMesh(original);
	curr_file_name = fileName;

	int mask= tri::io::Mask::IOM_VERTCOORD + tri::io::Mask::IOM_VERTNORMAL ;

	int err = tri::io::Importer<CMesh>::Open(original, curr_file_name.toAscii().data(), mask);  
	if(err) 
	{
		cout << "Failed reading mesh: " << err << "\n";
		return;
	}  
	cout << "points loaded\n";

	CMesh::VertexIterator vi;
	int idx = 0;
	for(vi = original.vert.begin(); vi != original.vert.end(); ++vi)
	{
		vi->is_original = true;
		vi->m_index = idx++;
		//vi->N() = Point3f(0, 0, 0);
		original.bbox.Add(vi->P());
	}
	original.vn = original.vert.size();
}

void DataMgr::loadPlyToSample(QString fileName)
{
	clearCMesh(samples);
	curr_file_name = fileName;

	int mask= tri::io::Mask::IOM_VERTCOORD + tri::io::Mask::IOM_VERTNORMAL ;
	mask += tri::io::Mask::IOM_VERTCOLOR;
	mask += tri::io::Mask::IOM_BITPOLYGONAL;

	int err = tri::io::Importer<CMesh>::Open(samples, curr_file_name.toAscii().data(), mask);  
	if(err) 
	{
		cout << "Failed reading mesh: " << err << "\n";
		return;
	}  

	CMesh::VertexIterator vi;
	int idx = 0;
	for(vi = samples.vert.begin(); vi != samples.vert.end(); ++vi)
	{
		vi->is_original = false;
		vi->m_index = idx++;
		samples.bbox.Add(vi->P());
	}
	samples.vn = samples.vert.size();
}

void DataMgr::loadPlyToISO(QString fileName)
{
  clearCMesh(iso_points);
  curr_file_name = fileName;

  int mask= tri::io::Mask::IOM_VERTCOORD + tri::io::Mask::IOM_VERTNORMAL ;
  mask += tri::io::Mask::IOM_VERTCOLOR;
  mask += tri::io::Mask::IOM_BITPOLYGONAL;

  int err = tri::io::Importer<CMesh>::Open(iso_points, curr_file_name.toAscii().data(), mask);  
  if(err) 
  {
    cout << "Failed reading mesh: " << err << "\n";
    return;
  }  

  CMesh::VertexIterator vi;
  int idx = 0;
  for(vi = iso_points.vert.begin(); vi != iso_points.vert.end(); ++vi)
  {
    vi->is_iso = true;
    vi->m_index = idx++;
    iso_points.bbox.Add(vi->P());
  }
  iso_points.vn = iso_points.vert.size();
}

void DataMgr::loadXYZN(QString fileName)
{
  clearCMesh(samples);
  ifstream infile;
  infile.open(fileName.toStdString().c_str());

  int i = 0;
  while(!infile.eof())
  {
    CVertex v;
    float temp = 0.;
    for (int j=0; j<3; j++)
    {

      infile >> temp;
      v.P()[j] = temp;
    }


    for (int j=0; j<3; j++) {
      infile >> v.N()[j];
    }

    v.m_index = i++;

    samples.vert.push_back(v);
    samples.bbox.Add(v.P());
  }

 // mesh.vert.erase(mesh.vert.end()-1);
  samples.vert.pop_back();
  samples.vn = samples.vert.size();

  infile.close();



}

void DataMgr::loadImage(QString fileName)
{

	//image = cv::imread(fileName.toAscii().data());

	////cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
	////cv::imshow("image", image);

	//clearCMesh(samples);
	//clearCMesh(original);
	//int cnt = 0;
	//for (int i = 0; i < image.rows; i++)
	//{
	//	for (int j = 0; j < image.cols; j++)
	//	{
	//		cv::Vec3b intensity = image.at<cv::Vec3b>(i, j);
	//		Point3f p;
	//		Color4b c;
	//		c.Z() = 1;
	//		p.X() = c.X() = intensity.val[0];
	//		p.Y() = c.Y() = intensity.val[1];
	//		p.Z() = c.Z() = intensity.val[2];
	//		CVertex new_v;
	//		new_v.P() = p;
	//		new_v.C() = c;
	//		new_v.m_index = cnt++;

	//		samples.vert.push_back(new_v);
	//		samples.bbox.Add(p);

	//		new_v.is_original = true;
	//		original.vert.push_back(new_v);
	//		original.bbox.Add(p);
	//	}
	//}
	//samples.vn = samples.vert.size();
	//original.vn = samples.vn;

	//cv::waitKey();

}

void DataMgr::loadCameraModel(QString fileName)
{
  clearCMesh(camera_model);
  curr_file_name = fileName;
  int mask = tri::io::Mask::IOM_VERTCOORD + tri::io::Mask::IOM_VERTNORMAL;
  mask += tri::io::Mask::IOM_FACEFLAGS;

  int err = tri::io::Importer<CMesh>::Open(camera_model, curr_file_name.toAscii().data(), mask);
  if (err)
  {
    cout<<"Failed to read camera model: "<< err << "\n";
    return;
  }
  cout<<"camera model loaded \n";
}

void DataMgr::setCurrentTemperalSample(CMesh *mesh)
{
  this->temperal_sample = mesh;
}

CMesh* DataMgr::getCurrentIsoPoints()
{
  if(&iso_points == NULL)
  {
    return NULL;
  }

  if(iso_points.vert.empty())
  {
    return & iso_points;
  }

  return & iso_points;
}

CMesh* DataMgr::getCurrentFieldPoints()
{
  if(&field_points == NULL)
  {
    return NULL;
  }

  if(field_points.vert.empty())
  {
    return & field_points;
  }

  return & field_points;
}


CMesh* DataMgr::getCurrentSamples()
{
  if(&samples == NULL)
  {
    //cout << "DataMgr::getCurrentSamples samples = NULL!!" <<endl;
    return NULL;
  }

	if(samples.vert.empty())
	{
		//cout << "DataMgr::getCurrentSamples samples.vert.empty()!!" <<endl;
		//return NULL;
    return & samples;
	}

	return & samples;
}

CMesh* DataMgr::getCurrentTemperalSamples()
{
  return temperal_sample;
}

CMesh* DataMgr::getCurrentModel()
{
  return &model;
}

CMesh* DataMgr::getCurrentOriginal()
{
  if(&original == NULL)
  {
    //cout << "DataMgr::getCurrentOriginal() samples = NULL!!" <<endl;
    return NULL;
  }

	if(original.vert.empty())
	{
		//cout << "DataMgr::getCurrentOriginal() original.vert.empty()!!" <<endl;
		return & original;
	}

	return & original;
}

CMesh* DataMgr::getCurrentTemperalOriginal()
{
  return temperal_original;
}

Skeleton* DataMgr::getCurrentSkeleton()
{
	return &skeleton;
}

CMesh* DataMgr::getCameraModel()
{
  return &camera_model;
}

Point3f& DataMgr::getCameraPos()
{
  return camera_pos;
}

Point3f& DataMgr::getCameraDirection()
{
  return camera_direction;
}

double DataMgr::getCameraResolution()
{
  return camera_resolution;
}

double DataMgr::getCameraHorizonDist()
{
  return camera_horizon_dist;
}

double DataMgr::getCameraVerticalDist()
{
  return camera_vertical_dist;
}

double DataMgr::getCameraMaxDistance()
{
  return camera_max_distance;
}

double DataMgr::getCameraMaxAngle()
{
  return camera_max_angle;
}

CMesh*
DataMgr::getViewGridPoints()
{
  return &view_grid_points;
}

CMesh*
DataMgr::getNbvCandidates()
{
  return &nbv_candidates;
}

vector<ScanCandidate>* DataMgr::getInitCameraScanCandidates()
{
  return &init_scan_candidates;
}

vector<ScanCandidate>* DataMgr::getScanCandidates()
{
  return &scan_candidates;
}

vector<ScanCandidate>* DataMgr::getSelectedScanCandidates()
{
  return &selected_scan_candidates;
}

CMesh* DataMgr::getCurrentScannedMesh()
{
  return &current_scanned_mesh;
}

vector<CMesh* >* DataMgr::getScannedResults()
{
  return &scanned_results;
}

Slices* DataMgr::getCurrentSlices()
{
  return &slices;
}

void DataMgr::recomputeBox()
{
  model.bbox.SetNull();
	samples.bbox.SetNull();
	original.bbox.SetNull();

	CMesh::VertexIterator vi;
  for (vi = model.vert.begin(); vi != model.vert.end(); ++vi)
  {
    model.bbox.Add(vi->P());
  }

	for(vi = samples.vert.begin(); vi != samples.vert.end(); ++vi) 
	{
		if (vi->is_ignore)
		{
			continue;
		}
		samples.bbox.Add(vi->P());
	}

	for(vi = original.vert.begin(); vi != original.vert.end(); ++vi) 
	{
		original.bbox.Add(vi->P());
	}

  double camera_max_dist = global_paraMgr.camera.getDouble("Camera Far Distance") /
    global_paraMgr.camera.getDouble("Predicted Model Size"); 
  float scan_box_size = camera_max_dist + 0.5;

  Point3f whole_space_box_min = Point3f(-scan_box_size, -scan_box_size, -scan_box_size);
  Point3f whole_space_box_max = Point3f(scan_box_size, scan_box_size, scan_box_size);
  whole_space_box.SetNull();
  whole_space_box.Add(whole_space_box_min);
  whole_space_box.Add(whole_space_box_max);

}

double DataMgr::getInitRadiuse()
{
	double init_para = para->getDouble("Init Radius Para");
  if (isOriginalEmpty() && isModelEmpty())
  {
    global_paraMgr.setGlobalParameter("CGrid Radius", DoubleValue(init_radius));
    global_paraMgr.setGlobalParameter("Initial Radius", DoubleValue(init_radius));
    return init_radius;
  }

  Box3f box;
	if (!isOriginalEmpty())   box = original.bbox;
	else if (!isModelEmpty()) box = model.bbox;

  if ( abs(box.min.X() - box.max.X()) < 1e-5 ||   
    abs(box.min.Y() - box.max.Y()) < 1e-5 ||   
    abs(box.min.Z() - box.max.Z()) < 1e-5 )
  {
    double diagonal_length = sqrt((box.min - box.max).SquaredNorm());
    double original_size = sqrt(double(original.vn));
    init_radius = 2 * init_para * diagonal_length / original_size;
  }
  else
  {
    double diagonal_length = sqrt((box.min - box.max).SquaredNorm());
    double original_size = pow(double(original.vn), 0.333);
    init_radius = init_para * diagonal_length / original_size;
  }
 
  global_paraMgr.setGlobalParameter("CGrid Radius", DoubleValue(init_radius));
  global_paraMgr.setGlobalParameter("Initial Radius", DoubleValue(init_radius));

	return init_radius;
}

void DataMgr::downSamplesByNum(bool use_random_downsample)
{
	if (isOriginalEmpty() && !isSamplesEmpty())
	{
		subSamples();
		return;
	}

	if (isOriginalEmpty())
	{
		return;
	}

	int want_sample_num = para->getDouble("Down Sample Num");

	if (want_sample_num > original.vn)
	{
		want_sample_num = original.vn;
	}

	clearCMesh(samples);
	samples.vn = want_sample_num;

	vector<int> nCard = GlobalFun::GetRandomCards(original.vert.size());
	for(int i = 0; i < samples.vn; i++) 
	{
		int index = nCard[i]; //could be not random!

    if (!use_random_downsample)
    {
      index = i;
    }

		CVertex& v = original.vert[index];
		samples.vert.push_back(v);
		samples.bbox.Add(v.P());
	}

	CMesh::VertexIterator vi;
	for(vi = samples.vert.begin(); vi != samples.vert.end(); ++vi)
	{
		vi->is_original = false;
	}

  getInitRadiuse();
}

void DataMgr::subSamples()
{
	clearCMesh(original);

	CMesh::VertexIterator vi;
	original.vn = samples.vert.size();
	original.bbox.SetNull();
	for(vi = samples.vert.begin(); vi != samples.vert.end(); ++vi)
	{
		CVertex v = (*vi);
		v.is_original = true;
		original.vert.push_back(v);
		original.bbox.Add(v.P());
	}

	downSamplesByNum();
  getInitRadiuse();
}


void DataMgr::savePly(QString fileName, CMesh& mesh)
{
	int mask= tri::io::Mask::IOM_VERTNORMAL ;
	//mask += tri::io::Mask::IOM_VERTCOLOR;
  //mask += tri::io::Mask::IOM_ALL;
	//mask += tri::io::Mask::IOM_BITPOLYGONAL;
  //mask += tri::io::Mask::IOM_FACEINDEX;

	if (fileName.endsWith("ply"))
		tri::io::ExporterPLY<CMesh>::Save(mesh, fileName.toAscii().data(), mask, false);
}

void DataMgr::normalizeROSA_Mesh(CMesh& mesh, bool is_original)
{
  if (mesh.vert.empty()) return;

  mesh.bbox.SetNull();
  Box3f box = mesh.bbox;
  
  float max_length = global_paraMgr.data.getDouble("Max Normalize Length");
  
  Box3f box_temp;
  for(int i = 0; i < mesh.vert.size(); i++)
  {
    Point3f& p = mesh.vert[i].P();

    p /= max_length;

    mesh.vert[i].N().Normalize(); 
    box_temp.Add(p);
  }

  Point3f mid_point = (box_temp.min + box_temp.max) / 2.0;

  for(int i = 0; i < mesh.vert.size(); i++)
  {
    Point3f& p = mesh.vert[i].P();
    p -= mid_point;
    mesh.bbox.Add(p);
  }
  
  if (is_original)
  {
    this->original_center_point = mid_point;
  }
  
}


Box3f DataMgr::normalizeAllMesh()
{
  Box3f box;
  if (!isModelEmpty())
  {
    for (int i = 0; i < model.vert.size(); ++i)
      box.Add(model.vert[i].P());
  }

  if (!isSamplesEmpty())
  {
    for (int i = 0; i < samples.vert.size(); ++i)
      box.Add(samples.vert[i].P());
  }

  if (!isOriginalEmpty())
  {
    for (int i = 0; i < original.vert.size(); ++i)
      box.Add(original.vert[i].P());
  }

  model.bbox = box;
  original.bbox =box;
  samples.bbox = box;

  float max_x = abs((box.min - box.max).X());
  float max_y = abs((box.min - box.max).Y());
  float max_z = abs((box.min - box.max).Z());
  float max_length = std::max(max_x, std::max(max_y, max_z));
  global_paraMgr.data.setValue("Max Normalize Length", DoubleValue(max_length));

  normalizeROSA_Mesh(model);
  normalizeROSA_Mesh(original, true);
  normalizeROSA_Mesh(samples);
  normalizeROSA_Mesh(iso_points);

  recomputeBox();
  getInitRadiuse();

  return samples.bbox;
}


void DataMgr::eraseRemovedSamples()
{
	int cnt = 0;
	vector<CVertex> temp_mesh;
	for (int i = 0; i < samples.vert.size(); i++)
	{
		CVertex& v = samples.vert[i];
		if (!v.is_ignore)
		{
			temp_mesh.push_back(v);
		}
	}

	samples.vert.clear();
	samples.vn = temp_mesh.size();
	for (int i = 0; i < temp_mesh.size(); i++)
	{
		temp_mesh[i].m_index = i;
		samples.vert.push_back(temp_mesh[i]);
	}

}

void DataMgr::clearData()
{
	clearCMesh(original);
	clearCMesh(samples);
  clearCMesh(iso_points);
  clearCMesh(field_points);

  clearCMesh(model);  
  clearCMesh(current_scanned_mesh);

  clearCMesh(view_grid_points);
  clearCMesh(nbv_candidates);
  clearCMesh(current_scanned_mesh);



	skeleton.clear();
  slices.clear();
}

void DataMgr::recomputeQuad()
{
	for (int i = 0; i < samples.vert.size(); i++)
	{
		samples.vert[i].recompute_m_render();
	}
  for (int i = 0; i < iso_points.vert.size(); i++)
  {
    iso_points.vert[i].recompute_m_render();
  }
  for (int i = 0; i < original.vert.size(); i++)
  {
    original.vert[i].recompute_m_render();
  }

  recomputeCandidatesAxis();
}

bool cmp_angle(const CVertex &v1, const CVertex &v2)
{
  if (v1.ground_angle == v2.ground_angle) 
    return false;

  //in ascending order
  return v1.ground_angle > v2.ground_angle;
}

void DataMgr::recomputeCandidatesAxis()
{
  for (int i = 0; i < nbv_candidates.vert.size(); i++)
  {
    CVertex& v = nbv_candidates.vert[i];

    Point3f directionZ = v.N();

    Point3f X_axis(1, 0, 0);;

    Point3f directionX = directionZ ^ X_axis;
    Point3f directionY = directionX ^ directionZ;

    v.eigen_vector0 = -directionX.Normalize();
    v.eigen_vector1 = -directionY.Normalize();
  }

  for (int i = 0; i < nbv_candidates.vert.size(); i++)
  {
    CVertex& v = nbv_candidates.vert[i];

    v.N() *= -1;
    v.eigen_vector0 *= -1;
    v.eigen_vector1 *= -1;
  }

  for (int i = 0; i < nbv_candidates.vert.size(); i++)
  {
    CVertex& v = nbv_candidates.vert[i];

    Point3f direction = v.N();
    direction.X() = 0;
    direction = direction.Normalize();

    Point3f Z_axis = Point3f(0, 0, 1);
    Point3f X_axis(1, 0, 0);
    double angle = GlobalFun::computeRealAngleOfTwoVertor(direction, Z_axis);
    Point3f up_direction = Z_axis ^ direction;
    if (up_direction.X() < 0)
    {
      angle = 360 - angle;
    }

    v.ground_angle = angle;
  }

  sort(nbv_candidates.vert.begin(), nbv_candidates.vert.end(), cmp_angle);

  //for (int i = 0; i < nbv_candidates.vert.size(); i++)
  //{
  //  CVertex& v = nbv_candidates.vert[i];

  //  Point3f directionZ = v.N();

  //  Point3f Y_axis(0, 1, 0);;

  //  Point3f directionX = directionZ ^ Y_axis;
  //  Point3f directionY = directionX ^ directionZ;

  //  v.eigen_vector0 = directionX;
  //  v.eigen_vector1 = directionY;
  //}

  //for (int i = 0; i < nbv_candidates.vert.size(); i++)
  //{
  //  CVertex& v = nbv_candidates.vert[i];
  //  
  //  Point3f directionZ = v.N();

  //  float rand_fvalue1 = (rand() % 1000) * 0.001;
  //  float rand_fvalue2 = (rand() % 1000) * 0.001;

  //  Point3f ground_dir(rand_fvalue1, 0, rand_fvalue2);
  //  ground_dir = ground_dir.Normalize();

  //  Point3f directionY = directionZ ^ ground_dir;
  //  Point3f directionX = directionY ^ directionZ;

  //  v.eigen_vector0 = directionX;
  //  v.eigen_vector1 = directionY;
  //}

}

void DataMgr::saveGridPoints(QString fileName)
{
  CMesh* grid_points;
  if (global_paraMgr.glarea.getBool("Show View Grid Slice"))
  {
    grid_points = &view_grid_points;
  }
  else
  {
    grid_points = &field_points;
  }

  ofstream outfile;
  outfile.open(fileName.toStdString().c_str());

  ostringstream strStream; 

  strStream << "GN " << grid_points->vert.size() << endl;
  for(int i = 0; i < grid_points->vert.size(); i++)
  {
    CVertex& v = grid_points->vert[i];
    strStream << v.P()[0] << "	" << v.P()[1] << "	" << v.P()[2] << "	";
    strStream << v.N()[0] << "	" << v.N()[1] << "	" << v.N()[2] << "	" << endl;
  }
  strStream << endl;

  strStream << "Confidence " << grid_points->vert.size() << endl;
  for(int i = 0; i < grid_points->vert.size(); i++)
  {
    CVertex& v = grid_points->vert[i];
    strStream << v.eigen_confidence << "  ";
  }
  strStream << endl;

  outfile.write( strStream.str().c_str(), strStream.str().size() ); 
  outfile.close();
}

void DataMgr::LoadGridPoints(QString fileName, bool is_poisson_field)
{
  CMesh* grid_points;
  if (!is_poisson_field)
  {
    grid_points = &view_grid_points;
  }
  else
  {
    grid_points = &field_points;
  }

  ifstream infile;
  infile.open(fileName.toStdString().c_str());

  stringstream sem; 
  sem << infile.rdbuf(); 

  string str;
  int num;
  int num2;

  grid_points->vert.clear();
  sem >> str;
  if (str == "GN")
  {
    sem >> num;

    for (int i = 0; i < num; i++)
    {
      CVertex v;
      v.is_field_grid = is_poisson_field;
      v.is_grid_center = !is_poisson_field;
      v.m_index = i;
      sem >> v.P()[0] >> v.P()[1] >> v.P()[2];
      sem >> v.N()[0] >> v.N()[1] >> v.N()[2];
      grid_points->vert.push_back(v);
      grid_points->bbox.Add(v.P());
    }
    grid_points->vn = original.vert.size();
  }

  sem >> str;
  float temp;
  if (str == "Confidence")
  {
    sem >> num;
    for (int i = 0; i < num; i++)
    {
      sem >> temp;
      grid_points->vert[i].eigen_confidence = temp;
    }
  }

}


void DataMgr::saveSkeletonAsSkel(QString fileName)
{
	ofstream outfile;
	outfile.open(fileName.toStdString().c_str());

	ostringstream strStream; 

	strStream << "ON " << original.vert.size() << endl;
	for(int i = 0; i < original.vert.size(); i++)
	{
		CVertex& v = original.vert[i];
		strStream << v.P()[0] << "	" << v.P()[1] << "	" << v.P()[2] << "	";
		strStream << v.N()[0] << "	" << v.N()[1] << "	" << v.N()[2] << "	" << endl;
	}
	strStream << endl;

	strStream << "SN " << samples.vert.size() << endl;
	for(int i = 0; i < samples.vert.size(); i++)
	{
		CVertex& v = samples.vert[i];
		strStream << v.P()[0] << "	" << v.P()[1] << "	" << v.P()[2] << "	";
		strStream << v.N()[0] << "	" << v.N()[1] << "	" << v.N()[2] << "	" << endl;
	}
	strStream << endl;

	strStream << "CN " << skeleton.branches.size() << endl;
	for (int i = 0; i < skeleton.branches.size(); i++)
	{
		Branch& branch = skeleton.branches[i];
		strStream << "CNN " << branch.curve.size() << endl;
		for (int j = 0; j < branch.curve.size(); j++)
		{
			strStream << branch.curve[j][0] << "	" << branch.curve[j][1] << "	" << branch.curve[j][2] << "	" << endl;
		}
	}
	strStream << endl;

	strStream << "EN " << 0 << endl;
	strStream << endl;


	strStream << "BN " << 0 << endl;
	strStream << endl;


	strStream << "S_onedge " << samples.vert.size() << endl;
	for(int i = 0; i < samples.vert.size(); i++)
	{
		CVertex& v = samples.vert[i];
		strStream << v.is_fixed_sample << "	"; 
	}
	strStream << endl;

	strStream << "GroupID " << samples.vert.size() << endl;
	for(int i = 0; i < samples.vert.size(); i++)
	{
		int id = samples.vert[i].m_index;//group_id no use now
		strStream << id << "	"; 
	}
	strStream << endl;

	//strStream << "SkelRadius " << 0 << endl;
	//strStream << endl;
  
  strStream << "SkelRadius " << skeleton.size << endl;
  for (int i = 0; i < skeleton.branches.size(); i++)
  {
    for (int j = 0; j < skeleton.branches[i].curve.size(); j++)
    {
      double skel_radius = skeleton.branches[i].curve[j].skel_radius;
      strStream << skel_radius << "	"; 
    }
  }
  strStream << endl;

	strStream << "Confidence_Sigma	" << samples.vert.size() << endl;
	for(int i = 0; i < samples.vert.size(); i++)
	{
		double sigma = samples.vert[i].eigen_confidence;
		strStream << sigma << "	"; 
	}
	strStream << endl;

	strStream << "SkelRadius2 " << 0 << endl;
	strStream << endl;

	strStream << "Alpha " << 0 << endl;
	strStream << endl;

	strStream << "Sample_isVirtual " << samples.vert.size() << endl;
	for(int i = 0; i < samples.vert.size(); i++)
	{
		CVertex& v = samples.vert[i];
		strStream << v.is_skel_virtual << "	"; 
	}
	strStream << endl;

	strStream << "Sample_isBranch " << samples.vert.size() << endl;
	for(int i = 0; i < samples.vert.size(); i++)
	{
		CVertex& v = samples.vert[i];
		strStream << v.is_skel_branch << "	"; 
	}
	strStream << endl;

  strStream << "Sample_radius " << samples.vert.size() << endl;
  for(int i = 0; i < samples.vert.size(); i++)
  {
    CVertex& v = samples.vert[i];
    strStream << 0 << "	"; 
  }
  strStream << endl;

	strStream << "Skel_isVirtual " << skeleton.size << endl;
	for (int i = 0; i < skeleton.branches.size(); i++)
	{
		for (int j = 0; j < skeleton.branches[i].curve.size(); j++)
		{
			bool is_virtual = skeleton.branches[i].curve[j].is_skel_virtual;
			strStream << is_virtual << "	"; 
		}
	}
	strStream << endl;


  strStream << "Corresponding_sample_index " << skeleton.size << endl;
  for (int i = 0; i < skeleton.branches.size(); i++)
  {
    for (int j = 0; j < skeleton.branches[i].curve.size(); j++)
    {
      int index = skeleton.branches[i].curve[j].m_index;
      strStream << index << "	"; 
    }
  }
  strStream << endl;

  strStream << "IN " << iso_points.vert.size() << endl;
  for(int i = 0; i < iso_points.vert.size(); i++)
  {
    CVertex& v = iso_points.vert[i];
    strStream << v.P()[0] << "	" << v.P()[1] << "	" << v.P()[2] << "	";
    strStream << v.N()[0] << "	" << v.N()[1] << "	" << v.N()[2] << "	" << endl;
  }
  strStream << endl;

  strStream << "ISO_Value	" << iso_points.vert.size() << endl;
  for(int i = 0; i < iso_points.vert.size(); i++)
  {
    double sigma = iso_points.vert[i].eigen_confidence;
    strStream << sigma << "	"; 
  }
  strStream << endl;

  strStream << "Is_hole " << iso_points.vert.size() << endl;
  for(int i = 0; i < iso_points.vert.size(); i++)
  {
    CVertex& v = iso_points.vert[i];
    strStream << v.is_hole << "	"; 
  }
  strStream << endl;

  strStream << "CandidateN " << nbv_candidates.vert.size() << endl;
  for(int i = 0; i < nbv_candidates.vert.size(); i++)
  {
    CVertex& v = nbv_candidates.vert[i];
    strStream << v.P()[0] << "	" << v.P()[1] << "	" << v.P()[2] << "	";
    strStream << v.N()[0] << "	" << v.N()[1] << "	" << v.N()[2] << "	" << endl;
  }
  strStream << endl;


  strStream << "Candidate_Confidence	" << nbv_candidates.vert.size() << endl;
  for(int i = 0; i < nbv_candidates.vert.size(); i++)
  {
    double sigma = nbv_candidates.vert[i].eigen_confidence;
    strStream << sigma << "	"; 
  }
  strStream << endl;

  strStream << "Remember_ISO_Index " << nbv_candidates.vert.size() << endl;
  for(int i = 0; i < nbv_candidates.vert.size(); i++)
  {
    int id = nbv_candidates.vert[i].remember_iso_index;
    strStream << id << "	"; 
  }
  strStream << endl;

  strStream << "Scanned_Candidates " << scan_candidates.size() << " " << endl;
  for (int i = 0; i < scan_candidates.size(); ++i)
  {
    strStream << scan_candidates[i].first.X() << " " << scan_candidates[i].first.Y() << " " << scan_candidates[i].first.Z()
              << scan_candidates[i].second.X() << " " << scan_candidates[i].second.Y() << " "<< scan_candidates[i].second.Z() <<endl;
  }
  strStream << endl;

  strStream << "Scanned_Meshes " <<scanned_results.size() <<endl;
  for(int i = 0; i < scanned_results.size(); ++i)
  {
    strStream << "one_scanned_mesh_begins " << scanned_results[i]->vert.size() <<endl;
    for (int j = 0; j < scanned_results[i]->vert.size(); ++j)
    {
      CVertex &v = scanned_results[i]->vert[j];
      strStream<< v.P()[0] << " " << v.P()[1] << " " << v.P()[2] << " " << endl;
    }
  }
  strStream << endl;

	outfile.write( strStream.str().c_str(), strStream.str().size() ); 
	outfile.close();
}


void DataMgr::loadSkeletonFromSkel(QString fileName)
{
  clearCMesh(original);
  clearCMesh(samples);
  clearCMesh(iso_points);
  clearCMesh(field_points);
  clearCMesh(current_scanned_mesh);
  clearCMesh(view_grid_points);
  clearCMesh(nbv_candidates);
  clearCMesh(current_scanned_mesh);

  for (int i = 0; i < scanned_results.size(); ++i)
    clearCMesh(*scanned_results[i]);

  scanned_results.clear();
  scan_candidates.clear();
  slices.clear();
	skeleton.clear();

	ifstream infile;
	infile.open(fileName.toStdString().c_str());

	stringstream sem; 
	sem << infile.rdbuf(); 

	string str;
	int num;
	int num2;

	sem >> str;
	if (str == "ON")
	{
		sem >> num;
		bool is_same_original = false;
		if (num == original.vn)
		{
			is_same_original = true;
		}
		if (is_same_original)
		{
			double temp;
			for (int i = 0; i < num * 6; i++)
			{
				sem >> temp;
			}
		}
		else
		{
			for (int i = 0; i < num; i++)
			{
				CVertex v;
				v.is_original = true;
				v.m_index = i;
				sem >> v.P()[0] >> v.P()[1] >> v.P()[2];
				sem >> v.N()[0] >> v.N()[1] >> v.N()[2];
				original.vert.push_back(v);
				original.bbox.Add(v.P());
			}
			original.vn = original.vert.size();
		}
	}

	sem >> str;
	if (str == "SN")
	{
		sem >> num;
		for (int i = 0; i < num; i++)
		{
			CVertex v;
			v.is_original = false;
			v.m_index = i;
			sem >> v.P()[0] >> v.P()[1] >> v.P()[2];
			sem >> v.N()[0] >> v.N()[1] >> v.N()[2];
			samples.vert.push_back(v);
			samples.bbox.Add(v.P());
		}
		samples.vn = samples.vert.size();
	}


	sem >> str;
	if (str == "CN")
	{
		sem >> num;
		for (int i = 0; i < num; i++)
		{
			Branch branch;
			sem >> str;
			sem >> num2;
			for(int j = 0; j < num2; j++)
			{
				Point3f p;
				CVertex v;
				sem >> p[0] >> p[1] >> p[2];
				v.P() = p;
				branch.curve.push_back(v);
			}
			skeleton.branches.push_back(branch);
		}
	}

	sem >> str;
	if (str == "EN")
	{
    sem >> num;
    for (int i = 0; i < num; i++)
    {
      int a, b;
      sem >> a >> b;
    }
	}

	sem >> str;
	if (str == "BN")
	{
    sem >> num;
    for (int i = 0; i < num; i++)
    {
      sem >> str;
      sem >> num2;

      for(int j = 0; j < num2; j++)
      {
        int id;
        sem >> id;
    }
	}

	if (!sem.eof())
	{
		sem >> str;
		if (str == "S_onedge")
		{
			sem >> num;
			for (int i = 0; i < num; i++)
			{
				bool b;
				sem >> b;
				samples.vert[i].is_fixed_sample = b;
			}
		}
	}

	sem >> str;
	if (str == "GroupID")
	{
		sem >> num;
		for (int i = 0; i < num; i++)
		{
			int id;
			sem >> id;

      }
		}
	}

	sem >> str;
	if (str == "SkelRadius")
	{
		sem >> num;

    if (num > 1)
    {
      double radius;
      for (int i = 0; i < skeleton.branches.size(); i++)
      {
        for (int j = 0; j < skeleton.branches[i].curve.size(); j++)
        {
          sem >> radius;
          skeleton.branches[i].curve[j].skel_radius = radius;
        }
      }
    }

	}

	sem >> str;
	if (str == "Confidence_Sigma")
	{
		sem >> num;
		for (int i = 0; i < num; i++)
		{
			double sigma;
			sem >> sigma;
			samples.vert[i].eigen_confidence = sigma;
		}
	}

	sem >> str;
	if (str == "SkelRadius2")
	{
		sem >> num;

    if (num > 1)
    {
      double radius;
      for (int i = 0; i < skeleton.branches.size(); i++)
      {
        for (int j = 0; j < skeleton.branches[i].curve.size(); j++)
        {
          sem >> radius;
          //skeleton.branches[i].curve[j].skel_radius = radius;
        }
      }
    }

	}

	sem >> str;
	if (str == "Alpha")
	{
		sem >> num;
    double Alpha;
    if (num > 1)
    {
      for (int i = 0; i < skeleton.branches.size(); i++)
      {
        for (int j = 0; j < skeleton.branches[i].curve.size(); j++)
        {
          sem >> Alpha;
          //skeleton.curves[i][j].alpha = Alpha;
        }
      }
    }

	}

	if (!sem.eof())
	{
		sem >> str;
		if (str == "Sample_isVirtual")
		{
			sem >> num;
			for (int i = 0; i < num; i++)
			{
				bool b;
				sem >> b;
				samples.vert[i].is_skel_virtual = b;
			}
		}
	}

	if (!sem.eof())
	{
		sem >> str;
		if (str == "Sample_isBranch")
		{
			sem >> num;
			for (int i = 0; i < num; i++)
			{
				bool b;
				sem >> b;
				samples.vert[i].is_skel_branch = b;
			}
		}
	}

	sem >> str;
	if (str == "Sample_radius")
	{
    sem >> num;
    for (int i = 0; i < num; i++)
    {
      double temp;
      sem >> temp;
    }
	}

	sem >> str;
	if (str == "Skel_isVirtual")
	{
		sem >> num;
		bool temp;
		for (int i = 0; i < skeleton.branches.size(); i++)
		{
			for (int j = 0; j < skeleton.branches[i].curve.size(); j++)
			{
				sem >> temp;
				skeleton.branches[i].curve[j].is_skel_virtual = temp;
			}
		}
	}

  sem >> str;
  if (str == "Corresponding_sample_index")
  {
    sem >> num;
    int temp;
    for (int i = 0; i < skeleton.branches.size(); i++)
    {
      for (int j = 0; j < skeleton.branches[i].curve.size(); j++)
      {
        sem >> temp;
        skeleton.branches[i].curve[j].m_index = temp;
      }
    }
  }
	skeleton.generateBranchSampleMap();

  sem >> str;
  if (str == "IN")
  {
    sem >> num;
    for (int i = 0; i < num; i++)
    {
      CVertex v;
      v.is_original = false;
      v.is_iso = true;
      v.m_index = i;
      sem >> v.P()[0] >> v.P()[1] >> v.P()[2];
      sem >> v.N()[0] >> v.N()[1] >> v.N()[2];
      iso_points.vert.push_back(v);
      iso_points.bbox.Add(v.P());
    }
    iso_points.vn = iso_points.vert.size();
  }


  sem >> str;
  if (str == "ISO_Value")
  {
    sem >> num;
    for (int i = 0; i < num; i++)
    {
      double sigma;
      sem >> sigma;
      iso_points.vert[i].eigen_confidence = sigma;
    }
  }

  if (!sem.eof())
  {
    sem >> str;
    if (str == "Is_hole")
    {
      sem >> num;
      for (int i = 0; i < num; i++)
      {
        bool b;
        sem >> b;
        iso_points.vert[i].is_hole = b;
      }
    }
  }

  sem >> str;
  if (str == "CandidateN")
  {
    sem >> num;
    for (int i = 0; i < num; i++)
    {
      CVertex v;
      v.is_original = false;
      //v.is_grid_center = true;
      v.m_index = i;
      sem >> v.P()[0] >> v.P()[1] >> v.P()[2];
      sem >> v.N()[0] >> v.N()[1] >> v.N()[2];
      cout << v.N()[0] << " " << v.N()[1] << endl; 
      nbv_candidates.vert.push_back(v);
      nbv_candidates.bbox.Add(v.P());
    }
    nbv_candidates.vn = nbv_candidates.vert.size();
  }

  sem >> str;
  if (str == "Candidate_Confidence")
  {
    sem >> num;
    for (int i = 0; i < num; i++)
    {
      double sigma;
      sem >> sigma;
      nbv_candidates.vert[i].eigen_confidence = sigma;
    }
  }

  sem >> str;
  if (str == "Remember_ISO_Index")
  {
    sem >> num;
    for (int i = 0; i < num; i++)
    {
      int id;
      sem >> id;
      nbv_candidates.vert[i].remember_iso_index = id;
    }
  }

  sem>>str;
  if (str == "Scanned_Candidates")
  {
    sem >> num;
    for (int i = 0; i < num; ++i)
    {
      Point3f pos, direction;
      sem >> pos.X() >> pos.Y() >> pos.Z() >> direction.X() >> direction.Y() >> direction.Z();
      ScanCandidate sc = std::make_pair(pos, direction);
      scan_candidates.push_back(sc);
    }
  }

  sem >> str;
  if (str == "Scanned_Meshes")
  {
    sem >> num;
    for (int i = 0; i < num; ++i)
    {
      sem >> str;
      if (str == "one_scanned_mesh_begins")
      {
        int sc_num = 0;
        sem >> sc_num;

        CMesh *m = new CMesh;
        int index = 0;
        for (int j = 0; j < sc_num; ++j)
        {
          CVertex v;
          v.is_scanned = true;
          v.m_index = index++;
          sem >> v.P()[0] >> v.P()[1] >> v.P()[2];
          m->vert.push_back(v);
          m->bbox.Add(v.P());
        }
        m->vn = m->vert.size();
        scanned_results.push_back(m);
      }
    }
  }
  /*if (str == "VN")
  {
  sem >> num;
  for (int i = 0; i < num; i++)
  {
  CVertex v;
  v.is_original = false;
  v.is_iso = false;
  v.is_view_candidates = true;
  v.m_index = i;
  sem >> v.P()[0] >> v.P()[1] >> v.P()[2];
  sem >> v.N()[0] >> v.N()[1] >> v.N()[2];
  view_candidates.vert.push_back(v);
  view_candidates.bbox.Add(v.P());
  }
  view_candidates.vn = iso_points.vert.size();
  }*/
}

void DataMgr::saveFieldPoints(QString fileName)
{
  if (field_points.vert.empty())
  {
    cout<<"save Field Points Error: Empty field_points" <<endl;
    return;
  }

  //ofstream outfile;
  //outfile.open(fileName.toStdString().c_str());

  //ostringstream strStream; 

  //strStream << "ON " << original.vert.size() << endl;

  //FILE *fp = fopen(fileName.toStdString().c_str(),"rb");
  //if(!fp)
  //  cerr<<"open "<< fileName.toStdString().c_str() <<" failed"<<endl;

  ofstream fout;
  fout.open(fileName.toAscii(), std::ios::out | std::ios::binary);
  if( fout == NULL)
  {
    cout<<" error ----- "<<endl;
    return;
  }


  //for (int i = 0; i < field_points.vert.size(); i++)
  cout << field_points.vert.size() << " grids" << endl;
  for (int i = 0; i < field_points.vert.size(); i++)  
  {
    CVertex& v = field_points.vert[i];
    float eigen_value = v.eigen_confidence * 255;

    unsigned char pTest = static_cast<unsigned char>(eigen_value);

    fout << pTest;
  }
  fout.close();

  ofstream fout_dat;
  QString fileName_dat = fileName;
  QString temp = fileName;
  QStringList str_list = temp.split(QRegExp("[/]"));
  QString last_name = str_list.at(str_list.size()-1);
  cout << "file name: " << last_name.toStdString() << endl;

  int resolution = global_paraMgr.poisson.getInt("Field Points Resolution");
  fileName_dat.replace(".raw", ".dat");
  fout_dat.open(fileName_dat.toAscii());
  fout_dat << "ObjectFileName:  " << last_name.toStdString() << endl;
  fout_dat << "Resolution:  " << resolution << " " << resolution << " " << resolution << endl;
  fout_dat << "SliceThickness:	0.0127651 0.0127389 0.0128079" << endl;
  fout_dat << "Format:		    UCHAR" << endl;
  fout_dat << "ObjectModel:	I" << endl;
  fout_dat << "Modality:	    CT" << endl;
  fout_dat << "Checksum:	    7b197a4391516321308b81101d6f09e8" << endl;
  fout_dat.close();

}

void
DataMgr::saveViewGrids(QString fileName)
{
  if (view_grid_points.vert.empty()) return;

  ofstream out;
  out.open(fileName.toAscii(), std::ios::out | std::ios::binary);
  if (NULL == out) 
  {
      cout<<"open file Error!" <<endl;
      return;
  }

  for (int i = 0; i < view_grid_points.vert.size(); ++i)
  {
    CVertex &v = view_grid_points.vert[i];
    float eigen_value = v.eigen_confidence * 255;
    unsigned char p = static_cast<unsigned char>(eigen_value);
    out << p;
  }
  out.close();

  QString tmp = fileName;
  QStringList str_lst = tmp.split(QRegExp("[/]"));
  QString last_name = str_lst.at(str_lst.size() - 1);

  double resolution = global_paraMgr.nbv.getDouble("View Grid Resolution");
  ofstream out_dat;
  QString fileName_dat = fileName;
  fileName_dat.replace(".raw", ".dat");
  out_dat.open(fileName_dat.toAscii());
  out_dat << "ObjectFileName:  " << last_name.toStdString() << endl;
  out_dat << "Resolution:  " << resolution << " " << resolution << " " << resolution << endl;
  out_dat << "SliceThickness:	0.0127651 0.0127389 0.0128079" << endl;
  out_dat << "Format:		    UCHAR" << endl;
  out_dat << "ObjectModel:	I" << endl;
  out_dat << "Modality:	    CT" << endl;
  out_dat << "Checksum:	    7b197a4391516321308b81101d6f09e8" << endl;
  out_dat.close();
}

void
DataMgr::saveMergedMesh(QString fileName)
{
  for(int i = 0; i < scanned_results.size(); ++i)
  {
    QString s_i;
    s_i.sprintf("_%d.ply", i);
    QString r = fileName + s_i;
    savePly(r, *scanned_results[i]);
  }
}

void 
DataMgr::saveParameters(QString fileName)
{
  ofstream out_para;
  out_para.open(fileName.toAscii().data(), std::ios::out);
  if (out_para == NULL)
    return ;

  out_para << "#1. KNN for compute PCA normal" << endl
    << global_paraMgr.norSmooth.getInt("PCA KNN") << endl << endl; 

  out_para << "#2. Camera Resolution, something like(1.0 / 50.0f)" << endl
    << global_paraMgr.camera.getDouble("Camera Resolution") << endl << endl;

  out_para << "#3. Sharp Sigma" << endl
    << global_paraMgr.norSmooth.getDouble("Sharpe Feature Bandwidth Sigma") << endl <<endl;

  out_para << "#4. View Grid Resolution" <<endl
    << global_paraMgr.nbv.getDouble("View Grid Resolution") << endl <<endl;

  out_para << "#5. Poisson Max Depth" <<endl
    << global_paraMgr.poisson.getDouble("Max Depth") << endl <<endl;

  out_para << "#6. Original KNN" <<endl
    << global_paraMgr.poisson.getDouble("Original KNN") << endl << endl;

  out_para << "#7. merge probability X . pow(1-confidence, x)" <<endl
    << global_paraMgr.nbv.getDouble("Merge Probability Pow") <<endl <<endl;

  out_para << "#8. Optimal Plane Width" <<endl
    << global_paraMgr.camera.getDouble("Optimal Plane Width") <<endl <<endl;

  out_para << "#9. Merge Confidence Threshold" << endl
    << global_paraMgr.camera.getDouble("Merge Confidence Threshold") <<endl << endl;

  out_para << "#10. View Bin Number On Each Axis" << endl
    << global_paraMgr.nbv.getInt("View Bin Each Axis") <<endl << endl;

  out_para.close();
}

void
DataMgr::loadParameters(QString fileName)
{
  ifstream in_para;
  in_para.open(fileName.toAscii().data());
  if (in_para == NULL)
    return;
  
  string value;

  in_para.ignore(1000, '\n');
  int knn;
  getline(in_para, value);
  knn = atoi(value.c_str());
  global_paraMgr.norSmooth.setValue("PCA KNN", IntValue(knn));

  in_para.ignore(1000, '\n');
  in_para.ignore(1000, '\n');
  double camera_resolution;
  getline(in_para, value);
  camera_resolution = atof(value.c_str());
  global_paraMgr.camera.setValue("Camera Resolution", DoubleValue(camera_resolution));

  in_para.ignore(1000, '\n');
  in_para.ignore(1000, '\n');
  double sharp_sigma;
  getline(in_para, value);
  sharp_sigma = atof(value.c_str());
  global_paraMgr.norSmooth.setValue("Sharpe Feature Bandwidth Sigma", DoubleValue(sharp_sigma));

  in_para.ignore(1000, '\n');
  in_para.ignore(1000, '\n');
  int grid_resolution;
  getline(in_para, value);
  grid_resolution = atoi(value.c_str());
  global_paraMgr.nbv.setValue("View Grid Resolution", DoubleValue(grid_resolution));

  in_para.ignore(1000, '\n');
  in_para.ignore(1000, '\n');
  int poisson_depth;
  getline(in_para, value);
  poisson_depth = atoi(value.c_str());
  global_paraMgr.poisson.setValue("Max Depth", DoubleValue(poisson_depth));

  in_para.ignore(1000, '\n');
  in_para.ignore(1000, '\n');
  double original_knn;
  getline(in_para, value);
  original_knn = atof(value.c_str());
  global_paraMgr.poisson.setValue("Original KNN", DoubleValue(original_knn));

  in_para.ignore(1000, '\n');
  in_para.ignore(1000, '\n');
  double merge_pow;
  getline(in_para, value);
  merge_pow = atof(value.c_str());
  global_paraMgr.nbv.setValue("Merge Probability Pow", DoubleValue(merge_pow));


  in_para.ignore(1000, '\n');
  in_para.ignore(1000, '\n');
  double optimal_plane_width;
  getline(in_para, value);
  optimal_plane_width = atof(value.c_str());
  global_paraMgr.camera.setValue("Optimal Plane Width", DoubleValue(optimal_plane_width));

  in_para.ignore(1000, '\n');
  in_para.ignore(1000, '\n');
  double merge_confidence_threshold;
  getline(in_para, value);
  merge_confidence_threshold = atof(value.c_str());
  global_paraMgr.camera.setValue("Merge Confidence Threshold", DoubleValue(merge_confidence_threshold));

  in_para.ignore(1000, '\n');
  in_para.ignore(1000, '\n');
  int nbv_bin_num;
  getline(in_para, value);
  nbv_bin_num = atoi(value.c_str());
  global_paraMgr.nbv.setValue("View Bin Each Axis", IntValue(nbv_bin_num));

  in_para.close();
}

void DataMgr::switchSampleToOriginal()
{
  CMesh temp_mesh;
  replaceMesh(original, temp_mesh, false);
  replaceMesh(samples, original, true);
  replaceMesh(temp_mesh, samples, false);
}

void DataMgr::switchSampleToISO()
{
  CMesh temp_mesh;
  replaceMesh2(iso_points, temp_mesh, false);
  replaceMesh2(samples, iso_points, true);
  replaceMesh2(temp_mesh, samples, false);
}

void DataMgr::replaceMesh(CMesh& src_mesh, CMesh& target_mesh, bool isOriginal)
{
  clearCMesh(target_mesh);
  for(int i = 0; i < src_mesh.vert.size(); i++)
  {
    CVertex v = src_mesh.vert[i];
    v.is_original = isOriginal;
    v.m_index = i;
    target_mesh.vert.push_back(v);
  }
  target_mesh.vn = src_mesh.vn;
  target_mesh.bbox = src_mesh.bbox;
}

void DataMgr::replaceMesh2(CMesh& src_mesh, CMesh& target_mesh, bool isIso)
{
  clearCMesh(target_mesh);
  for(int i = 0; i < src_mesh.vert.size(); i++)
  {
    CVertex v = src_mesh.vert[i];
    v.is_iso = isIso;
    v.m_index = i;
    target_mesh.vert.push_back(v);
  }
  target_mesh.vn = src_mesh.vn;
  target_mesh.bbox = src_mesh.bbox;
}

void DataMgr::loadCurrentTF(QString fileName)
{
  //ifstream infile;
  //infile.open(fileName.toStdString().c_str());
  
  QFile file(fileName);
  
  if(!file.open( QIODevice::ReadOnly ))
  {
    return;
  }

  QTextStream infile(&file);
  std::string temp_str;
  float temp_float;
  char temp_c;
  QString line;

  line = infile.readLine();
  cout << line.toStdString() << endl;

  line.replace("- Translation: [", "");
  line.replace("]", "");

  QStringList str_list;
  str_list = line.split(", ");
  if (str_list.size() != 3)
  {
    cout << "wrong tf file!!!" << endl;
    return;
  }
  for (int i = 0; i < 3; i++)
  {
    QString qstr = str_list.at(i);
    float f_value = qstr.toFloat();
    current_L_to_R_Translation[i] = f_value * 1000.;
  }


  line = infile.readLine();
  cout << line.toStdString() << endl;

  line.replace("- Rotation: in Quaternion [", "");
  line.replace("]", "");

  str_list = line.split(", ");
  if (str_list.size() != 4)
  {
    cout << "wrong tf file!!!" << endl;
    return;
  }
  for (int i = 0; i < 4; i++)
  {
    QString qstr = str_list.at(i);
    float f_value = qstr.toFloat();
    current_L_to_R_Rotation_Qua[i] = f_value;
  }


  line = infile.readLine();
  cout << line.toStdString() << endl;
  line.replace("in RPY [", "");
  line.replace("]", "");

  str_list = line.split(", ");
  if (str_list.size() != 3)
  {
    cout << "wrong tf file!!!" << endl;
    return;
  }
  for (int i = 0; i < 3; i++)
  {
    QString qstr = str_list.at(i);
    float f_value = qstr.toFloat();
    current_L_to_R_Angle[i] = f_value;
  }
 
}

void DataMgr::loadCommonTransform()
{
  ifstream infile("common_transform.txt");
  std::string temp_str;

  infile >> temp_str;
  Point3f R_to_S_angle;
  infile >> R_to_S_angle[0] >> R_to_S_angle[1] >> R_to_S_angle[2];
  infile >> temp_str;
  Point3f R_to_S_translation;
  infile >> R_to_S_translation[0] >> R_to_S_translation[1] >> R_to_S_translation[2];
  
  Quaternionf R_to_S__Qua;
  R_to_S__Qua.FromEulerAngles(R_to_S_angle[0], R_to_S_angle[1], R_to_S_angle[2]);  
  
  vcg::Matrix33f R_to_S_Matrix33;
  R_to_S__Qua.ToMatrix(R_to_S_Matrix33);

  R_to_S_Matrix44.SetIdentity();
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      R_to_S_Matrix44[i][j] = R_to_S_Matrix33[i][j];
    }
  }
  R_to_S_Matrix44[0][3] = R_to_S_translation[0];
  R_to_S_Matrix44[1][3] = R_to_S_translation[1];
  R_to_S_Matrix44[2][3] = R_to_S_translation[2];


  infile >> temp_str;
  float temp_f = 0.0;
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      infile >> temp_f;
      T_to_L_Matrix44[i][j] = temp_f;
    }
  }
}

void DataMgr::coordinateTransform()
{

  Matrix44f L_to_R_Matrix44;
  L_to_R_Matrix44.SetIdentity();
  Matrix33f L_to_R_Matrix33;
  //vcg::Transpose(L_to_R_Matrix33);
  //current_L_to_R_Rotation_Qua.ToMatrix(L_to_R_Matrix33);
  L_to_R_Matrix33 = GlobalFun::myQuaternionToMatrix33(current_L_to_R_Rotation_Qua);

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      L_to_R_Matrix44[i][j] = L_to_R_Matrix33[i][j];
    }
  }

  L_to_R_Matrix44[0][3] = current_L_to_R_Translation[0];
  L_to_R_Matrix44[1][3] = current_L_to_R_Translation[1];
  L_to_R_Matrix44[2][3] = current_L_to_R_Translation[2];

  vcg::Matrix44f T_to_S_Matrix44 = T_to_L_Matrix44 * L_to_R_Matrix44 * R_to_S_Matrix44;
  //vcg::Matrix44f inv_T_to_S_Matrix44 = vcg::Inverse(T_to_S_Matrix44);
  ofstream out("test_trans_out.txt");
  out << "T_to_L_Matrix44 " << endl;
  GlobalFun::printMatrix44(out, T_to_L_Matrix44);
  out << "L_to_R_Matrix44 " << endl;
  GlobalFun::printMatrix44(out, L_to_R_Matrix44);
  out << "R_to_S_Matrix44" << endl;
  GlobalFun::printMatrix44(out, R_to_S_Matrix44);
  out << "T_to_S_Matrix44" << endl;
  GlobalFun::printMatrix44(out, T_to_S_Matrix44);


  for (int i = 0; i < samples.vert.size(); i++)
  {

    CVertex& v = samples.vert[i];
    if (i < 10)
    {
      out << "before trans: ";
      GlobalFun::printPoint3(out, v.P());
    }

    v.P() = T_to_S_Matrix44 * v.P();

    if (i < 10)
    {
      out << "after trans: ";
      GlobalFun::printPoint3(out, v.P());
    }

  }
}


//void DataMgr::coordinateTransform()
//{
//  vcg::Quaternionf L_to_R_rotation_Qua;
//  Point3f C_to_L_translation;
//  Point3f L_to_R_translation;
//  Point3f L_displacement;
//
//  ifstream infile("common_transform.txt");
//  std::string temp_str;
//
//  infile >> temp_str; 
//  infile >> L_to_R_translation[0] >> L_to_R_translation[1] >> L_to_R_translation[2];
//
//  infile >> temp_str; 
//  infile >> L_to_R_rotation_Qua.X() >> L_to_R_rotation_Qua.Y() >> L_to_R_rotation_Qua.Z() >> L_to_R_rotation_Qua.W();
//
//  Matrix44f L_to_R_Matrix44;
//  L_to_R_Matrix44.SetIdentity();
//  Matrix33f L_to_R_Matrix33;
//  L_to_R_rotation_Qua.ToMatrix(L_to_R_Matrix33);
//  for (int i = 0; i < 3; i++)
//  {
//    for (int j = 0; j < 3; j++)
//    {
//      L_to_R_Matrix44[i][j] = L_to_R_Matrix33[i][j];
//    }
//  }
//  L_to_R_Matrix44[0][3] = L_to_R_translation[0];
//  L_to_R_Matrix44[1][3] = L_to_R_translation[1];
//  L_to_R_Matrix44[2][3] = L_to_R_translation[2];
//
//
//   Matrix44f R_to_S_Matrix44;
//  infile >> temp_str;
//  vcg::Matrix44f R_to_S_Matrix;
//  float temp_f = 0.0;
//  for (int i = 0; i < 4; i++)
//  {
//    for (int j = 0; j < 4; j++)
//    {
//      infile >> temp_f;
//      R_to_S_Matrix44[i][j] = temp_f;
//    }
//  }
//
//  Matrix44f T_to_L_Matrix44;
//  infile >> temp_str;
//  temp_f = 0.0;
//  for (int i = 0; i < 4; i++)
//  {
//    for (int j = 0; j < 4; j++)
//    {
//      infile >> temp_f;
//      T_to_L_Matrix44[i][j] = temp_f;
//    }
//  }
//
//  vcg::Quaternionf R_to_S_rotation_Qua;
//  Point3f R_to_S_Translation;
//
//  infile >> temp_str; 
//  infile >> R_to_S_Translation[0] >> R_to_S_Translation[1] >> R_to_S_Translation[2];
//
//  infile >> temp_str; 
//  infile >> R_to_S_rotation_Qua.X() >> R_to_S_rotation_Qua.Y() >> R_to_S_rotation_Qua.Z() >> R_to_S_rotation_Qua.W();
//
//  R_to_S_Matrix44.SetIdentity();
//  Matrix33f R_to_S_Matrix33;
//  L_to_R_rotation_Qua.ToMatrix(R_to_S_Matrix33);
//  for (int i = 0; i < 3; i++)
//  {
//    for (int j = 0; j < 3; j++)
//    {
//      R_to_S_Matrix44[i][j] = R_to_S_Matrix33[i][j];
//    }
//  }
//  R_to_S_Matrix44[0][3] = R_to_S_Translation[0];
//  R_to_S_Matrix44[1][3] = R_to_S_Translation[1];
//  R_to_S_Matrix44[2][3] = R_to_S_Translation[2];
//
//  vcg::Matrix44f matrix = T_to_L_Matrix44 * L_to_R_Matrix44 * R_to_S_Matrix44;
//  
//  //R_to_S_Qua.FromMatrix(R_to_S_Matrix44);
//
//  //Matrix44f C_to_R_Matrix44 = vcg::Inverse(R_to_S_Matrix44);
//  //
//  //vcg::Quaternionf C_to_R_Qua;
//  //C_to_R_Qua.FromMatrix(C_to_R_Matrix44);
//  //vcg::Matrix44f matrix = C_to_R_Matrix44;
//  //vcg::Matrix44f matrix_inv = vcg::Inverse(matrix);
//
//  for (int i = 0; i < samples.vert.size(); i++)
//  {
//    CVertex& v = samples.vert[i];
//    v.P() = matrix * v.P();
//  }
//
//
//}


//void DataMgr::coordinateTransform()
//{
//  //int index = rand() % samples.vert.size();
//  //CVertex v = samples.vert[index];
//  //Point3f anchor(0, 0, 0);
//  //Point3f direction(0, 1, 0);
//
//  //v.P()[0] = 0.0;
//  //Point3f direction2 = v.P().Normalize();
//  //cout << "Angle" << GlobalFun::computeRealAngleOfTwoVertor(direction, direction2) << endl;
//
//  ifstream infile_new("transform.txt");
//  std::string temp_str;
//
//
//
//  vcg::Quaternionf L_to_R_rotation_Qua;
//  Point3f C_to_L_translation;
//  Point3f L_to_R_translation;
//  Point3f L_displacement;
//
//  ifstream infile("transform.txt");
//  //std::string temp_str;
//
//  infile >> temp_str; 
//  infile >> C_to_L_translation[0] >> C_to_L_translation[1] >> C_to_L_translation[2];
//  
//  infile >> temp_str;
//  vcg::Matrix33f C_to_L_rotation;
//  float temp_f = 0.0;
//  for (int i = 0; i < 3; i++)
//  {
//    for (int j = 0; j < 3; j++)
//    {
//      infile >> temp_f;
//      C_to_L_rotation[i][j] = temp_f;
//    }
//  }
//
//  infile >> temp_str; 
//  infile >> L_displacement[0] >> L_displacement[1] >> L_displacement[2];
//
//  infile >> temp_str; 
//  infile >> L_to_R_translation[0] >> L_to_R_translation[1] >> L_to_R_translation[2];
//
//  infile >> temp_str >> L_to_R_rotation_Qua.X() >> L_to_R_rotation_Qua.Y() >> L_to_R_rotation_Qua.Z() >> L_to_R_rotation_Qua.W();
//  
//
//  infile.close();
//
//  vcg::Matrix44f matrix;
//  Point3f p;
//  Point3f pt = matrix * p;
//
//  //vcg::Matrix44f matrix;
//  //L_to_R_rotation_Qua.ToMatrix(matrix);
//
//  //for (int i = 0; i < samples.vert.size(); i++)
//  //{
//  //  CVertex& v = samples.vert[i];
//
//  //  v.P() -= C_to_L_translation;
//  //  v.P() = matrix * v.P();;
//  //  v.P() = C_to_L_rotation * v.P();
//  //  v.P() -= L_displacement;
//  //}
//
//  ofstream outfile("transform_out.txt");
//  vcg::Quaternionf C_to_L_rotation_Qua;
//  C_to_L_rotation_Qua.FromMatrix(C_to_L_rotation);
//
//  outfile << "C_to_L_rotation_Qua:  " << C_to_L_rotation_Qua.X() << " "
//                                      << C_to_L_rotation_Qua.Y() << " "
//                                      << C_to_L_rotation_Qua.Z() << " "
//                                      << C_to_L_rotation_Qua.W() << endl;
//
//  vcg::Quaternionf C_to_R_rotation_Qua;
//  C_to_R_rotation_Qua = C_to_L_rotation_Qua * L_to_R_rotation_Qua;
//  outfile << "C_to_R_rotation_Qua:  " << C_to_R_rotation_Qua.X() << " "
//                                      << C_to_R_rotation_Qua.Y() << " "
//                                      << C_to_R_rotation_Qua.Z() << " "
//                                      << C_to_R_rotation_Qua.W() << endl;
//
//  vcg::Quaternionf R_to_S_rotation_Qua = C_to_R_rotation_Qua.Inverse();
//  outfile << "R_to_S_rotation_Qua:  " << R_to_S_rotation_Qua.X() << " "
//                                      << R_to_S_rotation_Qua.Y() << " "
//                                      << R_to_S_rotation_Qua.Z() << " "
//                                      << R_to_S_rotation_Qua.W() << endl;
//  Point3f C_to_R_translation;
//  C_to_R_translation = C_to_L_translation + L_displacement + L_to_R_translation;
//  outfile << "C_to_R_translation:  " << C_to_R_translation[0] << " " 
//                                     << C_to_R_translation[1] << " "
//                                     << C_to_R_translation[2] << endl;
//
//
//
//  Quaternionf test_qua;
//  //test_qua.FromEulerAngles(1.57, 1.57, 0.26);
//  test_qua.FromEulerAngles(2.87979, 0, 1.5707);  
//  outfile << "test_qua:  " << test_qua.X() << " "
//                                      << test_qua.Y() << " "
//                                      << test_qua.Z() << " "
//                                      << test_qua.W() << endl << endl;;
//  Matrix33f test_mat;
//  test_qua.ToMatrix(test_mat);
//  for (int i = 0; i < 3; i++)
//  {
//    for (int j = 0; j < 3; j++)
//    {
//      outfile << test_mat[i][j] << " ";
//    }
//    outfile << endl;
//  }
//
//  //Matrix33f test_mat2;
//  //test_mat2[0][0] = -0.021;
//  //test_mat2[0][1] = 0.977;
//  //test_mat2[0][2] = 0.249;
//  //test_mat2[1][0] = 0.98377;
//  //test_mat2[1][1] = -0.04260;
//  //test_mat2[1][2] = -0.041029;
//  //test_mat2[2][0] = -0.050324;
//  //test_mat2[2][1] = -0.29;
//  //test_mat2[2][2] = -0.967985;
//  //Quaternionf test_qua2;
//  //test_qua2.FromMatrix(test_mat2);
//  //float a, b, c;
//  //test_qua2.ToEulerAngles(a, b, c);
//  //outfile << "test angle: " << a << " " << b << " " << c << endl;
//
//  //double dist = GlobalFun::computeEulerDist(v.P(), anchor);
//  //double perpend_dist = GlobalFun::computePerpendicularDist(anchor, v.P(), direction);
//
//  //double proj_dist = GlobalFun::
//}
