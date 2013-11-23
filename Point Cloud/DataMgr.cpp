#include "DataMgr.h"


DataMgr::DataMgr(RichParameterSet* _para)
{
	para = _para;
  camera_pos = Point3f(0.0f, 0.0f, 1.0f);
  camera_direction = Point3f(0.0f, 0.0f, -1.0f);

  initDefaultScanCamera();

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
  init_scan_candidates.push_back(make_pair(Point3f(0.0f, 1.0f, 0.0f), Point3f(0.0f, -1.0f, 0.0f)));
  init_scan_candidates.push_back(make_pair(Point3f(0.0f, -1.0f, 0.0f), Point3f(0.0f, 1.0f, 0.0f)));
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

bool DataMgr::isNBVGridsEmpty()
{
  return all_nbv_grid_centers.vert.empty();
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

vector<NBVGrid>*
DataMgr::getAllNBVGrids()
{
  return &all_nbv_grids;
}

CMesh*
DataMgr::getAllNBVGridCenters()
{
  return &all_nbv_grid_centers;
}

CMesh*
DataMgr::getRayHitGrids()
{
  return &ray_hit_grids;
}

vector<ScanCandidate>* DataMgr::getInitCameraScanCandidates()
{
  return &init_scan_candidates;
}

vector<ScanCandidate>* DataMgr::getAllScanCandidates()
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
	samples.bbox.SetNull();
	original.bbox.SetNull();

	CMesh::VertexIterator vi;
	for(vi = samples.vert.begin(); vi != samples.vert.end(); ++vi) 
	{
		if (vi->is_skel_ignore)
		{
			continue;
		}
		samples.bbox.Add(vi->P());
	}

	for(vi = original.vert.begin(); vi != original.vert.end(); ++vi) 
	{
		original.bbox.Add(vi->P());
	}
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
  mask += tri::io::Mask::IOM_ALL;
	mask += tri::io::Mask::IOM_BITPOLYGONAL;
  mask += tri::io::Mask::IOM_FACEINDEX;

	if (fileName.endsWith("ply"))
		tri::io::ExporterPLY<CMesh>::Save(mesh, fileName.toAscii().data(), mask, false);
}

void DataMgr::normalizeROSA_Mesh(CMesh& mesh)
{
  if (mesh.vert.empty())
  {
    return;
  }
  Box3f box = mesh.bbox;
  mesh.bbox.SetNull();
  float max_x = abs((box.min - box.max).X());
  float max_y = abs((box.min - box.max).Y());
  float max_z = abs((box.min - box.max).Z());
  float max_length = max_x > max_y ? max_x : max_y;
  max_length = max_length > max_z ? max_length : max_z;

  Box3f box_temp;
  for(int i = 0; i < mesh.vert.size(); i++)
  {
    Point3f& p = mesh.vert[i].P();

    p -= box.min;
    p /= max_length;

    p -= Point3f(0.5, .5, .5);
    //p *= 2.0;

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
}


Box3f DataMgr::normalizeAllMesh()
{
	Box3f box;
  if (!isModelEmpty())
  {
    for (int i = 0; i < model.vert.size(); ++i)
    {
      box.Add(model.vert[i].P());
    }

    model.bbox = box;
    normalizeROSA_Mesh(model);
    recomputeBox();

    return model.bbox;
  }
  else
  {
    if (!isSamplesEmpty())
    {
      for (int i = 0; i < samples.vert.size(); ++i)
      {
        box.Add(samples.vert[i].P());
      }
    }
    if (!isOriginalEmpty())
    {
      for (int i = 0; i < original.vert.size(); ++i)
      {
        box.Add(original.vert[i].P());
      }
      original.bbox =box;
    }

    samples.bbox = box;

    normalizeROSA_Mesh(samples);
    normalizeROSA_Mesh(original);
    normalizeROSA_Mesh(iso_points);

    recomputeBox();
    getInitRadiuse();

    return samples.bbox;
  }
}


void DataMgr::eraseRemovedSamples()
{
	int cnt = 0;
	vector<CVertex> temp_mesh;
	for (int i = 0; i < samples.vert.size(); i++)
	{
		CVertex& v = samples.vert[i];
		if (!v.is_skel_ignore)
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

  //clearCMesh(model);  
  clearCMesh(current_scanned_mesh);

  clearCMesh(all_nbv_grid_centers);
  clearCMesh(ray_hit_grids);
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

  //strStream << "Is_hole " << iso_points.vert.size() << endl;
  //for(int i = 0; i < iso_points.vert.size(); i++)
  //{
  //  CVertex& v = iso_points.vert[i];
  //  strStream << v.is_hole << "	"; 
  //}
  //strStream << endl;

	outfile.write( strStream.str().c_str(), strStream.str().size() ); 
	outfile.close();
}




void DataMgr::loadSkeletonFromSkel(QString fileName)
{
  clearData();
	//clearCMesh(samples);
	//clearCMesh(original);
 // clearCMesh(iso_points);
 // clearCMesh(field_points);

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
      //samples.vert[i].saved_radius = temp;
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
