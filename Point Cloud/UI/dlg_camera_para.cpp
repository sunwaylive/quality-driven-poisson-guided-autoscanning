#include "UI/dlg_camera_para.h"

CameraParaDlg::CameraParaDlg(QWidget *p, ParameterMgr * _paras, GLArea * _area) : QFrame(p)
{
  ui = new Ui::camera_paras;
  CameraParaDlg::ui->setupUi(this);
  area = _area;
  m_paras = _paras;

  if(!initWidgets())
  {
  cerr << " PoissonParaDlg::initWidgets failed." << endl;
  return;
  }
	initConnects();
}

void CameraParaDlg::initConnects()
{
  if (!connect(area,SIGNAL(needUpdateStatus()),this,SLOT(initWidgets())))
  {
    cout << "can not connect signal" << endl;
  }

  connect(ui->doubleSpinBox_view_prune_confidence_threshold, SIGNAL(valueChanged(double)), this, SLOT(getViewPruneConfidenceThreshold(double)));
  connect(ui->pushButton_view_prune, SIGNAL(clicked()), this, SLOT(runViewPrune()));
  connect(ui->pushButton_show_candidate_index, SIGNAL(clicked()), this, SLOT(showCandidateIndex()));
  connect(ui->pushButton_load_real_initial_scan, SIGNAL(clicked()), this, SLOT(loadRealInitialScan()));
  connect(ui->pushButton_load_real_scan, SIGNAL(clicked()), this, SLOT(loadRealScan()));
  connect(ui->spinBox_nbv_iteration_count, SIGNAL(valueChanged(int)), this, SLOT(getNbvIterationCount(int)));
  connect(ui->pushButton_one_key_nbv_iteration, SIGNAL(clicked()), this, SLOT(runOneKeyNbvIteration()));
  connect(ui->checkBox_show_init_cameras,SIGNAL(clicked(bool)),this,SLOT(showInitCameras(bool)));
  connect(ui->pushButton_scan, SIGNAL(clicked()), this, SLOT(NBVCandidatesScanByHand()));
  connect(ui->pushButton_initial_scan, SIGNAL(clicked()), this, SLOT(initialScan()));
  connect(ui->horizon_dist, SIGNAL(valueChanged(double)), this, SLOT(getCameraHorizonDist(double)));
  connect(ui->vertical_dist, SIGNAL(valueChanged(double)), this, SLOT(getCameraVerticalDist(double)));
  connect(ui->dist_to_model, SIGNAL(valueChanged(double)), this, SLOT(getCameraDistToModel(double)));
  connect(ui->tableView_scan_candidates, SIGNAL(clicked(QModelIndex)), this, SLOT(showSelectedScannCandidates(QModelIndex)));
  connect(ui->tableView_scan_results, SIGNAL(clicked(QModelIndex)), this, SLOT(showSelectedScannedMesh(QModelIndex)));
  connect(ui->pushButton_merge_with_original, SIGNAL(clicked()), this, SLOT(mergeScannedMeshWithOriginalByHand()));
  connect(ui->doubleSpinBox_merge_confidence_threshold, SIGNAL(valueChanged(double)), this, SLOT(getMergeConfidenceThreshold(double)));
  connect(ui->doubleSpinBox_far_distance, SIGNAL(valueChanged(double)), this, SLOT(getCameraFarDistance(double)));
  connect(ui->doubleSpinBox_near_distance, SIGNAL(valueChanged(double)), this, SLOT(getCameraNearDistance(double)));
  connect(ui->doubleSpinBox_predicted_model_size, SIGNAL(valueChanged(double)), this, SLOT(getPredictedModelSize(double)));
  connect(ui->doubleSpinBox_optimal_plane_width, SIGNAL(valueChanged(double)), this, SLOT(getOptimalPlaneWidth(double)));
  connect(ui->ray_resolution_para, SIGNAL(valueChanged(double)), this, SLOT(getRayResolutionPara(double)));
  
  connect(ui->pushButton_build_grid, SIGNAL(clicked()), this, SLOT(buildGrid()));
  connect(ui->pushButton_propagate, SIGNAL(clicked()), this, SLOT(propagate()));
  connect(ui->pushButton_propagate_one, SIGNAL(clicked()), this, SLOT(propagateOnePoint()));
  connect(ui->pushButton_grid_segment, SIGNAL(clicked()), this, SLOT(gridSegment()));
 
  connect(ui->extract_view_candidates, SIGNAL(clicked()), this, SLOT(extractViewCandidates()));
  connect(ui->pushButton_extract_views_into_bins, SIGNAL(clicked()), this, SLOT(extractViewIntoBins()));
  connect(ui->cluster_view_candidates, SIGNAL(clicked()), this, SLOT(runViewClustering()));

  connect(ui->max_ray_steps, SIGNAL(valueChanged(double)), this, SLOT(getMaxRaySteps(double)));
  connect(ui->view_grid_resolution, SIGNAL(valueChanged(double)), this, SLOT(getGridResolution(double)));
  connect(ui->propagate_one_point_index, SIGNAL(valueChanged(double)), this, SLOT(getPropagateIndex(double)));
    
  connect(ui->use_other_inside_segment,SIGNAL(clicked(bool)),this,SLOT(useOtherInsideSegment(bool)));
  connect(ui->use_confidence_Separation,SIGNAL(clicked(bool)),this,SLOT(useConfidenceSeparation(bool)));
  connect(ui->need_more_overlaps,SIGNAL(clicked(bool)),this,SLOT(needMoreOverlaps(bool)));
  //connect(ui->use_nbv_test1, SIGNAL(clicked(bool)), this, SLOT(useNbvTest1(bool)));
  connect(ui->use_max_propagation, SIGNAL(clicked(bool)), this, SLOT(useMaxConfidencePropagation(bool)));
  connect(ui->doubleSpinBox_bottom_delta, SIGNAL(valueChanged(double)), this, SLOT(getIsoBottomDelta(double)));
  connect(ui->pushButton_set_iso_bottom_confidence, SIGNAL(clicked()), this, SLOT(runSetIsoBottomConfidence()));
  connect(ui->update_view_directions, SIGNAL(clicked()), this, SLOT(runUpdateViewDirections()));
  

  connect(ui->pushButton_setup_initial_scans, SIGNAL(clicked()), this, SLOT(runSetupInitialScanns()));
  connect(ui->step1_run_WLOP, SIGNAL(clicked()), this, SLOT(runStep1WLOP()));
  connect(ui->step2_run_Poisson_Confidence, SIGNAL(clicked()), this, SLOT(runStep2PoissonConfidence()));
  connect(ui->step2_run_Poisson_Confidence_original, SIGNAL(clicked()), this, SLOT(runStep2PoissonConfidenceViaOiginal()));
  connect(ui->step3_run_NBV, SIGNAL(clicked()), this, SLOT(runStep3NBVcandidates()));
  connect(ui->step4_run_New_Scan, SIGNAL(clicked()), this, SLOT(runStep4NewScans()));
  connect(ui->pushButton_wlop_on_scanned_mesh, SIGNAL(clicked()), this, SLOT(runWlopOnScannedMesh()));
  connect(ui->pushButton_ICP, SIGNAL(clicked()), this, SLOT(runICP()));
  connect(ui->pushButton_remove_sample_outliers, SIGNAL(clicked()), this, SLOT(runRemoveSampleOutliers()));
  connect(ui->pushButton_remove_low_confidence_samples, SIGNAL(clicked()), this, SLOT(runRemoveSamplesWithLowConfidence()));
  connect(ui->pushButton_add_samples_to_original, SIGNAL(clicked()), this, SLOT(runAddSamplesToOiriginal()));

  connect(ui->rotate_center_X,SIGNAL(valueChanged(double)),this,SLOT(getRotateCenterX(double)));
  connect(ui->rotate_center_Y,SIGNAL(valueChanged(double)),this,SLOT(getRotateCenterY(double)));
  connect(ui->rotate_center_Z,SIGNAL(valueChanged(double)),this,SLOT(getRotateCenterZ(double)));
  connect(ui->rotate_normal_X,SIGNAL(valueChanged(double)),this,SLOT(getRotateNormalX(double)));
  connect(ui->rotate_normal_Y,SIGNAL(valueChanged(double)),this,SLOT(getRotateNormalY(double)));
  connect(ui->rotate_normal_Z,SIGNAL(valueChanged(double)),this,SLOT(getRotateNormalZ(double)));
  connect(ui->rotate_step,SIGNAL(valueChanged(double)),this,SLOT(getRotateStep(double)));
  connect(ui->rotate_angle,SIGNAL(valueChanged(double)),this,SLOT(getRotateAngle(double)));

  connect(ui->pushButton_rotate,SIGNAL(clicked()),this,SLOT(rotateStep()));
  connect(ui->pushButton_rotate_around,SIGNAL(clicked()),this,SLOT(rotateAnimation()));
  connect(ui->wlop_snap_shot_each_iteration,SIGNAL(clicked(bool)),this,SLOT(needSnapShotEachIteration(bool)));
  connect(ui->wlop_snapshot_index,SIGNAL(valueChanged(double)),this,SLOT(getSnapShotIndex(double)));
  connect(ui->pushButton_slice_animation, SIGNAL(clicked()), this, SLOT(sliceAnimation()));

  connect(ui->pushButton_pick_center, SIGNAL(clicked()), this, SLOT(moveTranslation()));

  
}

bool CameraParaDlg::initWidgets()
{
  ui->doubleSpinBox_view_prune_confidence_threshold->setValue(m_paras->nbv.getDouble("View Prune Confidence Threshold"));
  ui->doubleSpinBox_merge_confidence_threshold->setValue(m_paras->camera.getDouble("Merge Confidence Threshold"));
  ui->spinBox_nbv_iteration_count->setValue(m_paras->nbv.getInt("NBV Iteration Count"));
  ui->horizon_dist->setValue(m_paras->camera.getDouble("Camera Horizon Dist"));
  ui->vertical_dist->setValue(m_paras->camera.getDouble("Camera Vertical Dist"));
  ui->view_grid_resolution->setValue(m_paras->nbv.getDouble("View Grid Resolution"));
  ui->max_ray_steps->setValue(m_paras->nbv.getDouble("Max Ray Steps Para"));
  
  ui->doubleSpinBox_far_distance->setValue(m_paras->camera.getDouble("Camera Far Distance"));
  ui->doubleSpinBox_near_distance->setValue(m_paras->camera.getDouble("Camera Near Distance"));
  ui->doubleSpinBox_predicted_model_size->setValue(m_paras->camera.getDouble("Predicted Model Size"));
  ui->doubleSpinBox_optimal_plane_width->setValue(m_paras->camera.getDouble("Optimal Plane Width"));
  ui->propagate_one_point_index->setValue(m_paras->nbv.getDouble("Propagate One Point Index"));
  ui->ray_resolution_para->setValue(m_paras->nbv.getDouble("Ray Resolution Para"));

  Qt::CheckState state = m_paras->nbv.getBool("Test Other Inside Segment") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->use_other_inside_segment->setCheckState(state);

  state = m_paras->nbv.getBool("Use Confidence Separation") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->use_confidence_Separation->setCheckState(state);

  state = m_paras->nbv.getBool("Need Update Direction With More Overlaps") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->need_more_overlaps->setCheckState(state);

  state = m_paras->glarea.getBool("SnapShot Each Iteration") ? (Qt::CheckState::Checked): (Qt::CheckState::Unchecked);
  ui->wlop_snap_shot_each_iteration->setCheckState(state);

  ui->rotate_center_X->setValue(area->rotate_pos.X());
  ui->rotate_center_Y->setValue(area->rotate_pos.Y());
  ui->rotate_center_Z->setValue(area->rotate_pos.Z());
  ui->rotate_normal_X->setValue(area->rotate_normal.X());
  ui->rotate_normal_Y->setValue(area->rotate_normal.Y());
  ui->rotate_normal_Z->setValue(area->rotate_normal.Z());
  ui->rotate_step->setValue(area->rotate_delta);
  ui->rotate_angle->setValue(area->rotate_angle);

  ui->wlop_snapshot_index->setValue(m_paras->glarea.getDouble("Snapshot Index"));
  //state = m_paras->nbv.getBool("Use NBV Test1") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  //ui->use_nbv_test1->setCheckState(state);

  state = m_paras->nbv.getBool("Use Max Propagation") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->use_max_propagation->setCheckState(state);

  updateTableViewNBVCandidate();
  updateTabelViewScanResults();
  update();
  repaint();
	return true;
}

CameraParaDlg::~CameraParaDlg()
{
	delete ui;
	ui = NULL;
	area = NULL;
	m_paras = NULL;
}

void CameraParaDlg::setFrameConent()
{
  if(layout()) delete layout();
  QGridLayout * vLayout = new QGridLayout(this);
  vLayout->setAlignment(Qt::AlignTop);
  setLayout(vLayout);

  showNormal();
  adjustSize();
}

void CameraParaDlg::updateTableViewNBVCandidate()
{
  //init nbv scan candidates table view
  //add table header
  QStandardItemModel *model = new QStandardItemModel(); 
  model->setColumnCount(2); 
  model->setHeaderData(0,Qt::Horizontal,QString::fromLocal8Bit("position")); 
  model->setHeaderData(1,Qt::Horizontal,QString::fromLocal8Bit("direction"));
  //set table property
  ui->tableView_scan_candidates->setModel(model);
  //select the whole row
  ui->tableView_scan_candidates->setSelectionBehavior(QAbstractItemView::SelectRows);
  ui->tableView_scan_candidates->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);
  ui->tableView_scan_candidates->horizontalHeader()->setResizeMode(0, QHeaderView::Stretch);
  ui->tableView_scan_candidates->horizontalHeader()->setResizeMode(1, QHeaderView::Stretch);
  ui->tableView_scan_candidates->setColumnWidth(0, 100);
  ui->tableView_scan_candidates->setColumnWidth(1, 100);

  //set table contents
  vector<ScanCandidate> *nbv_candidates = area->dataMgr.getScanCandidates();
  int i = 0;
  for (vector<ScanCandidate>::iterator it = nbv_candidates->begin(); 
    it != nbv_candidates->end(); ++it, ++i)
  {
    QString pos, direction;
    pos.sprintf("(%4.3f, %4.3f, %4.3f)", it->first.X(), it->first.Y(), it->first.Z());
    direction.sprintf("(%4.3f, %4.3f, %4.3f)", it->second.X(), it->second.Y(), it->second.Z());
    model->setItem(i, 0, new QStandardItem(pos));
    //model->item(i, 0)->setForeground(QBrush(QColor(255, 0, 0)));
    model->item(i, 0)->setTextAlignment(Qt::AlignCenter);
    model->setItem(i, 1, new QStandardItem(direction));
    model->item(i, 1)->setTextAlignment(Qt::AlignCenter);
  }
}

void CameraParaDlg::updateTabelViewScanResults()
{
  //init tableView_scan_results
  //add table header
  QStandardItemModel *model = new QStandardItemModel(); 
  model->setColumnCount(1); 
  model->setHeaderData(0,Qt::Horizontal,QString::fromLocal8Bit("scanned mesh")); 
  //set table property
  ui->tableView_scan_results->setContextMenuPolicy(Qt::CustomContextMenu);
  ui->tableView_scan_results->setModel(model);
  ui->tableView_scan_results->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);
  ui->tableView_scan_results->horizontalHeader()->setResizeMode(0, QHeaderView::Stretch);
  ui->tableView_scan_results->setColumnWidth(0, 100);
  //set table contents
  vector<CMesh* > *rs = area->dataMgr.getScannedResults();
  vector<CMesh* >::iterator it = (*rs).begin();
  int i = 0;
  for (; it != (*rs).end(); ++it, ++i)
  {
    QString scanned_mesh_str;
    scanned_mesh_str.sprintf("Scanned mesh %d", i+1);
    model->setItem(i, 0, new QStandardItem(scanned_mesh_str));
    model->item(i, 0)->setTextAlignment(Qt::AlignLeft);
  }
}

void CameraParaDlg::initialScan()
{
  global_paraMgr.camera.setValue("Run Initial Scan", BoolValue(true));
  area->runCamera();
  global_paraMgr.camera.setValue("Run Initial Scan", BoolValue(false));

  updateTableViewNBVCandidate();
}

void CameraParaDlg::NBVCandidatesScan()
{
  //scan only the candidates those are chosen
  /*vector<ScanCandidate> *sc = area->dataMgr.getScanCandidates();
  vector<ScanCandidate> select_scan_candidates;
  QModelIndexList sil = ui->tableView_scan_candidates->selectionModel()->selectedRows();
  for (int i = 0; i < sil.size(); ++i)
  {
  int row = sil[i].row();
  select_scan_candidates.push_back((*sc)[row]);
  }
  sc->clear();
  std::copy(select_scan_candidates.begin(), select_scan_candidates.end(), std::back_inserter(*sc));*/
  
  global_paraMgr.camera.setValue("Run NBV Scan", BoolValue(true));
  area->runCamera();
  global_paraMgr.camera.setValue("Run NBV Scan", BoolValue(false));

  area->updateGL();
  updateTableViewNBVCandidate();
  updateTabelViewScanResults();
}

void CameraParaDlg::NBVCandidatesScanByHand()
{
  //scan only the candidates those are chosen
  vector<ScanCandidate> *sc = area->dataMgr.getScanCandidates();
  vector<ScanCandidate> select_scan_candidates;
  QModelIndexList sil = ui->tableView_scan_candidates->selectionModel()->selectedRows();
  for (int i = 0; i < sil.size(); ++i)
  {
  int row = sil[i].row();
  select_scan_candidates.push_back((*sc)[row]);
  }
  sc->clear();
  std::copy(select_scan_candidates.begin(), select_scan_candidates.end(), std::back_inserter(*sc));
  
  global_paraMgr.camera.setValue("Run NBV Scan", BoolValue(true));
  area->runCamera();
  global_paraMgr.camera.setValue("Run NBV Scan", BoolValue(false));

  area->updateGL();
  updateTableViewNBVCandidate();
  updateTabelViewScanResults();
}

void CameraParaDlg::loadRealScan()
{
  QString file_location = QFileDialog::getExistingDirectory(this, "choose a directory...", "",QFileDialog::ShowDirsOnly);
  if (!file_location.size()) 
    return;

  QDir dir(file_location);
  if (!dir.exists()) 
    return;

  dir.setFilter(QDir::Files);
  dir.setSorting(QDir::Name);
  QFileInfoList list = dir.entryInfoList();

  CMesh *original = area->dataMgr.getCurrentOriginal();
  vector< CMesh* > *sc = area->dataMgr.getScannedResults();

  for (int i = 0; i < list.size(); ++i)
  {
    QFileInfo fileInfo = list.at(i);
    QString f_name = fileInfo.fileName();

    if (!f_name.endsWith(".ply"))
      continue;
  
    f_name = file_location + "\\" + f_name;
    CMesh *real_scan = new CMesh;
    int mask = tri::io::Mask::IOM_VERTCOORD + tri::io::Mask::IOM_VERTNORMAL;
    tri::io::ImporterPLY<CMesh>::Open(*real_scan, f_name.toAscii().data(), mask);
    sc->push_back(real_scan);

    if (original == NULL)
      cout << "Empty original, real scans unable to ICP on original!" <<endl;
    else
      GlobalFun::computeICP(original, real_scan);
  }
}

void CameraParaDlg::showCandidateIndex()
{
  global_paraMgr.nbv.setValue("Run Compute View Candidate Index", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Compute View Candidate Index", BoolValue(false));
}

void CameraParaDlg::loadRealInitialScan()
{
  QString file_location = QFileDialog::getExistingDirectory(this, "choose a directory...", "",QFileDialog::ShowDirsOnly);
  if (!file_location.size()) 
    return;

  QDir dir(file_location);
  if (!dir.exists()) 
    return;

  dir.setFilter(QDir::Files);
  dir.setSorting(QDir::Name);
  QFileInfoList list = dir.entryInfoList();

  CMesh *original = area->dataMgr.getCurrentOriginal();

  for (int i = 0; i < list.size(); ++i)
  {
    QFileInfo fileInfo = list.at(i);
    QString f_name = fileInfo.fileName();

    if (!f_name.endsWith(".ply"))
      continue;

    f_name = file_location + "\\" + f_name;
    CMesh initial_scan;
    int mask = tri::io::Mask::IOM_VERTCOORD + tri::io::Mask::IOM_VERTNORMAL;
    if (i == 0)
    {
      area->dataMgr.loadPlyToOriginal(f_name);
    }else
    {
      CMesh* iso_points = area->dataMgr.getCurrentIsoPoints();
      tri::io::ImporterPLY<CMesh>::Open(*iso_points, f_name.toAscii().data(), mask);
      GlobalFun::computeICP(original, iso_points);
      //tri::io::ImporterPLY<CMesh>::Open(initial_scan, f_name.toAscii().data(), mask);
      //GlobalFun::computeICP(original, &initial_scan);
    }
  }
}

void CameraParaDlg::virtualScan()
{
  global_paraMgr.camera.setValue("Run Virtual Scan", BoolValue(true));
  area->runCamera();
  global_paraMgr.camera.setValue("Run Virtual Scan", BoolValue(false));
}

void CameraParaDlg::showInitCameras(bool is_show)
{
  if (is_show)
  {
    global_paraMgr.camera.setValue("Is Init Camera Show", BoolValue(true));
    vector<ScanCandidate> *seletedViewCameras  = area->dataMgr.getSelectedScanCandidates();
    vector<ScanCandidate> *initViewCameras = area->dataMgr.getInitCameraScanCandidates();
    seletedViewCameras->clear();

    std::copy(initViewCameras->begin(), initViewCameras->end(), std::back_inserter((*seletedViewCameras)));
  }else
  {
    global_paraMgr.camera.setValue("Is Init Camera Show", BoolValue(false));
    vector<ScanCandidate> *seletedViewCameras  = area->dataMgr.getSelectedScanCandidates();
    seletedViewCameras->clear();
  }
}

void CameraParaDlg::useOtherInsideSegment(bool _val)
{
  global_paraMgr.nbv.setValue("Test Other Inside Segment", BoolValue(_val));
}

void CameraParaDlg::useConfidenceSeparation(bool _val)
{
  global_paraMgr.nbv.setValue("Use Confidence Separation", BoolValue(_val));
  cout << "Use Confidence Separation" << endl;
}

void CameraParaDlg::needMoreOverlaps(bool _val)
{
  global_paraMgr.nbv.setValue("Need Update Direction With More Overlaps", BoolValue(_val));
  cout << "Need Update Direction With More Overlaps" << endl;
}

void CameraParaDlg::useMaxConfidencePropagation(bool _val)
{
  global_paraMgr.nbv.setValue("Use Max Propagation", BoolValue(_val));
  cout << "Use Max Propagation" << endl;
}

void CameraParaDlg::useNbvTest1(bool _val)
{
  global_paraMgr.nbv.setValue("Use NBV Test1", BoolValue(_val));
  cout << "Use NBV Test1" << endl;
}

void CameraParaDlg::showSelectedScannCandidates(QModelIndex index)
{
  ui->checkBox_show_init_cameras->setChecked(false);
  //draw camera at selected position
  QModelIndexList sil = ui->tableView_scan_candidates->selectionModel()->selectedRows();
  vector<ScanCandidate> *seletedViewCameras  = area->dataMgr.getSelectedScanCandidates();
  vector<ScanCandidate> *allScanCandidates = area->dataMgr.getScanCandidates();

  seletedViewCameras->clear();
  //add selected scan candidates into selected vector
  for (int i = 0; i < sil.size(); ++i)
  {
    int row = sil[i].row();
    seletedViewCameras->push_back((*allScanCandidates)[row]);
  }
}

void CameraParaDlg::showSelectedScannedMesh(QModelIndex index)
{
  ui->checkBox_show_init_cameras->setChecked(false);
  //get selected rows
  QModelIndexList sil = ui->tableView_scan_results->selectionModel()->selectedRows();
  vector<CMesh* > *scanned_results = area->dataMgr.getScannedResults();
 
  int row_of_mesh = 0;
  vector<ScanCandidate> *seletedViewCameras  = area->dataMgr.getSelectedScanCandidates();
  vector<ScanCandidate> *allScanCandidates = area->dataMgr.getScanCandidates();
  seletedViewCameras->clear();

  for (vector<CMesh* >::iterator it = scanned_results->begin(); 
    it != scanned_results->end(); ++it, ++row_of_mesh)
  {
    if ((*it)->vert.empty()) continue;
    //set default to invisible
    (*it)->vert[0].is_scanned_visible = false;

    for (int i = 0; i < sil.size(); ++i)
    {
      int row = sil[i].row();
      //if the mesh is chosen, change to visible
      if (row == row_of_mesh) 
       {
          (*it)->vert[0].is_scanned_visible = true;
          seletedViewCameras->push_back((*allScanCandidates)[row]);
      }
    }
  }
}


void CameraParaDlg::mergeScannedMeshWithOriginal()
{
  /*QModelIndexList sil = ui->tableView_scan_results->selectionModel()->selectedRows();
  if (sil.isEmpty()) return;*/

  CMesh* original = area->dataMgr.getCurrentOriginal();
  vector<CMesh* > *scanned_results = area->dataMgr.getScannedResults();
  double merge_confidence_threshold = global_paraMgr.camera.getDouble("Merge Confidence Threshold");
  int merge_pow = static_cast<int>(global_paraMgr.nbv.getDouble("Merge Probability Pow"));

  //wsh added 12-24
  double radius_threshold = global_paraMgr.wLop.getDouble("CGrid Radius");
  double radius2 = radius_threshold * radius_threshold;
  double iradius16 = -4/radius2;

  double sigma = global_paraMgr.norSmooth.getDouble("Sharpe Feature Bandwidth Sigma");
  double sigma_threshold = pow(max(1e-8,1-cos(sigma/180.0*3.1415926)), 2);
  CMesh* iso_points = area->dataMgr.getCurrentIsoPoints();
  //end wsh added

  /*int row_of_mesh = 0;*/
  for (vector<CMesh* >::iterator it = scanned_results->begin(); 
    it != scanned_results->end(); ++it /*++row_of_mesh*/)
  {
    if ((*it)->vert.empty()) continue;

    // for (int i = 0; i < sil.size(); ++i)
    // {
    //int row = sil[i].row();
    //combine the selected mesh with original
    // if (row == row_of_mesh) 
    // {
    //make the merged mesh invisible
    (*it)->vert[0].is_scanned_visible = false;
    //compute new scanned mesh's iso neighbors

    //wsh updated 12-24
    //GlobalFun::computeAnnNeigbhors(area->dataMgr.getCurrentIsoPoints()->vert, (*it)->vert, 1, false, "runNewScannedMeshNearestIsoPoint");
    Timer time;
    time.start("Sample ISOpoints Neighbor Tree!!");
    GlobalFun::computeBallNeighbors((*it), area->dataMgr.getCurrentIsoPoints(), 
      radius_threshold, (*it)->bbox);
    time.end();
    //end wsh updated

    cout<<"Before merge with original: " << original->vert.size() <<endl;
    cout<<"scanned mesh num: "<<(*it)->vert.size() <<endl;
    int skip_num = 0;

    vector<double> v_confidence;
    double max_confidence = 0.0f;
    double min_confidence = BIG;

    for (int k = 0; k < (*it)->vert.size(); ++k)
    {
      //wsh updated 12-24
      CVertex& v = (*it)->vert[k];
      //add or not
      //CVertex &nearest = area->dataMgr.getCurrentIsoPoints()->vert[v.neighbors[0]];
      double sum_confidence = 0.0;
      double sum_w = 0.0;

      for(int ni = 0; ni < v.original_neighbors.size(); ni++)
      {
        CVertex& t = iso_points->vert[v.original_neighbors[ni]];

        double dist2 = GlobalFun::computeEulerDistSquare(v.P(), t.P());
        double dist_diff = exp(dist2 * iradius16);
        //double normal_diff = exp(-pow(1-v.N()*t.N(), 2)/sigma_threshold);
        double normal_diff = 1.0;
        double w = dist_diff * normal_diff;
        //double w = 1.0f;
        sum_confidence += w * t.eigen_confidence;
        sum_w += 1;
      }

      if (v.original_neighbors.size() > 0 )
        sum_confidence /= sum_w;

      v_confidence.push_back(sum_confidence);

      max_confidence = sum_confidence > max_confidence ? sum_confidence : max_confidence;
      min_confidence = sum_confidence < min_confidence ? sum_confidence : min_confidence;
    }

    //normalize the confidence
    for (vector<double>::iterator it = v_confidence.begin(); it != v_confidence.end(); ++it)
      *it = (*it - min_confidence) / (max_confidence - min_confidence);

    int index = original->vert.empty() ? 0 : original->vert.back().m_index;
    for (int k = 0;  k < (*it)->vert.size(); ++k)
    {
      CVertex& v = (*it)->vert[k];

      if (k < 10)
        cout<<"sum_confidence: " << v_confidence[k] <<endl;

      if (v_confidence[k] > merge_confidence_threshold 
      || (1.0f * rand() / (RAND_MAX+1.0) > pow((1 - v_confidence[k]), merge_pow)))
      {
      v.is_ignore = true;
      skip_num++; 
      continue;
      }

      CVertex new_v;
      new_v.m_index = ++index;
      new_v.is_original = true;
      new_v.P() = v.P();
      new_v.N() = v.N();
      original->vert.push_back(new_v);
      original->bbox.Add(new_v.P());
    } 
    original->vn = original->vert.size();
    cout<<"skip points num:" <<skip_num <<endl;
    cout<<"After merge with original: " << original->vert.size() <<endl <<endl;
        //set combined row unable to be chosen
        //ui->tableView_scan_candidates->setRowHidden(row, true);
        //ui->tableView_scan_results->setRowHidden(row, true);
      //}
   // }//end for sil
  }
}

void CameraParaDlg::mergeScannedMeshWithOriginalByHand()
{
  QModelIndexList sil = ui->tableView_scan_results->selectionModel()->selectedRows();
  if (sil.isEmpty()) return;

  CMesh* original = area->dataMgr.getCurrentOriginal();
  vector<CMesh* > *scanned_results = area->dataMgr.getScannedResults();
  double merge_confidence_threshold = global_paraMgr.camera.getDouble("Merge Confidence Threshold");
  int merge_pow = static_cast<int>(global_paraMgr.nbv.getDouble("Merge Probability Pow"));
  cout<< "merge_confidence_threshold: " <<merge_confidence_threshold <<endl;

  //wsh added 12-24
  double radius_threshold = global_paraMgr.wLop.getDouble("CGrid Radius");
  double radius2 = radius_threshold * radius_threshold;
  double iradius16 = -4/radius2;

  double sigma = global_paraMgr.norSmooth.getDouble("Sharpe Feature Bandwidth Sigma");
  double sigma_threshold = pow(max(1e-8,1-cos(sigma/180.0*3.1415926)), 2);
  CMesh* iso_points = area->dataMgr.getCurrentIsoPoints();
  //end wsh added

  int row_of_mesh = 0;
  for (vector<CMesh* >::iterator it = scanned_results->begin(); 
    it != scanned_results->end(); ++it, ++row_of_mesh)
  {
    if ((*it)->vert.empty()) continue;

    for (int i = 0; i < sil.size(); ++i)
    {
      int row = sil[i].row();
      //combine the selected mesh with original
      if (row == row_of_mesh) 
      {
        (*it)->vert[0].is_scanned_visible = false;
        //compute new scanned mesh's iso neighbors

        //wsh updated 12-24
        //GlobalFun::computeAnnNeigbhors(area->dataMgr.getCurrentIsoPoints()->vert, (*it)->vert, 1, false, "runNewScannedMeshNearestIsoPoint");
        Timer time;
        time.start("Sample ISOpoints Neighbor Tree!!");
        GlobalFun::computeBallNeighbors((*it), area->dataMgr.getCurrentIsoPoints(), 
          radius_threshold, (*it)->bbox);
        time.end();
        //end wsh updated

        cout<<"Before merge with original: " << original->vert.size() <<endl;
        cout<<"scanned mesh num: "<<(*it)->vert.size() <<endl;
                
        int skip_num = 0;

        vector<double> v_confidence;
        double max_confidence = 0.0f;
        double min_confidence = BIG;

        for (int k = 0; k < (*it)->vert.size(); ++k)
        {
          //wsh updated 12-24
          CVertex& v = (*it)->vert[k];
          //add or not
          //CVertex &nearest = area->dataMgr.getCurrentIsoPoints()->vert[v.neighbors[0]];
          double sum_confidence = 0.0;
          double sum_w = 0.0;

          for(int ni = 0; ni < v.original_neighbors.size(); ni++)
          {
            CVertex& t = iso_points->vert[v.original_neighbors[ni]];

            double dist2 = GlobalFun::computeEulerDistSquare(v.P(), t.P());
            double dist_diff = exp(dist2 * iradius16);
            //double normal_diff = exp(-pow(1-v.N()*t.N(), 2)/sigma_threshold);
            double normal_diff = 1.0;
            double w = dist_diff * normal_diff;
            //double w = 1.0f;
            sum_confidence += w * t.eigen_confidence;
            sum_w += 1;
          }

          if (v.original_neighbors.size() > 0 )
            sum_confidence /= sum_w;

          v_confidence.push_back(sum_confidence);

          max_confidence = sum_confidence > max_confidence ? sum_confidence : max_confidence;
          min_confidence = sum_confidence < min_confidence ? sum_confidence : min_confidence;
        }

        //normalize the confidence
        for (vector<double>::iterator it = v_confidence.begin(); it != v_confidence.end(); ++it)
          *it = (*it - min_confidence) / (max_confidence - min_confidence);

        int index = original->vert.empty() ? 0 : original->vert.back().m_index;
        for (int k = 0;  k < (*it)->vert.size(); ++k)
        {
          CVertex& v = (*it)->vert[k];

          if (k < 10)
            cout<<"sum_confidence: " << v_confidence[k] <<endl;

          if (v_confidence[k] > merge_confidence_threshold 
          || (1.0f * rand() / (RAND_MAX+1.0) > pow((1 - v_confidence[k]), merge_pow)))
          {
          v.is_ignore = true;
          skip_num++; 
          continue;
          }

          CVertex new_v;
          new_v.m_index = ++index;
          new_v.is_original = true;
          new_v.P() = v.P();
          new_v.N() = v.N();
          new_v.C().SetRGB(255, 0, 0);
          original->vert.push_back(new_v);
          original->bbox.Add(new_v.P());
        } 
        original->vn = original->vert.size();
        cout<<"skip points num:" <<skip_num <<endl;
        cout<<"After merge with original: " << original->vert.size() <<endl <<endl;
        //set combined row unable to be chosen
        //ui->tableView_scan_candidates->setRowHidden(row, true);
        //ui->tableView_scan_results->setRowHidden(row, true);
      }
    }
  }
}

void CameraParaDlg::getNbvIterationCount(int _val)
{
  global_paraMgr.nbv.setValue("NBV Iteration Count", IntValue(_val));
}

void CameraParaDlg::getCameraHorizonDist(double _val)
{
  global_paraMgr.camera.setValue("Camera Horizon Dist", DoubleValue(_val));
}

void CameraParaDlg::getCameraVerticalDist(double _val)
{
  global_paraMgr.camera.setValue("Camera Vertical Dist", DoubleValue(_val));
}

void CameraParaDlg::getCameraDistToModel(double _val)
{
  global_paraMgr.camera.setValue("Camera Dist To Model", DoubleValue(_val));
}

void CameraParaDlg::getViewPruneConfidenceThreshold(double _val)
{
  global_paraMgr.nbv.setValue("View Prune Confidence Threshold", DoubleValue(_val));
}

void CameraParaDlg::getGridResolution(double _val)
{
  global_paraMgr.nbv.setValue("View Grid Resolution", DoubleValue(_val));
}

void CameraParaDlg::getMaxRaySteps(double _val)
{
  global_paraMgr.nbv.setValue("Max Ray Steps Para", DoubleValue(_val));
}

void CameraParaDlg::getIsoBottomDelta(double _val)
{
  global_paraMgr.nbv.setValue("Iso Bottom Delta", DoubleValue(_val));
}
void CameraParaDlg::getMergeConfidenceThreshold(double _val)
{
  global_paraMgr.camera.setValue("Merge Confidence Threshold", DoubleValue(_val));
}

void CameraParaDlg::getCameraFarDistance(double _val)
{
  global_paraMgr.camera.setValue("Camera Far Distance", DoubleValue(_val));
}

void CameraParaDlg::getCameraNearDistance(double _val)
{
  global_paraMgr.camera.setValue("Camera Near Distance", DoubleValue(_val));
}

void CameraParaDlg::getPredictedModelSize(double _val)
{
  global_paraMgr.camera.setValue("Predicted Model Size", DoubleValue(_val));
}

void CameraParaDlg::getOptimalPlaneWidth(double _val)
{
  global_paraMgr.camera.setValue("Optimal Plane Width", DoubleValue(_val));
}

void CameraParaDlg::getPropagateIndex(double _val)
{
  global_paraMgr.nbv.setValue("Propagate One Point Index", DoubleValue(_val));
}

void CameraParaDlg::getRayResolutionPara(double _val)
{
  global_paraMgr.nbv.setValue("Ray Resolution Para", DoubleValue(_val));
}


void
CameraParaDlg::buildGrid()
{
  global_paraMgr.nbv.setValue("Run Build Grid", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Build Grid", BoolValue(false));
}

void
CameraParaDlg::propagate()
{
  global_paraMgr.nbv.setValue("Run Propagate", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Propagate", BoolValue(false));
}

void CameraParaDlg::propagateOnePoint()
{
  global_paraMgr.nbv.setValue("Run Propagate", BoolValue(true));
  global_paraMgr.nbv.setValue("Run Propagate One Point", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Propagate One Point", BoolValue(false));
  global_paraMgr.nbv.setValue("Run Propagate", BoolValue(false));

}

void CameraParaDlg::gridSegment()
{
  global_paraMgr.nbv.setValue("Run Build Grid", BoolValue(true));
  global_paraMgr.nbv.setValue("Run Grid Segment", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Grid Segment", BoolValue(false));
  global_paraMgr.nbv.setValue("Run Build Grid", BoolValue(false));

}

void CameraParaDlg::extractViewCandidates()
{
  global_paraMgr.nbv.setValue("Run Viewing Extract", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Viewing Extract", BoolValue(false));
}

void CameraParaDlg::extractViewIntoBins()
{
  global_paraMgr.nbv.setValue("Run Extract Views Into Bins", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Extract Views Into Bins", BoolValue(false));
}

void CameraParaDlg::runViewClustering()
{
  global_paraMgr.nbv.setValue("Run Viewing Clustering", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Viewing Clustering", BoolValue(false));
}

void CameraParaDlg::runViewPrune()
{
  global_paraMgr.nbv.setValue("Run View Prune", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run View Prune", BoolValue(false));
}

void CameraParaDlg::runSetIsoBottomConfidence()
{
  global_paraMgr.nbv.setValue("Run Set Iso Bottom Confidence", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Set Iso Bottom Confidence", BoolValue(false));
}

void CameraParaDlg::runUpdateViewDirections()
{
  global_paraMgr.nbv.setValue("Run Update View Directions", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run Update View Directions", BoolValue(false));
}

void CameraParaDlg::runSetupInitialScanns()
{
	global_paraMgr.camera.setValue("Run Initial Scan", BoolValue(true));
	area->runCamera();
	global_paraMgr.camera.setValue("Run Initial Scan", BoolValue(false));
	updateTableViewNBVCandidate();
}

void CameraParaDlg::runStep1WLOP()
{
  area->dataMgr.downSamplesByNum();
  //area->initSetting();

  global_paraMgr.wLop.setValue("Run One Key WLOP", BoolValue(true));
  area->runWlop();
  global_paraMgr.wLop.setValue("Run One Key WLOP", BoolValue(false));

  int knn = global_paraMgr.norSmooth.getInt("PCA KNN");
  CMesh* samples = area->dataMgr.getCurrentSamples();
  vcg::NormalExtrapolation<vector<CVertex> >::ExtrapolateNormals(samples->vert.begin(), samples->vert.end(), knn, -1);
  area->dataMgr.recomputeQuad();
  GlobalFun::removeOutliers(samples, global_paraMgr.data.getDouble("CGrid Radius"), 0.001);
}

void CameraParaDlg::runStep2PoissonConfidence()
{
  global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(true));
  global_paraMgr.poisson.setValue("Run Extract MC Points", BoolValue(true));
  global_paraMgr.poisson.setValue("Run One Key PoissonConfidence", BoolValue(true));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run Extract MC Points", BoolValue(false));
  global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(false));
  global_paraMgr.poisson.setValue("Run One Key PoissonConfidence", BoolValue(false));
}

void CameraParaDlg::runStep2PoissonConfidenceViaOiginal()
{
  global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(true));
  global_paraMgr.poisson.setValue("Run One Key PoissonConfidence", BoolValue(true));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run One Key PoissonConfidence", BoolValue(false));
  global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(false));
}

void CameraParaDlg::runStep3NBVcandidates()
{
  global_paraMgr.nbv.setValue("Run One Key NBV", BoolValue(true));
  area->runNBV();
  global_paraMgr.nbv.setValue("Run One Key NBV", BoolValue(false));
  updateTableViewNBVCandidate();
  ui->tableView_scan_results->clearSpans();
}

void CameraParaDlg::runStep4NewScans()
{
  global_paraMgr.camera.setValue("Run One Key NewScans", BoolValue(true));
  area->runCamera();
  global_paraMgr.camera.setValue("Run One Key NewScans", BoolValue(false));
  updateTabelViewScanResults();
}

void CameraParaDlg::runWlopOnScannedMesh()
{
  vector< CMesh* > scanned_results = area->dataMgr.scanned_results;
  vector< CMesh* >::iterator it = scanned_results.begin();
  for (; it != scanned_results.end(); ++it)
  {
    CMesh *temperal_sample = new CMesh;
    CMesh *temperal_original = *it;
    GlobalFun::downSample(temperal_sample, temperal_original, 0.2);

    area->dataMgr.temperal_original = *it;
    area->dataMgr.temperal_sample = temperal_sample;
    global_paraMgr.wLop.setValue("Run Wlop On Scanned Mesh", BoolValue(true));
    area->runWlop();
    global_paraMgr.wLop.setValue("Run Wlop On Scanned Mesh", BoolValue(false));
    //we should deal with wlop result, ICP with 
    GlobalFun::computeICP(area->dataMgr.getCurrentSamples(), area->dataMgr.getCurrentTemperalSamples());
   
    delete temperal_sample;
  }
}



void
CameraParaDlg::runOneKeyNbvIteration()
{
  QString file_location = QFileDialog::getExistingDirectory(this, "choose a directory...", "",QFileDialog::ShowDirsOnly);
  if (!file_location.size()) return;
  
  QString s_log = "\\log.txt";
  s_log = file_location + s_log;
  ofstream log;
  log.open(s_log.toAscii().data());
  cout.rdbuf(log.rdbuf());

  QString para = "\\parameter.para";
  para = file_location + para;
  area->dataMgr.saveParameters(para);

  int iteration_cout = global_paraMgr.nbv.getInt("NBV Iteration Count");
  CMesh *original = area->dataMgr.getCurrentOriginal();
  for (int ic = 0; ic < iteration_cout; ++ic)
  {
    //save original
    QString s_original;
    s_original.sprintf("\\%d_original.ply", ic);
    s_original = file_location + s_original;
    area->dataMgr.savePly(s_original, *area->dataMgr.getCurrentOriginal());

    //compute normal on original
    vector<Point3f> before_normal;
    for (int i = 0; i < original->vert.size(); ++i)
      before_normal.push_back(original->vert[i].N()); 

    int knn = global_paraMgr.norSmooth.getInt("PCA KNN");
    vcg::tri::PointCloudNormal<CMesh>::Param pca_para;
    pca_para.fittingAdjNum = knn;
    //fixme: a debug error in compute
    vcg::tri::PointCloudNormal<CMesh>::Compute(*original, pca_para, NULL);

    for (int i = 0; i < original->vert.size(); ++i)
    {
      if (before_normal[i] * original->vert[i].N() < 0.0f)
        original->vert[i].N() *= -1;
    }

    //save normalized original
    QString s_normal_original;
    s_normal_original.sprintf("\\%d_normal_original.ply", ic);
    s_normal_original = file_location + s_normal_original;
    area->dataMgr.savePly(s_normal_original, *area->dataMgr.getCurrentOriginal());

    //compute radius
    area->dataMgr.downSamplesByNum();
    area->initSetting();

    cout<<"begin to run poisson confidence" <<endl;
    runStep2PoissonConfidence();
    cout<<"end run poisson confidence" <<endl;
    //save poisson surface "copy poisson_out.ply file_location\\%d_poisson_out.ply"
    cout<<"begin to copy poisson_surface" <<endl;
    QString s_poisson_surface;
    s_poisson_surface.sprintf("\\%d_poisson_out.ply", ic);
    QString s_cmd_copy_poisson = "copy poisson_out.ply ";
    s_cmd_copy_poisson += file_location;
    s_cmd_copy_poisson += s_poisson_surface;
    cout << s_cmd_copy_poisson.toStdString() <<endl;
    system(s_cmd_copy_poisson.toAscii().data());
    cout<<"end to copy poisson_surface" <<endl;
    //save iso-skel and view
    QString s_iso;
    s_iso.sprintf("\\%d_iso.skel", ic);
    s_iso = file_location + s_iso;
    area->dataMgr.saveSkeletonAsSkel(s_iso);
    s_iso.replace(".skel", ".View");
    area->saveView(s_iso);
    //save iso dat and raw
    s_iso.replace(".View", ".raw");
    global_paraMgr.poisson.setValue("Run Normalize Field Confidence", BoolValue(true));  
    area->runPoisson();
    global_paraMgr.poisson.setValue("Run Normalize Field Confidence", BoolValue(false));  
    area->dataMgr.saveFieldPoints(s_iso);    

    runStep3NBVcandidates();
    NBVCandidatesScan();
    //save nbv skel and view
    QString s_nbv;
    s_nbv.sprintf("\\%d_nbv.skel", ic);
    s_nbv = file_location + s_nbv;
    area->dataMgr.saveSkeletonAsSkel(s_nbv);
    s_nbv.replace(".skel", ".View");
    area->saveView(s_nbv);

    mergeScannedMeshWithOriginal();
    //save merged scan
    cout<<"begin to save merged mesh" <<endl;
    QString s_merged_mesh;
    s_merged_mesh.sprintf("\\%d_merged_mesh", ic);
    s_merged_mesh = file_location + s_merged_mesh;
    area->dataMgr.saveMergedMesh(s_merged_mesh);
    cout<< "end save merged mesh" <<endl;

    /* cout << "Begin remove outliers!" <<endl;
    GlobalFun::removeOutliers(original, global_paraMgr.data.getDouble("CGrid Radius") * 2, 10);
    cout << "End remove outliers!" <<endl;*/
  }

  QString last_original = "\\ultimate_original.ply";
  last_original = file_location + last_original;
  area->dataMgr.savePly(last_original, *area->dataMgr.getCurrentOriginal());

  log.close();
  cout << "All is done!" <<endl;
}


void CameraParaDlg::getRotateCenterX(double _val)
{
  area->rotate_pos.X() = _val;
  update(); area->update(); //area->updateGL();
}

void CameraParaDlg::getRotateCenterY(double _val)
{
  area->rotate_pos.Y() = _val;
  update(); area->update(); //area->updateGL();
}

void CameraParaDlg::getRotateCenterZ(double _val)
{
  area->rotate_pos.Z() = _val;
  update(); area->update(); //area->updateGL();
}

void CameraParaDlg::getRotateNormalX(double _val)
{
  area->rotate_normal.X() = _val;
  update(); area->update(); //area->updateGL();
}

void CameraParaDlg::getRotateNormalY(double _val)
{
  area->rotate_normal.Y() = _val;
  update(); area->update(); //area->updateGL();
}

void CameraParaDlg::getRotateNormalZ(double _val)
{
  area->rotate_normal.Z() = _val;
  update(); area->update(); //area->updateGL();
}

void CameraParaDlg::getRotateStep(double _val)
{
  area->rotate_delta = _val;
  update(); area->update(); //area->updateGL();
}

void CameraParaDlg::getRotateAngle(double _val)
{
  area->rotate_angle = _val;
  update(); area->update(); //area->updateGL();
}


void CameraParaDlg::rotateStep()
{
  area->rotate_angle += area->rotate_delta;
  if (area->rotate_angle >= 360)
  {
    area->rotate_angle -= 360;
  }
  initWidgets();
  if (m_paras->glarea.getBool("SnapShot Each Iteration"))
  {
    area->figureSnapShot();
  }
  update(); area->update(); area->updateGL();
}

void CameraParaDlg::rotateAnimation()
{
  if (m_paras->glarea.getBool("SnapShot Each Iteration"))
  {
    area->figureSnapShot();
  }

  int rotate_time  = 360 / area->rotate_delta;
  for (int i = 0; i < rotate_time; i++)
  {
    rotateStep();
  }
  update(); area->update(); area->updateGL();


}

void CameraParaDlg::sliceAnimation()
{
  double step_percentage_size =  area->rotate_delta / 360.;
  int step_number = 360. / area->rotate_delta;

  //if (m_paras->glarea.getBool("SnapShot Each Iteration"))
  //{
  //  area->figureSnapShot();
  //}

  if (global_paraMgr.poisson.getBool("Show X Slices"))
  {
    double recored_pos = global_paraMgr.poisson.getDouble("Current X Slice Position");
    for (int i = 0; i < step_number+1; i++)
    {
      double position = step_percentage_size * i;
      position = (std::min)(1.0, position);
      global_paraMgr.poisson.setValue("Current X Slice Position", DoubleValue(position));

      global_paraMgr.poisson.setValue("Run Slice", BoolValue(true));
      area->runPoisson();
      global_paraMgr.poisson.setValue("Run Slice", BoolValue(false));

      initWidgets();
      if (m_paras->glarea.getBool("SnapShot Each Iteration"))
      {
        area->figureSnapShot();
      }
      update(); area->update(); area->updateGL();
    }

  }
  else if (global_paraMgr.poisson.getBool("Show Y Slices"))
  {
    double recored_pos = global_paraMgr.poisson.getDouble("Current Y Slice Position");
    for (int i = 0; i < step_number+1; i++)
    {
      double position = step_percentage_size * i;
      position = (std::min)(1.0, position);
      global_paraMgr.poisson.setValue("Current Y Slice Position", DoubleValue(position));

      global_paraMgr.poisson.setValue("Run Slice", BoolValue(true));
      area->runPoisson();
      global_paraMgr.poisson.setValue("Run Slice", BoolValue(false));

      initWidgets();
      if (m_paras->glarea.getBool("SnapShot Each Iteration"))
      {
        area->figureSnapShot();
      }
      update(); area->update(); area->updateGL();
    }
  }
  else if (global_paraMgr.poisson.getBool("Show Z Slices"))
  {
    double recored_pos = global_paraMgr.poisson.getDouble("Current Z Slice Position");
    for (int i = 0; i < step_number+1; i++)
    {
      double position = step_percentage_size * i;
      position = (std::min)(1.0, position);
      global_paraMgr.poisson.setValue("Current Z Slice Position", DoubleValue(position));

      global_paraMgr.poisson.setValue("Run Slice", BoolValue(true));
      area->runPoisson();
      global_paraMgr.poisson.setValue("Run Slice", BoolValue(false));

      initWidgets();
      if (m_paras->glarea.getBool("SnapShot Each Iteration"))
      {
        area->figureSnapShot();
      }
      update(); area->update(); area->updateGL();
    }
  }
}

void CameraParaDlg::needSnapShotEachIteration(bool _val)
{
  global_paraMgr.glarea.setValue("SnapShot Each Iteration",BoolValue(_val));
}


void CameraParaDlg::getSnapShotIndex(double _val)
{
  m_paras->glarea.setValue("Snapshot Index", DoubleValue(_val));
  area->updateGL();
}


void CameraParaDlg::runRemoveSampleOutliers()
{
	//area->removeOutliers();
	
	double outlier_percentage = global_paraMgr.wLop.getDouble("Outlier Percentage");
	GlobalFun::removeOutliers(area->dataMgr.getCurrentSamples(), global_paraMgr.data.getDouble("CGrid Radius"), outlier_percentage);
	cout<<"has removed samples outliers"<<endl;

	area->initView();
	area->updateGL();
}

void CameraParaDlg::runRemoveSamplesWithLowConfidence()
{
  if (area->dataMgr.isIsoPointsEmpty() || area->dataMgr.isSamplesEmpty())
  {
    cout << "it required Sample and ISO points!" << endl;
    return;
  }

  vector<CMesh* > *scanned_results = area->dataMgr.getScannedResults();
  double merge_confidence_threshold = global_paraMgr.camera.getDouble("Merge Confidence Threshold");
  int merge_pow = static_cast<int>(global_paraMgr.nbv.getDouble("Merge Probability Pow"));

  double radius_threshold = global_paraMgr.wLop.getDouble("CGrid Radius");
  double radius2 = radius_threshold * radius_threshold;
  double iradius16 = -4/radius2;

  double sigma = global_paraMgr.norSmooth.getDouble("Sharpe Feature Bandwidth Sigma");
  double sigma_threshold = pow(max(1e-8,1-cos(sigma/180.0*3.1415926)), 2);
  CMesh* iso_points = area->dataMgr.getCurrentIsoPoints();

  CMesh* samples = area->dataMgr.getCurrentSamples();
  //CMesh* iso_points = area->dataMgr.getCurrentIsoPoints();

  Timer time;
  time.start("Sample ISOpoints Neighbor Tree!!");
  GlobalFun::computeBallNeighbors(samples, iso_points, 
                                  radius_threshold, samples->bbox);
  time.end();

  double max_confidence = 0.0f;
  double min_confidence = BIG;

  for (int i = 0; i < samples->vert.size(); i++)
  {
    CVertex& v = samples->vert[i];

    double sum_confidence = 0.0;
    double sum_w = 0.0;

    for (int j = 0; j < v.original_neighbors.size(); j++)
    {
      int iso_index = v.original_neighbors[j];
      CVertex& t = iso_points->vert[iso_index];

      double dist2 = GlobalFun::computeEulerDistSquare(v.P(), t.P());
      double dist_diff = exp(dist2 * iradius16);

      sum_confidence += dist_diff * t.eigen_confidence;
      sum_w += 1;
    }

    if (v.original_neighbors.size() > 0 )
      sum_confidence /= sum_w;

    v.eigen_confidence = sum_confidence;

    max_confidence = sum_confidence > max_confidence ? sum_confidence : max_confidence;
    min_confidence = sum_confidence < min_confidence ? sum_confidence : min_confidence;
  }
  


  for (int i = 0; i < samples->vert.size(); i++)
  {
    CVertex& v = samples->vert[i];

    v.eigen_confidence = (v.eigen_confidence - min_confidence) / (max_confidence - min_confidence);

    if (v.eigen_confidence > merge_confidence_threshold 
      || (1.0f * rand() / (RAND_MAX+1.0) > pow((1 - v.eigen_confidence), merge_pow)))
    {
      v.is_ignore = true;
      continue;
    }
  }

  GlobalFun::deleteIgnore(samples);

  area->updateUI();
}

void CameraParaDlg::runAddSamplesToOiriginal()
{
	CMesh* samples = area->dataMgr.getCurrentSamples();
	CMesh* original = area->dataMgr.getCurrentOriginal();
	
	for (int i = 0; i < samples->vert.size(); i++)
	{
		CVertex t = samples->vert[i];
		t.is_original = true;
		t.m_index = original->vert.size() + i;

		original->vert.push_back(t);
	}
}

void CameraParaDlg::runICP()
{
	GlobalFun::computeICP(area->dataMgr.getCurrentOriginal(), area->dataMgr.getCurrentSamples());
}

void CameraParaDlg::moveTranslation()
{
  ifstream infile("artect_movement.txt");
  CMesh* samples = area->dataMgr.getCurrentSamples();
  Point3f movement;
  infile >> movement.X() >> movement.Y() >> movement.Z();
  cout << "movement " << endl;
  GlobalFun::printPoint3(cout, movement);
  for (int i = 0; i < samples->vert.size(); i++)
  {
    CVertex& v = samples->vert[i];

    v.P() -= movement;
  }

  return;
}