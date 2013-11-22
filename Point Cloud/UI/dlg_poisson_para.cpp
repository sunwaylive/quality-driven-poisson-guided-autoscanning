#include "UI/dlg_poisson_para.h"

PoissonParaDlg::PoissonParaDlg(QWidget *p, ParameterMgr * _paras, GLArea * _area) : QFrame(p)
{
	ui = new Ui::poisson_paras;
	PoissonParaDlg::ui->setupUi(this);
	area = _area;
	m_paras = _paras;

	if(!initWidgets())
	{
		cerr << " PoissonParaDlg::initWidgets failed." << endl;
		return;
	}
	initConnects();
}

// 
void PoissonParaDlg::initConnects()
{
  if (!connect(area,SIGNAL(needUpdateStatus()),this,SLOT(initWidgets())))
  {
    cout << "can not connect signal" << endl;
  }

  if(!connect(ui->radius,SIGNAL(valueChanged(double)),this,SLOT(getRadiusValues(double))))
  {
    cerr << "cannot connect WlopParaDlg::getDoubleValues(double)." << endl;
  }

	connect(ui->radius,SIGNAL(valueChanged(double)),this,SLOT(getRadiusValues(double)));
  connect(ui->poisson_depth,SIGNAL(valueChanged(double)),this,SLOT(getPoissonDepth(double)));
  connect(ui->poisson_ISO_interval,SIGNAL(valueChanged(double)),this,SLOT(getPoissonIsoInterval(double)));
  connect(ui->poisson_sample_number,SIGNAL(valueChanged(double)),this,SLOT(getPoissonSampleNumber(double)));

	connect(ui->pushButton_poisson_leafs,SIGNAL(clicked()),this,SLOT(runPoissonAndExtractLeafs()));
  connect(ui->pushButton_poisson_nodes,SIGNAL(clicked()),this,SLOT(runPoissonAndExtractNodes()));  
  connect(ui->pushButton_poisson_MC,SIGNAL(clicked()),this,SLOT(runPoissonAndExtractMC()));
  connect(ui->pushButton_remove_non_iso_points,SIGNAL(clicked()),this,SLOT(removeNonIsoPoints()));
  connect(ui->pushButton_label_iso_points,SIGNAL(clicked()),this,SLOT(labelIsoPoints()));
  connect(ui->pushButton_label_smooth,SIGNAL(clicked()),this,SLOT(labelSmooth()));
  connect(ui->pushButton_label_boundary_points,SIGNAL(clicked()), this, SLOT(labelBoundaryPoints()));
  connect(ui->pushButton_compute_view_candidates, SIGNAL(clicked()), this, SLOT(computeViewCandidates()));
  connect(ui->pushButton_clear_label,SIGNAL(clicked()),this,SLOT(clearLabel()));
  connect(ui->pushButton_slice,SIGNAL(clicked()),this,SLOT(runSlice()));
  connect(ui->pushButton_view_candidates_clustering,SIGNAL(clicked()),this,SLOT(viewCandidatesClustering()));

  connect(ui->checkBox_show_slices,SIGNAL(clicked(bool)),this,SLOT(showSlices(bool)));
  connect(ui->checkBox_show_slices_transparent,SIGNAL(clicked(bool)),this,SLOT(showSlicesTransparent(bool)));
  connect(ui->checkBox_show_slices_X,SIGNAL(clicked(bool)),this,SLOT(showSlicesX(bool)));
  connect(ui->checkBox_show_slices_Y,SIGNAL(clicked(bool)),this,SLOT(showSlicesY(bool)));
  connect(ui->checkBox_show_slices_Z,SIGNAL(clicked(bool)),this,SLOT(showSlicesZ(bool)));

  connect(ui->checkBox_use_confidence1,SIGNAL(clicked(bool)),this,SLOT(useConfidence1(bool)));
  connect(ui->checkBox_use_confidence2,SIGNAL(clicked(bool)),this,SLOT(useConfidence2(bool)));
  connect(ui->checkBox_use_confidence3,SIGNAL(clicked(bool)),this,SLOT(useConfidence3(bool)));
  connect(ui->checkBox_use_confidence4,SIGNAL(clicked(bool)),this,SLOT(useConfidence4(bool)));

  connect(ui->pushButton_compute_confidence_original,SIGNAL(clicked()),this,SLOT(computeOriginalConfidence()));
  connect(ui->pushButton_compute_confidence_samples,SIGNAL(clicked()),this,SLOT(computeSamplesConfidence()));
  connect(ui->pushButton_compute_confidence_iso,SIGNAL(clicked()),this,SLOT(computeIsoConfidence()));


}

// 
bool PoissonParaDlg::initWidgets()
{
	ui->radius->setValue(m_paras->poisson.getDouble("CGrid Radius"));
  ui->poisson_depth->setValue(m_paras->poisson.getDouble("Max Depth"));
  ui->poisson_ISO_interval->setValue(m_paras->glarea.getDouble("ISO Interval Size"));
  ui->poisson_sample_number->setValue(m_paras->poisson.getDouble("Poisson Disk Sample Number"));

	Qt::CheckState state = m_paras->poisson.getBool("Show Slices Mode") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
	ui->checkBox_show_slices->setCheckState(state);

  state = m_paras->poisson.getBool("Show Transparent Slices") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->checkBox_show_slices_transparent->setCheckState(state);
  state = m_paras->poisson.getBool("Show X Slices") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->checkBox_show_slices_X->setCheckState(state);
  state = m_paras->poisson.getBool("Show Y Slices") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->checkBox_show_slices_Y->setCheckState(state);
  state = m_paras->poisson.getBool("Show Z Slices") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->checkBox_show_slices_Z->setCheckState(state);

  state = m_paras->poisson.getBool("Use Confidence 1") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->checkBox_use_confidence1->setCheckState(state);
  state = m_paras->poisson.getBool("Use Confidence 2") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->checkBox_use_confidence2->setCheckState(state);
  state = m_paras->poisson.getBool("Use Confidence 3") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->checkBox_use_confidence3->setCheckState(state);
  state = m_paras->poisson.getBool("Use Confidence 4") ? (Qt::CheckState::Checked) : (Qt::CheckState::Unchecked);
  ui->checkBox_use_confidence4->setCheckState(state);

  update();
  repaint();
	return true;
}

void PoissonParaDlg::getRadiusValues(double _val)
{
	m_paras->setGlobalParameter("CGrid Radius",DoubleValue(_val));
}

void PoissonParaDlg::getPoissonDepth(double _val)
{
  global_paraMgr.poisson.setValue("Max Depth", DoubleValue(_val));
}

void PoissonParaDlg::getPoissonIsoInterval(double _val)
{
  global_paraMgr.glarea.setValue("ISO Interval Size", DoubleValue(_val));
}

void PoissonParaDlg::getPoissonSampleNumber(double _val)
{
  global_paraMgr.poisson.setValue("Poisson Disk Sample Number", DoubleValue(_val));
}

void PoissonParaDlg::runPoissonAndExtractLeafs()
{
	area->runPoisson();
	area->updateGL();
}

//void PoissonParaDlg::runPoissonAndExtractNodes()
//{
//  global_paraMgr.poisson.setValue("Run Extract All Octree Nodes", BoolValue(true));
//  area->runPoisson();
//  global_paraMgr.poisson.setValue("Run Extract All Octree Nodes", BoolValue(false));
//}

void PoissonParaDlg::runPoissonAndExtractNodes()
{
  global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(false));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(true));
}

void PoissonParaDlg::runPoissonAndExtractMC()
{

  global_paraMgr.poisson.setValue("Run Extract MC Points", BoolValue(true));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run Extract MC Points", BoolValue(false));


}

void PoissonParaDlg::runSlice()
{
  global_paraMgr.poisson.setValue("Run Slice", BoolValue(true));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run Slice", BoolValue(false));
}

void PoissonParaDlg::removeNonIsoPoints()
{
  global_paraMgr.poisson.setValue("Run Poisson On Samples", BoolValue(true));
  global_paraMgr.poisson.setValue("Run Extract MC Points", BoolValue(true));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run Extract MC Points", BoolValue(false));
  global_paraMgr.poisson.setValue("Run Poisson On Samples", BoolValue(false));
  //if (area->dataMgr.isIsoPointsEmpty())
  //{
  //  return;
  //}

  //double iso_threshold = global_paraMgr.glarea.getDouble("ISO Interval Size");
  //CMesh* iso_points = area->dataMgr.getCurrentIsoPoints();

  //vector<CVertex> temp;
  //for (int i = 0; i < iso_points->vert.size(); i++)
  //{
  //  temp.push_back(iso_points->vert[i]);
  //}

  //iso_points->vert.clear();
  //for (int i = 0; i < temp.size(); i++)
  //{
  //  CVertex& v = temp[i];
  //  if (abs(v.eigen_confidence) < iso_threshold)
  //  {
  //    iso_points->vert.push_back(v);
  //    iso_points->bbox.Add(v.P());
  //  }
  //}
  //iso_points->vn = iso_points->vert.size();
}

void PoissonParaDlg::clearLabel()
{
  if (area->dataMgr.isIsoPointsEmpty())
  {
    return;
  }

  double iso_threshold = global_paraMgr.glarea.getDouble("ISO Interval Size");
  CMesh* iso_points = area->dataMgr.getCurrentIsoPoints();

  for (int i = 0; i < iso_points->vert.size(); i++)
  {
    iso_points->vert[i].is_hole = false;
  }
}

void PoissonParaDlg::labelIsoPoints()
{
  global_paraMgr.poisson.setValue("Run Label ISO Points", BoolValue(true));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run Label ISO Points", BoolValue(false));
}

void PoissonParaDlg::labelSmooth()
{
  global_paraMgr.poisson.setValue("Run Label Smooth", BoolValue(true));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run Label Smooth", BoolValue(false));
}

void PoissonParaDlg::labelBoundaryPoints()
{
  global_paraMgr.poisson.setValue("Run Label Boundary Points", BoolValue(true));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run Label Boundary Points", BoolValue(false));
}

void PoissonParaDlg::computeViewCandidates()
{
  global_paraMgr.poisson.setValue("Run Compute View Candidates", BoolValue(true));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run Compute View Candidates", BoolValue(false));
}

void PoissonParaDlg::viewCandidatesClustering()
{
  global_paraMgr.poisson.setValue("Run View Candidates Clustering", BoolValue(true));
  area->runPoisson();
  global_paraMgr.poisson.setValue("Run View Candidates Clustering", BoolValue(false));
}


void PoissonParaDlg::showSlices(bool _val)
{
  global_paraMgr.poisson.setValue("Show Slices Mode", BoolValue(_val));
  area->updateGL();
}

void PoissonParaDlg::showSlicesTransparent(bool _val)
{
  global_paraMgr.poisson.setValue("Show Transparent Slices", BoolValue(_val));
  area->updateGL();
}

void PoissonParaDlg::showSlicesX(bool _val)
{
  global_paraMgr.poisson.setValue("Show X Slices", BoolValue(_val));
  area->updateGL();
}

void PoissonParaDlg::showSlicesY(bool _val)
{
  global_paraMgr.poisson.setValue("Show Y Slices", BoolValue(_val));
  area->updateGL();
}

void PoissonParaDlg::showSlicesZ(bool _val)
{
  global_paraMgr.poisson.setValue("Show Z Slices", BoolValue(_val));
}

void PoissonParaDlg::useConfidence1(bool _val)
{
  global_paraMgr.poisson.setValue("Use Confidence 1", BoolValue(_val));  
}

void PoissonParaDlg::useConfidence2(bool _val)
{
  global_paraMgr.poisson.setValue("Use Confidence 2", BoolValue(_val));  
}

void PoissonParaDlg::useConfidence3(bool _val)
{
  global_paraMgr.poisson.setValue("Use Confidence 3", BoolValue(_val));  
}

void PoissonParaDlg::useConfidence4(bool _val)
{
  global_paraMgr.poisson.setValue("Use Confidence 4", BoolValue(_val));  
}

void PoissonParaDlg::computeOriginalConfidence()
{
  global_paraMgr.poisson.setValue("Compute Original Confidence", BoolValue(true)); 
  area->runPoisson();
  global_paraMgr.poisson.setValue("Compute Original Confidence", BoolValue(false));  
}

void PoissonParaDlg::computeSamplesConfidence()
{
  global_paraMgr.poisson.setValue("Compute Sample Confidence", BoolValue(true));  
  area->runPoisson();
  global_paraMgr.poisson.setValue("Compute Sample Confidence", BoolValue(false));  
}

void PoissonParaDlg::computeIsoConfidence()
{
  global_paraMgr.poisson.setValue("Compute ISO Confidence", BoolValue(true));  
  area->runPoisson();
  global_paraMgr.poisson.setValue("Compute ISO Confidence", BoolValue(false));  
}



PoissonParaDlg::~PoissonParaDlg()
{
	delete ui;
	ui = NULL;

	area = NULL;
	m_paras = NULL;

}

void PoissonParaDlg::setFrameConent()
{
  if(layout()) delete layout();
  QGridLayout * vLayout = new QGridLayout(this);
  vLayout->setAlignment(Qt::AlignTop);
  setLayout(vLayout);

  showNormal();
  adjustSize();
}

