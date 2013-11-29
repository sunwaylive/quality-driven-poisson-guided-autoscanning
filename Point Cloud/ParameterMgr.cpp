#include "ParameterMgr.h"
#include <iostream>

int ParameterMgr::init_time = 0;
ParameterMgr global_paraMgr;

ParameterMgr::ParameterMgr(void)
{
	init_time++;
	if(init_time > 1)
	{
		std::cout << "can not init ParameterMgr twice!" << endl;
		return;
	}

	grid_r = 0.2;


	initDataMgrParameter();
	initDrawerParameter();		
	initGlareaParameter();
	initWLopParameter();
	initNormalSmootherParameter();
	initSkeletonParameter();
	initUpsamplingParameter();
	initPoissonParameter();
	initCameraParameter();
	initNBVParameter();
}

ParameterMgr::~ParameterMgr(void)
{

}

void ParameterMgr::setGlobalParameter(QString paraName,Value& val)
{
	if(glarea.hasParameter(paraName))
		glarea.setValue(paraName, val);
	if(data.hasParameter(paraName))
		data.setValue(paraName, val);
	if(drawer.hasParameter(paraName))
		drawer.setValue(paraName, val);
	if(wLop.hasParameter(paraName))
		wLop.setValue(paraName, val);
	if(norSmooth.hasParameter(paraName))
		norSmooth.setValue(paraName, val);
	if (skeleton.hasParameter(paraName))
		skeleton.setValue(paraName, val);
	if (upsampling.hasParameter(paraName))
		upsampling.setValue(paraName, val);
	if (poisson.hasParameter(paraName))
		poisson.setValue(paraName, val);
}

void ParameterMgr::initDataMgrParameter()
{
	data.addParam(new RichDouble("Init Radius Para", 2.0));
	data.addParam(new RichDouble("Down Sample Num", 2500));
	data.addParam(new RichDouble("CGrid Radius", grid_r));
}


void ParameterMgr::initGlareaParameter()
{
	glarea.addParam(new RichString("Running Algorithm Name", "") );

	glarea.addParam(new RichBool("Light On or Off", false) );

	glarea.addParam(new RichBool("Show Normal", false) );

	glarea.addParam(new RichBool("Show Samples", true) );
	glarea.addParam(new RichBool("Show Samples Quad", false) );
	glarea.addParam(new RichBool("Show Samples Dot", true) );
	glarea.addParam(new RichBool("Show Samples Circle", false) );
	glarea.addParam(new RichBool("Show Samples Sphere", false) );
	glarea.addParam(new RichBool("Show ISO Points", true) );
	glarea.addParam(new RichBool("Use ISO Interval", false) );
	glarea.addParam(new RichBool("Show NBV Grids", false));
	glarea.addParam(new RichBool("Show NBV Candidates", false));
	glarea.addParam(new RichBool("Show Scan Candidates", true));
	glarea.addParam(new RichBool("Show Scanned Mesh", true));

	glarea.addParam(new RichBool("Show Model", true));
	glarea.addParam(new RichBool("Show Original", false) );
	glarea.addParam(new RichBool("Show Original Quad", false) );
	glarea.addParam(new RichBool("Show Original Dot", true) );
	glarea.addParam(new RichBool("Show Original Circle", false) );
	glarea.addParam(new RichBool("Show Original Sphere", false) );

	glarea.addParam(new RichBool("Show Skeleton", false));

	glarea.addParam(new RichBool("Show Radius", true));
	glarea.addParam(new RichBool("Show All Radius", false));
	glarea.addParam(new RichBool("Show Radius Use Pick", true));
	glarea.addParam(new RichBool("Show Red Radius Line", true));
	glarea.addParam(new RichBool("Multiply Pick Point", true) );

	glarea.addParam(new RichBool("Show Bounding Box", true));


	glarea.addParam(new RichBool("GLarea Busying", false) );
	glarea.addParam(new RichBool("Algorithom Stop", false) );


	glarea.addParam(new RichPoint3f("Light Position", vcg::Point3f(-4.0, -4.0, -4.0)));
	glarea.addParam(new RichColor("Light Ambient Color", QColor(85, 85, 85)));
	glarea.addParam(new RichColor("Light Diffuse Color", QColor(164, 241, 101)));
	glarea.addParam(new RichColor("Light Specular Color", QColor(255, 255, 255)));

	//glarea.addParam(new RichPoint3f("Light Position", vcg::Point3f(4.0, 4.0, 4.0)));
	//glarea.addParam(new RichColor("Light Ambient Color", QColor(0.0, 0.0, 0.0)));
	//glarea.addParam(new RichColor("Light Diffuse Color", QColor(204, 204, 204)));
	//glarea.addParam(new RichColor("Light Specular Color", QColor(255, 255, 255)));

	glarea.addParam(new RichDouble("Snapshot Resolution", 2));
	glarea.addParam(new RichDouble("Snapshot Index", 1));
	glarea.addParam(new RichDouble("Radius Ball Transparency", 0.3));
	glarea.addParam(new RichDouble("Slice Color Scale", 1));
	glarea.addParam(new RichDouble("ISO Interval Size", 50));
	glarea.addParam(new RichDouble("Confidence Color Scale", 0.5));
	glarea.addParam(new RichDouble("ISO Value Shift", 0.00));

	glarea.addParam(new RichBool("SnapShot Each Iteration", false));
	glarea.addParam(new RichBool("No Snap Radius", false));
	glarea.addParam(new RichBool("All Octree Nodes", false));

}


void ParameterMgr::initDrawerParameter()
{
	drawer.addParam(new RichBool("Doing Pick", false));
	drawer.addParam(new RichBool("Need Cull Points", false) );
	drawer.addParam(new RichBool("Use Pick Original", false));
	drawer.addParam(new RichBool("Use Pick Mode2", false) );
	drawer.addParam(new RichBool("Skeleton Light", true));
	drawer.addParam(new RichBool("Show Individual Color", true));
	drawer.addParam(new RichBool("Use Color From Normal", false));
	drawer.addParam(new RichBool("Use Differ Branch Color", false));
	drawer.addParam(new RichBool("Show Confidence Color", true));

	drawer.addParam(new RichDouble("Original Draw Width", 0.0015));
	drawer.addParam(new RichDouble("Sample Draw Width", 0.005));
	drawer.addParam(new RichDouble("Sample Dot Size", 6));
	drawer.addParam(new RichDouble("ISO Dot Size", 4));
	drawer.addParam(new RichDouble("Original Dot Size", 1));
	drawer.addParam(new RichDouble("Normal Line Width", 2));
	drawer.addParam(new RichDouble("Normal Line Length", 0.12));

	drawer.addParam(new RichColor("Background Color", QColor(255, 255, 255) ));
	drawer.addParam(new RichColor("Normal Line Color", QColor(0, 0, 255) ));
	drawer.addParam(new RichColor("Sample Point Color", QColor(255, 0, 0) ));
	drawer.addParam(new RichColor("Original Point Color", QColor(48, 48, 48) ));
	drawer.addParam(new RichColor("Feature Color", QColor(0, 0, 255) ));
	drawer.addParam(new RichColor("Pick Point Color", QColor(128, 128, 0) ));
	drawer.addParam(new RichColor("Pick Point DNN Color", QColor(0, 0, 155) ));

	drawer.addParam(new RichColor("Skeleton Bone Color", QColor(200, 0, 0) ));
	drawer.addParam(new RichColor("Skeleton Node Color", QColor(50, 250, 50) ));
	drawer.addParam(new RichColor("Skeleton Branch Color", QColor(0, 0, 0)));
	drawer.addParam(new RichDouble("Skeleton Bone Width", 100)); // ./10000
	drawer.addParam(new RichDouble("Skeleton Node Size", 180)); // ./10000
	drawer.addParam(new RichDouble("Skeleton Branch Size", 30)); // abandoned
}


void ParameterMgr::initWLopParameter()
{
	wLop.addParam(new RichString("Algorithm Name", "WLOP") );
	wLop.addParam(new RichBool("Run One Key WLOP", false));

	wLop.addParam(new RichDouble("Num Of Iterate Time", 5));

	wLop.addParam(new RichDouble("CGrid Radius", grid_r));
	wLop.addParam(new RichDouble("H Gaussian Para", 4));
	wLop.addParam(new RichDouble("Repulsion Power", 1.0));
	wLop.addParam(new RichDouble("Average Power", 1.0));
	wLop.addParam(new RichBool("Need Compute Density", true));
	wLop.addParam(new RichBool("Need Compute PCA", false));
	wLop.addParam(new RichDouble("Repulsion Mu", 0.5));
	wLop.addParam(new RichDouble("Repulsion Mu2", 0.0));
	wLop.addParam(new RichBool("Run Anisotropic LOP", false));
	wLop.addParam(new RichDouble("Current Movement Error", 0.0));
}

void ParameterMgr::initSkeletonParameter()
{
	/// 
	skeleton.addParam(new RichDouble("Repulsion Power", 1.0));
	skeleton.addParam(new RichDouble("Average Power", 2.0));


	/// 
	skeleton.addParam(new RichDouble("Num Of Iterate Time", 1));
	skeleton.addParam(new RichString("Algorithm Name", "Skeletonization") );

	skeleton.addParam(new RichDouble("CGrid Radius", grid_r));
	skeleton.addParam(new RichDouble("H Gaussian Para", 4));
	skeleton.addParam(new RichBool("Need Compute Density", true));

	skeleton.addParam(new RichDouble("Current Movement Error", 0.0));
	skeleton.addParam(new RichBool("Run Auto Wlop One Step", false));
	skeleton.addParam(new RichBool("Run Auto Wlop One Stage", false));
	skeleton.addParam(new RichBool("The Skeletonlization Process Should Stop", false));

	skeleton.addParam(new RichBool("Step1 Detect Skeleton Feature", false));
	skeleton.addParam(new RichBool("Step2 Run Search New Branchs", false));
	skeleton.addParam(new RichBool("Step3 Clean And Update Radius", false));
	skeleton.addParam(new RichBool("Run Skeletonlization", false));

	//init
	skeleton.addParam(new RichDouble("Max Iterate Time", 55));
	skeleton.addParam(new RichDouble("Stop And Grow Error", 0.0005));
	skeleton.addParam(new RichDouble("Initial Radius", -1.));
	skeleton.addParam(new RichDouble("Radius Update Speed", 0.5));

	//step0
	skeleton.addParam(new RichDouble("Repulsion Mu", 0.35));
	skeleton.addParam(new RichDouble("Repulsion Mu2", 0.15));
	skeleton.addParam(new RichDouble("Follow Sample Radius", 0.33));
	skeleton.addParam(new RichDouble("Follow Sample Max Angle", 80));// should add to UI
	skeleton.addParam(new RichDouble("Inactive And Keep Virtual Angle", 60)); // should add to UI
	skeleton.addParam(new RichDouble("Save Virtual Angle", 30)); // should add to UI

	skeleton.addParam(new RichDouble("Grow Accept Sigma", 0.8));// should add to UI
	skeleton.addParam(new RichDouble("Bad Virtual Angle", 101));// 2013-7-12

	//step1
	skeleton.addParam(new RichDouble("Combine Too Close Threshold", 0.01));
	skeleton.addParam(new RichDouble("Sigma KNN", 6));//this one is hard to determine, should be small for narrow region, but will lead to unnecessary small branches
	skeleton.addParam(new RichDouble("Eigen Feature Identification Threshold", 0.901));

	//step2
	skeleton.addParam(new RichDouble("Branches Search Angle", 25));
	skeleton.addParam(new RichDouble("Virtual Head Accecpt Angle", 25));
	skeleton.addParam(new RichDouble("Snake Search Max Dist Blue", 0.4));
	skeleton.addParam(new RichDouble("Accept Branch Size", 6)); // important, and hard to determine
	skeleton.addParam(new RichDouble("Branch Search Max Dist Yellow", 0.1));

	skeleton.addParam(new RichDouble("Branches Merge Max Dist", 0.08));
	skeleton.addParam(new RichDouble("Branch Search KNN", 12));
	skeleton.addParam(new RichDouble("Combine Similar Angle", 140));
	skeleton.addParam(new RichDouble("Grow Search Radius", 0.15));
	skeleton.addParam(new RichDouble("Add Accept Branch Size", 1));


	//step3
	skeleton.addParam(new RichDouble("Clean Near Branches Dist", 0.05));
	skeleton.addParam(new RichDouble("Fix Original Weight", 0.91));
	skeleton.addParam(new RichDouble("Curve Segment Length", 0.051));
	skeleton.addParam(new RichInt("Fix Original Mode", 4)); // 1 for noisy , 4 for clean

	skeleton.addParam(new RichBool("Run ALL Segment", false));
	skeleton.addParam(new RichBool("Need Segment Right Away", true));
	skeleton.addParam(new RichDouble("Max Stop Radius", 1.99));

	//strategy...
	skeleton.addParam(new RichBool("Use Nearby Combine Strategy", true));
	skeleton.addParam(new RichBool("Use Go Through Strategy", false));
	skeleton.addParam(new RichBool("Use Aggresive Growth Strategy", false));
	skeleton.addParam(new RichBool("Use Clean Points When Following Strategy", true));
	skeleton.addParam(new RichBool("Use All Connect Strategy", true));
	skeleton.addParam(new RichBool("Use Plus Perpendicular Dist Strategy", false));
	skeleton.addParam(new RichBool("Use Kill Too Close Strategy", false));
	skeleton.addParam(new RichBool("Use Compute Eigen Ignore Branch Strategy", true));
	skeleton.addParam(new RichBool("Use Virtual Group Merge Strategy", false));
	skeleton.addParam(new RichBool("Use Final Merge Strategy", true));
	skeleton.addParam(new RichBool("Use Search New Twice Strategy", false));
	skeleton.addParam(new RichBool("Inactive Overlap Strategy", false));
	skeleton.addParam(new RichBool("Move Overlap Strategy", false));
	skeleton.addParam(new RichBool("Use Virtual Near Body Stop Strategy", true));
	skeleton.addParam(new RichBool("Need To Keep Big Bug", false));
	skeleton.addParam(new RichDouble("Change Strategy Radius", 0.45));
	skeleton.addParam(new RichBool("Need Recentering", true));
}

void ParameterMgr::initNormalSmootherParameter()
{
	norSmooth.addParam(new RichString("Algorithm Name", "NormalSmooth") );

	norSmooth.addParam(new RichInt("PCA KNN", 20));
	norSmooth.addParam(new RichDouble("CGrid Radius", grid_r));
	norSmooth.addParam(new RichDouble("Sharpe Feature Bandwidth Sigma", 45));
	norSmooth.addParam(new RichBool("Run Anistropic PCA", false));
	norSmooth.addParam(new RichBool("Run Init Samples Using Normal", false));

	norSmooth.addParam(new RichInt("Number Of Iterate", 1));
	norSmooth.addParam(new RichInt("Number of KNN", 400));

	norSmooth.addParam(new RichDouble("PCA Threshold", 0.8));
}


void ParameterMgr::initUpsamplingParameter()
{
	upsampling.addParam(new RichString("Algorithm Name", "Upsampling") );

	upsampling.addParam(new RichDouble("CGrid Radius", 0.08) );
	upsampling.addParam(new RichInt("Number of Add Point", 50000) );
	upsampling.addParam(new RichDouble("Feature Sigma", 30));

	upsampling.addParam(new RichBool("Using Threshold Process", true) );
	upsampling.addParam(new RichDouble("Dist Threshold", 0.02));
	upsampling.addParam(new RichDouble("Edge Parameter", 0.0));
	upsampling.addParam(new RichDouble("Z Parameter", 0.1));

	upsampling.addParam(new RichBool("Auto Recompute Radius For Dist", true) );
	upsampling.addParam(new RichDouble("Min Dist Rate", 2.0));

	upsampling.addParam(new RichDouble("New Point Avg Sigma", 15));
	upsampling.addParam(new RichBool("Use Avg Normal Method", false) );
	upsampling.addParam(new RichBool("Use Max Theta Psi Method", false));
	upsampling.addParam(new RichBool("Use Sigma Threshold Method", true));
	upsampling.addParam(new RichBool("Use No Psi Method", false));

	upsampling.addParam(new RichDouble("Upsample Radius", grid_r * 0.5) );
	upsampling.addParam(new RichBool("Use Proj New Term", false));
	upsampling.addParam(new RichBool("Run Projection", false));

}

void ParameterMgr::initPoissonParameter()
{
	poisson.addParam(new RichString("Algorithm Name", "Poisson") );
	poisson.addParam(new RichDouble("CGrid Radius", 0.08) );
	poisson.addParam(new RichDouble("View Candidates Distance", 0.85));

	poisson.addParam(new RichBool("Run One Key PoissonConfidence", false));

	poisson.addParam(new RichBool("Run Extract All Octree Nodes", false));
	poisson.addParam(new RichBool("Run Extract MC Points", false));

	poisson.addParam(new RichBool("Run Poisson On Original", true));
	poisson.addParam(new RichBool("Run Generate Poisson Field", false));


	poisson.addParam(new RichBool("Run Poisson On Samples", false));  
	poisson.addParam(new RichBool("Run Label ISO Points", false));
	poisson.addParam(new RichBool("Run Label Smooth", false));
	poisson.addParam(new RichBool("Run Label Boundary Points", false));
	poisson.addParam(new RichBool("Run Compute View Candidates", false));
	poisson.addParam(new RichBool("Run View Candidates Clustering", false));

	poisson.addParam(new RichBool("Run Slice", false));
	poisson.addParam(new RichBool("Run Clear Slice", false));
	poisson.addParam(new RichDouble("Max Depth", 7));

	poisson.addParam(new RichBool("Show Slices Mode", false));
	poisson.addParam(new RichBool("Parallel Slices Mode", false));

	poisson.addParam(new RichBool("Show X Slices", false));
	poisson.addParam(new RichBool("Show Y Slices", false));
	poisson.addParam(new RichBool("Show Z Slices", false));
	poisson.addParam(new RichBool("Show Transparent Slices", false));
	poisson.addParam(new RichDouble("Current X Slice Position", 0.5));
	poisson.addParam(new RichDouble("Current Y Slice Position", 0.5));
	poisson.addParam(new RichDouble("Current Z Slice Position", 0.5));
	poisson.addParam(new RichDouble("Show Slice Percentage", 0.75));
	poisson.addParam(new RichDouble("Poisson Disk Sample Number", 3500));


	poisson.addParam(new RichBool("Use Confidence 1", false));
	poisson.addParam(new RichBool("Use Confidence 2", false));
	poisson.addParam(new RichBool("Use Confidence 3", false));
	poisson.addParam(new RichBool("Use Confidence 4", true));
	poisson.addParam(new RichBool("Compute Original Confidence", false));
	poisson.addParam(new RichBool("Compute Sample Confidence", false));
	poisson.addParam(new RichBool("Compute ISO Confidence", false));
	poisson.addParam(new RichBool("Use Sort Confidence Combination", true));
}

void ParameterMgr::initCameraParameter()
{
	camera.addParam(new RichString("Algorithm Name", "Camera") );
	camera.addParam(new RichBool("Run One Key NewScans", false));

	camera.addParam(new RichBool("Run Initial Scan", false));
	camera.addParam(new RichBool("Run NBV Scan", false));
	camera.addParam(new RichBool("Run Virtual Scan", false));
	camera.addParam(new RichDouble("Camera Resolution",1.0f / 100.0f));
	camera.addParam(new RichDouble("Camera Max Dist", 2.1f));
	camera.addParam(new RichDouble("Camera Horizon Dist", 1.0f));
	camera.addParam(new RichDouble("Camera Vertical Dist", 0.6f));
	camera.addParam(new RichDouble("Camera Dist To Model", 1.0f));
	camera.addParam(new RichBool("Is Init Camera Show", false));
}

void
	ParameterMgr::initNBVParameter()
{
  nbv.addParam(new RichString("Algorithm Name", "NBV"));
  nbv.addParam(new RichBool("Run One Key NBV", false));

  nbv.addParam(new RichBool("Run Build Grid", false));
  nbv.addParam(new RichBool("Run Propagate", false));
  nbv.addParam(new RichBool("Run Propagate One Point", false));
  nbv.addParam(new RichBool("Run Grid Segment", false));

  nbv.addParam(new RichBool("Run Viewing Clustering", false));
  nbv.addParam(new RichBool("Run Extract Views Into Bins", false));
  nbv.addParam(new RichBool("Run Viewing Extract", false));
  nbv.addParam(new RichBool("Run Update View Directions", false));
  
  nbv.addParam(new RichDouble("Grid resolution", 40.f));
  nbv.addParam(new RichBool("Test Other Inside Segment", false));

  nbv.addParam(new RichBool("Use Confidence Separation", false));
  nbv.addParam(new RichBool("Use Average Confidence", false));
  nbv.addParam(new RichBool("Use NBV Test1", false));
  nbv.addParam(new RichBool("Use Max Propagation", true));

  nbv.addParam(new RichDouble("Confidence Separation Value", 0.85));
  nbv.addParam(new RichDouble("Max Ray Steps Para", 1.5));

	nbv.addParam(new RichString("Algorithm Name", "NBV"));
	nbv.addParam(new RichBool("Run One Key NBV", false));

	nbv.addParam(new RichBool("Run Build Grid", false));
	nbv.addParam(new RichBool("Run Propagate", false));
	nbv.addParam(new RichBool("Run Propagate One Point", false));
	nbv.addParam(new RichBool("Run Grid Segment", false));

	nbv.addParam(new RichBool("Run Viewing Clustering", false));
	nbv.addParam(new RichBool("Run Extract Views Into Bins", false));
	nbv.addParam(new RichBool("Run Viewing Extract", false));

	nbv.addParam(new RichDouble("Grid resolution", 40.f));
	nbv.addParam(new RichBool("Test Other Inside Segment", false));

	nbv.addParam(new RichBool("Use Confidence Separation", false));
	nbv.addParam(new RichBool("Use Average Confidence", false));
	nbv.addParam(new RichBool("Use NBV Test1", false));
	nbv.addParam(new RichBool("Use Max Propagation", true));

	nbv.addParam(new RichDouble("Confidence Separation Value", 0.85));
	nbv.addParam(new RichDouble("Max Ray Steps Para", 1.5));

}