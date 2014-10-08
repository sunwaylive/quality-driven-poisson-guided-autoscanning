#include "OneKeyNBVBack.h"
#include "UI/dlg_camera_para.h"

OneKeyNBVBack::OneKeyNBVBack( QString file_location, GLArea* area)
{

}

OneKeyNBVBack::~OneKeyNBVBack()
{

}

void OneKeyNBVBack::oneKeyNBV(QString file_location, GLArea *area)
{
  printf("sub thread: %d\n", QThread::currentThreadId());
  QString s_log = "\\log.txt";
  s_log = file_location + s_log;
  ofstream log;
  log.open(s_log.toAscii().data());
  //cout.rdbuf(log.rdbuf());

  QString para = "\\parameter.para";
  para = file_location + para;
  area->dataMgr.saveParameters(para);

  int iteration_cout = global_paraMgr.nbv.getInt("NBV Iteration Count");
  const int holeFrequence = 4;
  CMesh *original = area->dataMgr.getCurrentOriginal();
  for (int ic = 0; ic < iteration_cout; ++ic)
  {
    //save original
    QString s_original;
    s_original.sprintf("\\%d_original.ply", ic);
    s_original =file_location + s_original;
    area->dataMgr.savePly(s_original, *area->dataMgr.getCurrentOriginal());

    //compute normal on original
    vector<Point3f> before_normal;
    for (int i = 0; i < original->vert.size(); ++i)
      before_normal.push_back(original->vert[i].N()); 

    int knn = global_paraMgr.norSmooth.getInt("PCA KNN");
    vcg::NormalExtrapolation<vector<CVertex> >::ExtrapolateNormals(original->vert.begin(), original->vert.end(), knn, -1);
    //vcg::tri::PointCloudNormal<CMesh>::Param pca_para;
    //pca_para.fittingAdjNum = knn;
    //fixme: a debug error in compute
    //vcg::tri::PointCloudNormal<CMesh>::Compute(*original, pca_para, NULL);

    for (int i = 0; i < original->vert.size(); ++i)
    {
      if (before_normal[i] * original->vert[i].N() < 0.0f)
        original->vert[i].N() *= -1;
    }
    Sleep(5000);
    //save normalized original
    QString s_normal_original;
    s_normal_original.sprintf("\\%d_normal_original.ply", ic);
    s_normal_original = file_location + s_normal_original;
    area->dataMgr.savePly(s_normal_original, *area->dataMgr.getCurrentOriginal());

    //compute radius
    area->dataMgr.downSamplesByNum();
    area->initSetting();
    Sleep(5000);

    if(ic % holeFrequence == 0)
    {
      cout<<"begin to run hole poisson confidence" <<endl;
      //runStep2HolePoissonConfidence();
      global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(true));
      global_paraMgr.poisson.setValue("Run Extract MC Points", BoolValue(true));
      global_paraMgr.poisson.setValue("Compute Hole Confidence", BoolValue(true));
      area->runPoisson();
      global_paraMgr.poisson.setValue("Run Extract MC Points", BoolValue(false));
      global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(false));
      global_paraMgr.poisson.setValue("Compute Hole Confidence", BoolValue(false));

      cout<<"end to run hole poisson confidence" <<endl;
    }else{
      cout<<"begin to run combined poisson confidence" <<endl;
      //runStep2CombinedPoissonConfidence();
      global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(true));
      global_paraMgr.poisson.setValue("Run Extract MC Points", BoolValue(true));
      global_paraMgr.poisson.setValue("Run One Key PoissonConfidence", BoolValue(true));
      area->runPoisson();
      global_paraMgr.poisson.setValue("Run Extract MC Points", BoolValue(false));
      global_paraMgr.poisson.setValue("Run Poisson On Original", BoolValue(false));
      global_paraMgr.poisson.setValue("Run One Key PoissonConfidence", BoolValue(false));
      cout<<"end run combined poisson confidence" <<endl;
    }
    Sleep(5000);

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
    s_iso.replace(".skel", ".View");
    area->saveView(s_iso);
    //save iso dat and raw
    s_iso.replace(".View", ".raw");
    global_paraMgr.poisson.setValue("Run Normalize Field Confidence", BoolValue(true));  
    area->runPoisson();
    global_paraMgr.poisson.setValue("Run Normalize Field Confidence", BoolValue(false));  
    area->dataMgr.saveFieldPoints(s_iso);    
    Sleep(5000);

    global_paraMgr.nbv.setValue("Run One Key NBV", BoolValue(true));
    area->runNBV();
    global_paraMgr.nbv.setValue("Run One Key NBV", BoolValue(false));
    emit updateTableViewNBVCandidate();
    Sleep(5000);

    global_paraMgr.camera.setValue("Run NBV Scan", BoolValue(true));
    area->runCamera();
    global_paraMgr.camera.setValue("Run NBV Scan", BoolValue(false));
    emit updateTableViewNBVCandidate();
    emit updateTabelViewScanResults();
    Sleep(5000);

    //save nbv skel and view
    QString s_nbv;
    s_nbv.sprintf("\\%d_nbv.skel", ic);
    s_nbv = file_location + s_nbv;
    s_nbv.replace(".skel", ".View");
    area->saveView(s_nbv);
    emit mergeScannedMeshWithOriginal();
    Sleep(5000);
    //save merged scan
    //cout<<"begin to save merged mesh" <<endl;
    //QString s_merged_mesh;
    //s_merged_mesh.sprintf("\\%d_merged_mesh", ic);
    //s_merged_mesh = m_file_location + s_merged_mesh;
    //area->dataMgr.saveMergedMesh(s_merged_mesh);
    //cout<< "end save merged mesh" <<endl;
  }

  QString last_original = "\\ultimate_original.ply";
  last_original = file_location + last_original;
  area->dataMgr.savePly(last_original, *area->dataMgr.getCurrentOriginal());

  cout << "All is done!" <<endl;
  log.close();
}