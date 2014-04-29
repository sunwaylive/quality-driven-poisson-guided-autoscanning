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

  sample = pData->getCurrentSamples();

  scanned_results = pData->getScannedResults();
  optimalDist = (global_paraMgr.camera.getDouble("Camera Far Distance") +global_paraMgr.camera.getDouble("Camera Near Distance")) 
    / 2 / global_paraMgr.camera.getDouble("Predicted Model Size");
  nbv_candidates = pData->getNbvCandidates();
  scan_candidates = pData->getScanCandidates();
}

void PVSBasedNBV::run()
{
  if (para->getBool("Run PVS Detect Boundary"))
  {
    std::cout << "Run PVS Detect Boundary" <<std::endl;
    runPVSDetectBoundary();
  }
}

void PVSBasedNBV::clear()
{

}

void PVSBasedNBV::runPVSDetectBoundary()
{
  //copy point from original to sample
  GlobalFun::clearCMesh(*sample);  

  for (int i = 0; i < original->vert.size(); ++i)
  {
    CVertex v = original->vert[i];
    v.is_original = false;
    v.is_fixed_sample = true;
    sample->vert.push_back(v);
  }
  sample->vn = sample->vert.size();

  typedef vcg::tri::UpdateFlags<CMesh>::EdgeSorter MyEdge;
  typedef std::vector<MyEdge>::iterator MyEdgeIter;
  typedef vcg::tri::UpdateFlags<CMesh>::FaceIterator FaceIter;
  //compute the topology of sample points
  GlobalFun::ballPivotingReconstruction(*sample);
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

  std::vector<MyEdge> v_edge;
  std::vector<MyEdge> v_border_edge;

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
  //This is important: ways to accessing board points
  sort(v_border_edge.begin(), v_border_edge.end());
  for (int i = 0; i < v_border_edge.size(); ++i)
  {
    std::cout<< (v_border_edge[i].v[0]->P()[0])<<" " << (v_border_edge[i].v[0]->P()[1]) <<" " << (v_border_edge[i].v[0]->P()[2]) <<"     ";
    std::cout<< (v_border_edge[i].v[1]->P()[0])<<" " << (v_border_edge[i].v[1]->P()[1]) <<" " << (v_border_edge[i].v[1]->P()[2]) <<std::endl;
  }
  
  //vcg::tri::UpdateTopology<>::VertexEdge(*sample);

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