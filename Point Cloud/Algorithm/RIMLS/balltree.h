/****************************************************************************
* MeshLab                                                           o o     *
* A versatile mesh processing toolbox                             o     o   *
*                                                                _   O  _   *
* Copyright(C) 2005                                                \/)\/    *
* Visual Computing Lab                                            /\/|      *
* ISTI - Italian National Research Council                           |      *
*                                                                    \      *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *
* it under the terms of the GNU General Public License as published by      *
* the Free Software Foundation; either version 2 of the License, or         *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/

#ifndef BALLTREE_H
#define BALLTREE_H

#include <vcg/space/point3.h>
#include <vcg/space/box3.h>
#include "mlsutils.h"



namespace vcg {

  template <typename Scalar>
  inline Point3<Scalar> CwiseMul(Point3<Scalar> const & p1, Point3<Scalar> const & p2)
  {
    return Point3<Scalar>(p1.X()*p2.X(), p1.Y()*p2.Y(), p1.Z()*p2.Z());
  }

  template <typename Scalar>
  inline Point3<Scalar> Min(Point3<Scalar> const & p1, Point3<Scalar> const & p2)
  {
    return Point3<Scalar>(std::min(p1.X(), p2.X()), std::min(p1.Y(), p2.Y()), std::min(p1.Z(), p2.Z()));
  }

  template <typename Scalar>
  inline Point3<Scalar> Max(Point3<Scalar> const & p1, Point3<Scalar> const & p2)
  {
    return Point3<Scalar>(std::max(p1.X(), p2.X()), std::max(p1.Y(), p2.Y()), std::max(p1.Z(), p2.Z()));
  }

  //template <typename Scalar>
  //inline Scalar MaxCoeff(Point3<Scalar> const & p)
  //{
  //  return std::max(std::max(p.X(), p.Y()), p.Z());
  //}

  //template <typename Scalar>
  //inline Scalar MinCoeff(Point3<Scalar> const & p)
  //{
  //  return std::min(std::min(p.X(), p.Y()), p.Z());
  //}

  template <typename Scalar>
  inline Scalar MyDot(Point3<Scalar> const & p1, Point3<Scalar> const & p2)
  {
    return p1.X() * p2.X() + p1.Y() * p2.Y() + p1.Z() * p2.Z();
  }

  template <typename Scalar>
  inline Point3<Scalar> Cross(Point3<Scalar> const & p1, Point3<Scalar> const & p2)
  {
    return p1 ^ p2;
  }

  template <typename Scalar>
  inline Point3<Scalar> CwiseAdd(Point3<Scalar> const & p1, Scalar s)
  {
    return Point3<Scalar>(p1.X() + s, p1.Y() + s, p1.Z() + s);
  }

  template <typename Scalar>
  inline int MyMaxCoeffId(Point3<Scalar> const & p)
  {
    if (p.X()>p.Y())
      return p.X()>p.Z() ? 0 : 2;
    else
      return p.Y()>p.Z() ? 1 : 2;
  }

  //template <typename Scalar>
  //inline int MinCoeffId(Point3<Scalar> const & p)
  //{
  //  if (p.X()<p.Y())
  //    return p.X()<p.Z() ? 0 : 2;
  //  else
  //    return p.Y()<p.Z() ? 1 : 2;
  //}

  template <typename ToType, typename Scalar>
  inline Point3<ToType> Point3Cast(const Point3<Scalar>& p)
  {
    return Point3<ToType>(p.X(), p.Y(), p.Z());
  }

  template<class Scalar>
  Scalar Distance(const Point3<Scalar> &p, const Box3<Scalar> &bbox)
  {
    Scalar dist2 = 0.;
    Scalar aux;
    for (int k=0 ; k<3 ; ++k)
    {
      if ( (aux = (p[k]-bbox.min[k]))<0. )
        dist2 += aux*aux;
      else if ( (aux = (bbox.max[k]-p[k]))<0. )
        dist2 += aux*aux;
    }
    return sqrt(dist2);
  }

}






namespace GaelMls {

template<typename _Scalar>
class Neighborhood
{
	public:
		typedef _Scalar Scalar;

		int index(int i) const { return mIndices.at(i); }
		Scalar squaredDistance(int i) const { return mSqDists.at(i); }

		void clear() { mIndices.clear(); mSqDists.clear(); }
		void resize(int size) { mIndices.resize(size); mSqDists.resize(size); }
		void reserve(int size) { mIndices.reserve(size); mSqDists.reserve(size); }
		int size() { return mIndices.size(); }

		void insert(int id, Scalar d2) { mIndices.push_back(id); mSqDists.push_back(d2); }

	protected:
		std::vector<int> mIndices;
		std::vector<Scalar> mSqDists;
};

template<typename _Scalar>
class BallTree
{
	public:
		typedef _Scalar Scalar;
		typedef vcg::Point3<Scalar> VectorType;

		BallTree(const ConstDataWrapper<VectorType>& points, const ConstDataWrapper<Scalar>& radii);

		void computeNeighbors(const VectorType& x, Neighborhood<Scalar>* pNei) const;

		void setRadiusScale(Scalar v) { mRadiusScale = v; mTreeIsUptodate = false; }

	protected:

		struct Node
		{
			~Node()
			{
				if (!leaf)
				{
					delete children[0];
					delete children[1];
				}
				else
				{
					delete[] indices;
				}
			}
			Scalar splitValue;
			unsigned char dim:2;
			unsigned char leaf:1;
			union {
				Node* children[2];
				struct {
					unsigned int* indices;
					unsigned int size;
				};
			};
		};

		typedef std::vector<int> IndexArray;
		typedef vcg::Box3<Scalar> AxisAlignedBoxType;

		void rebuild();
		void split(const IndexArray& indices, const AxisAlignedBoxType& aabbLeft, const AxisAlignedBoxType& aabbRight,
							IndexArray& iLeft, IndexArray& iRight);
		void buildNode(Node& node, std::vector<int>& indices, AxisAlignedBoxType aabb, int level);
		void queryNode(Node& node, Neighborhood<Scalar>* pNei) const;

	protected:
		ConstDataWrapper<VectorType> mPoints;
		ConstDataWrapper<Scalar> mRadii;
		Scalar mRadiusScale;

		int mMaxTreeDepth;
		int mTargetCellSize;
		mutable bool mTreeIsUptodate;
		mutable VectorType mQueryPosition;

		Node* mRootNode;
};


template<typename _Scalar>
BallTree<_Scalar>::BallTree(const ConstDataWrapper<VectorType>& points, const ConstDataWrapper<Scalar>& radii)
  : mPoints(points), mRadii(radii), mRadiusScale(1.), mTreeIsUptodate(false)
{
  mRootNode = 0;
  mMaxTreeDepth = 12;
  mTargetCellSize = 24;
}

template<typename _Scalar>
void BallTree<_Scalar>::computeNeighbors(const VectorType& x, Neighborhood<Scalar>* pNei) const
{
  if (!mTreeIsUptodate)
    const_cast<BallTree*>(this)->rebuild();

  pNei->clear();
  mQueryPosition = x;
  queryNode(*mRootNode, pNei);
}

template<typename _Scalar>
void BallTree<_Scalar>::queryNode(Node& node, Neighborhood<Scalar>* pNei) const
{
  if (node.leaf)
  {
    for (unsigned int i=0 ; i<node.size ; ++i)
    {
      int id = node.indices[i];
      Scalar d2 = vcg::SquaredNorm(mQueryPosition - mPoints[id]);
      Scalar r = mRadiusScale * mRadii[id];
      if (d2<r*r)
        pNei->insert(id, d2);
    }
  }
  else
  {
    if (mQueryPosition[node.dim] - node.splitValue < 0)
      queryNode(*node.children[0], pNei);
    else
      queryNode(*node.children[1], pNei);
  }
}

template<typename _Scalar>
void BallTree<_Scalar>::rebuild(void)
{
  delete mRootNode;

  mRootNode = new Node();
  IndexArray indices(mPoints.size());
  AxisAlignedBoxType aabb;
  aabb.Set(mPoints[0]);
  for (unsigned int i=0 ; i<mPoints.size() ; ++i)
  {
    indices[i] = i;
    aabb.min = vcg::Min(aabb.min, CwiseAdd(mPoints[i], -mRadii[i]*mRadiusScale));
    aabb.max = Max(aabb.max, CwiseAdd(mPoints[i],  mRadii[i]*mRadiusScale));
  }
  buildNode(*mRootNode, indices, aabb, 0);

  mTreeIsUptodate = true;
}

template<typename _Scalar>
void BallTree<_Scalar>::split(const IndexArray& indices, const AxisAlignedBoxType& aabbLeft, const AxisAlignedBoxType& aabbRight, IndexArray& iLeft, IndexArray& iRight)
{
  for (std::vector<int>::const_iterator it=indices.begin(), end=indices.end() ; it!=end ; ++it)
  {
    unsigned int i = *it;
    if (vcg::Distance(mPoints[i], aabbLeft) < mRadii[i]*mRadiusScale)
      iLeft.push_back(i);

    if (vcg::Distance(mPoints[i], aabbRight) < mRadii[i]*mRadiusScale)
      iRight.push_back(i);
  }
}

template<typename _Scalar>
void BallTree<_Scalar>::buildNode(Node& node, std::vector<int>& indices, AxisAlignedBoxType aabb, int level)
{
  Scalar avgradius = 0.;
  for (std::vector<int>::const_iterator it=indices.begin(), end=indices.end() ; it!=end ; ++it)
    avgradius += mRadii[*it];
  avgradius = mRadiusScale * avgradius / Scalar(indices.size());
  VectorType diag = aabb.max - aabb.min;
  if  (int(indices.size())<mTargetCellSize
    || avgradius*0.9 > std::max(std::max(diag.X(), diag.Y()), diag.Z())
    || int(level)>=mMaxTreeDepth)
  {
    node.leaf = true;
    node.size = indices.size();
    node.indices = new unsigned int[node.size];
    for (unsigned int i=0 ; i<node.size ; ++i)
      node.indices[i] = indices[i];
    return;
  }
  unsigned int dim = vcg::MyMaxCoeffId(diag);
  node.dim = dim;
  node.splitValue = Scalar(0.5*(aabb.max[dim] + aabb.min[dim]));
  node.leaf = 0;

  AxisAlignedBoxType aabbLeft=aabb, aabbRight=aabb;
  aabbLeft.max[dim] = node.splitValue;
  aabbRight.min[dim] = node.splitValue;

  std::vector<int> iLeft, iRight;
  split(indices, aabbLeft, aabbRight, iLeft,iRight);

  // we don't need the index list anymore
  indices.clear();

  {
    // left child
    //mNodes.resize(mNodes.size()+1);
    Node* pChild = new Node();
    node.children[0] = pChild;
    buildNode(*pChild, iLeft, aabbLeft, level+1);
  }

  {
    // right child
    //mNodes.resize(mNodes.size()+1);
    Node* pChild = new Node();
    node.children[1] = pChild;
    buildNode(*pChild, iRight, aabbRight, level+1);
  }
}

template class BallTree<float>;
template class BallTree<double>;


}
//#include "balltree.tpp"
#endif
