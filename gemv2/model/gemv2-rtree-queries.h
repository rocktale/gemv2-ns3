/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 Karsten Roscher
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef GEMV2_RTREE_QUERIES_H
#define GEMV2_RTREE_QUERIES_H

#include <ns3/gemv2-geometry.h>
#include <ns3/gemv2-bounding-boxes.h>

namespace ns3
{
namespace gemv2
{

/*!
 * @brief Test if the provided geometry intersects with anything in the tree
 * @param tree	Tree to check
 * @param g	Geometry to test
 * @return True if @a g intersects at least with one object in @a tree
 */
template<typename TreeType, typename Geometry>
bool
IntersectsAny (const TreeType& tree, const Geometry& g)
{
  return tree.qbegin (boost::geometry::index::intersects (g)) != tree.qend ();
}

/*!
 * @brief Find objects within range around a position.
 *
 * This version of the method calculates the bounding box using
 * @c MakeBoundingBoxCircle based on @a position and @a range.
 *
 * @param tree		Tree to query
 * @param position	Position to query for
 * @param range		Maximum distance from @a position
 * @param result	All objects in @a tree with distance to @a position
 * 			less than @a range are added here
 */
template<typename TreeType, typename OutputType>
void
FindObjectsInRange (const TreeType& tree, const Point2d& position,
		    double range, OutputType& result)
{
  // make bounding box around circle
  Box2d bBox = MakeBoundingBoxCircle (position, range);

  // query the tree with bounding box and range condition
  FindObjectsInRange (tree, bBox, position, range, result);
}


/*!
 * @brief Find objects with maximum accumulated distance to two points.
 * @param tree		Tree to query
 * @param bBox		Bounding box to limit the search
 * @param p1		First focal point
 * @param p2		Second focal point
 * @param range		Maximum distance from @a p1 and @a p2 combined
 * @param result	All objects in @a tree with distance sum to @a p1 and
 * 			@a p2 less than @a range are added here
 */
template<typename TreeType, typename OutputType>
void
FindObjectsInEllipse (const TreeType& tree, const Box2d& bBox,
		      const Point2d& p1, const Point2d& p2,
		      double range, OutputType& result)
{
  // query the tree with bounding box and range condition
  tree.query (
      boost::geometry::index::intersects (bBox) &&
      boost::geometry::index::satisfies (
	  [range, p1, p2] (const typename TreeType::value_type& v)
	  {
	    return
		boost::geometry::distance (p1, v->GetShape ()) +
		boost::geometry::distance (p2, v->GetShape ())
		< range;
	  }),

	  std::back_inserter (result));
}

/*!
 * @brief Find objects with maximum accumulated distance to two points.
 *
 * This version of the method will calculate the bounding box based on the
 * provided points @a p1 and @a p2 as well as @a range using
 * MakeBoundingBoxEllipse().
 *
 * @param tree		Tree to query
 * @param p1		First focal point
 * @param p2		Second focal point
 * @param range		Maximum distance from @a p1 and @a p2 combined
 * @param result	All objects in @a tree with distance sum to @a p1 and
 * 			@a p2 less than @a range are added here
 */
template<typename TreeType, typename OutputType>
void
FindObjectsInEllipse (const TreeType& tree,
		      const Point2d& p1, const Point2d& p2,
		      double range, OutputType& result)
{
  // make bounding box around ellipse
  Box2d bBox = MakeBoundingBoxEllipse (p1, p2, range);

  // use bounding box for query
  FindObjectsInEllipse  (tree, bBox, p1, p2, range, result);
}



}  // namespace gemv2
}  // namespace ns3


#endif /* GEMV2_RTREE_QUERIES_H */
