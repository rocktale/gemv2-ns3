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
namespace detail
{
// Specialize this with a AdapterType typedef to allow automatic
// selection of the shape adapter
template<typename T>
struct ShapeAdapterTrait {};
}

/*!
 * @brief Test if the provided geometry intersects with anything in the tree
 * @param tree		Tree to check
 * @param g		Geometry to test
 * @param shaper	Adapter to access the object shape
 * @return True if @a g intersects at least with one object in @a tree
 */
template<typename TreeType, typename Geometry,
	 typename ShapeAdapter = typename detail::ShapeAdapterTrait<TreeType>::AdapterType>
bool
IntersectsAny (const TreeType& tree, const Geometry& g,
	       ShapeAdapter shaper = ShapeAdapter ())
{
  return tree.qbegin (
      boost::geometry::index::intersects (g) &&
      boost::geometry::index::satisfies (
	  [&g, &shaper](const typename TreeType::value_type& v)
	  { return boost::geometry::intersects (shaper (v), g);}
	)
      ) != tree.qend ();
}

/*!
 * @brief Find objects intersecting a geometry.
 * @param tree			Tree to query
 * @param g			Geometry to check for intersections
 * @param outputIterator	All objects intersecting with @a g are added here
 * @param shaper		Adapter to access the shape of the stored object
 */
template<typename TreeType, typename Geometry,
         typename OutputIterator,
	 typename ShapeAdapter = typename detail::ShapeAdapterTrait<TreeType>::AdapterType>
void
FindObjectsThatIntersect (
    const TreeType& tree, const Geometry& g,
    OutputIterator outputIterator,
    ShapeAdapter shaper = ShapeAdapter ())
{
  tree.query (
      boost::geometry::index::intersects (g) &&
      boost::geometry::index::satisfies (
      	  [&g, &shaper](const typename TreeType::value_type& v)
      	  { return boost::geometry::intersects (shaper(v), g);}
      	),
	outputIterator);
}

/*!
 * @brief Find objects with maximum accumulated distance to two points.
 * @param tree		  Tree to query
 * @param bBox		  Bounding box to limit the search
 * @param p1		  First focal point
 * @param p2		  Second focal point
 * @param range		  Maximum distance from @a p1 and @a p2 combined
 * @param outputIterator  All objects in @a tree with distance sum to
 * 			  @a p1 and @a p2 less than @a range are added here
 * @param shaper	  Adapter to access the shape of the stored object
 */
template<typename TreeType, typename OutputIterator,
	 typename ShapeAdapter = typename detail::ShapeAdapterTrait<TreeType>::AdapterType>
void
FindObjectsInEllipse (const TreeType& tree, const Box2d& bBox,
		      const Point2d& p1, const Point2d& p2,
		      double range,
		      OutputIterator outputIterator,
		      ShapeAdapter shaper = ShapeAdapter ())
{
  // query the tree with bounding box and range condition
  tree.query (
      boost::geometry::index::intersects (bBox) &&
      boost::geometry::index::satisfies (
	  [range, &p1, &p2, &shaper] (const typename TreeType::value_type& v)
	  {
	    return
		boost::geometry::distance (p1, shaper (v)) +
		boost::geometry::distance (p2, shaper (v))
		< range;
	  }),
	  outputIterator);
}

/*!
 * @brief Find objects with maximum accumulated distance to two points.
 *
 * This version of the method will calculate the bounding box based on the
 * provided points @a p1 and @a p2 as well as @a range using
 * MakeBoundingBoxEllipse().
 *
 * @param tree		  Tree to query
 * @param p1		  First focal point
 * @param p2		  Second focal point
 * @param range		  Maximum distance from @a p1 and @a p2 combined
 * @param outputIterator  All objects in @a tree with distance sum to
 * 			  @a p1 and @a p2 less than @a range are added here
 * @param shaper	  Adapter to access the shape of the stored object
 */
template<typename TreeType, typename OutputIterator,
	 typename ShapeAdapter = typename detail::ShapeAdapterTrait<TreeType>::AdapterType>
void
FindObjectsInEllipse (const TreeType& tree,
		      const Point2d& p1, const Point2d& p2,
		      double range,
		      OutputIterator outputIterator,
		      ShapeAdapter shaper = ShapeAdapter ())
{
  // make bounding box around ellipse
  Box2d bBox = MakeBoundingBoxEllipse (p1, p2, range);

  // use bounding box for query
  FindObjectsInEllipse  (tree, bBox, p1, p2, range, outputIterator, shaper);
}



}  // namespace gemv2
}  // namespace ns3


#endif /* GEMV2_RTREE_QUERIES_H */
