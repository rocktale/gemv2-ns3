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
#ifndef GEMV2_GEOMETRY_H
#define GEMV2_GEOMETRY_H

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>

namespace ns3 {
namespace gemv2 {

/*
 * Types
 */

//! A 2D point
typedef boost::geometry::model::d2::point_xy<double> Point2d;

//! A 2D line segment
typedef boost::geometry::model::segment<Point2d> LineSegment2d;

//! A 2D box
typedef boost::geometry::model::box<Point2d> Box2d;

//! A 2D polygon
typedef boost::geometry::model::polygon<Point2d> Polygon2d;

/*
 * Common transformations
 */

//! Linear translation in 2D
typedef boost::geometry::strategy::transform::translate_transformer<
    double, 2, 2> Translate2d;

//! Rotation by degrees in 2D
typedef boost::geometry::strategy::transform::rotate_transformer<
    boost::geometry::degree, double, 2, 2> RotateDegree2d;


/*
 * Utility functions
 */

/*!
 * @brief Make 2D point from vector.
 * @param v	Vector to parse
 * @return Point with x and y coordinates of @a v.
 */
template<typename VectorType>
Point2d
MakePoint2d (const VectorType& v)
{
  return Point2d (v.x, v.y);
}

}  // namespace gemv2
}  // namespace ns3

#endif /* GEMV2_GEOMETRY_H */
