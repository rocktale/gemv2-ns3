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
#include "gemv2-bounding-boxes.h"

namespace ns3 {
namespace gemv2 {

Box2d
MakeBoundingBoxCircle (const Point2d& center, double radius)
{
  double range = radius / 2;
  return Box2d (Point2d (center.x () - range, center.y () - range),
		Point2d (center.y () + range, center.y () + range));
}

Box2d
MakeBoundingBoxEllipse (const Point2d& p1, const Point2d& p2, double range)
{
  double padding = (range - boost::geometry::distance (p1, p2)) / 2;

  /*
   * Make bounding box around ellipse
   *
   * This approach might be over conservative and leads to a
   * box larger than the area covered by the ellipse. However,
   * getting a tight box requires more computations beforehand.
   * I'm not sure if it is worth it.
   */
  return Box2d
      (Point2d (std::min(p1.x (), p2.x ()) - padding,
		std::min(p1.y (), p2.y ()) - padding),
       Point2d (std::max(p1.x (), p2.x ()) + padding,
		std::max(p1.y (), p2.y ()) + padding));
}

}  // namespace gemv2
}  // namespace ns3
