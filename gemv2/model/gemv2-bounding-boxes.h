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
#ifndef GEMV2_BOUNDING_BOXES_H
#define GEMV2_BOUNDING_BOXES_H

#include <ns3/gemv2-geometry.h>

namespace ns3
{
namespace gemv2
{

/*!
 * @brief Make bounding box around a circle
 * @param center	Center of the circle
 * @param radius	Radius of the circle
 * @return Bounding box enclosing the circle
 */
Box2d
MakeBoundingBoxCircle (const Point2d& center, double radius);

/*!
 * @brief Make bounding box around communication ellipse
 *
 * This bounding box assumes that the major diameter of the ellipse is
 * @a range. @a p1 and @a p2 are the focal point and the ellipse is described
 * by all points p where: distance(p, p1) + distance(p, p2) < range
 *
 * @param p1		First point
 * @param p2		Second point
 * @param range		Length of the major diameter of the ellipse
 * @return Bounding box around the ellipse
 */
Box2d
MakeBoundingBoxEllipse (const Point2d& p1, const Point2d& p2, double range);

}  // namespace gemv2
}  // namespace ns3


#endif /* GEMV2_BOUNDING_BOXES_H */
