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
#include "gemv2-foliage.h"

#include <ns3/log.h>
#include <boost/geometry/io/wkt/wkt.hpp>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("Gemv2Foliage");

namespace gemv2 {

Foliage::Foliage (Polygon2d const& shape)
  : m_shape (shape)
{
  // fix potential issues with shape
  boost::geometry::correct (m_shape);
  NS_LOG_LOGIC ("Created foliage with outline: " << boost::geometry::wkt (m_shape));

  // calculate bounding box
  boost::geometry::envelope (m_shape, m_boundingBox);
  NS_LOG_LOGIC ("Foliage bounding box: " << boost::geometry::wkt (m_boundingBox));

  // calculate area
  m_area = boost::geometry::area (m_shape);
  NS_LOG_LOGIC ("Area of the foliage object: " << m_area << " m^2");
}

Polygon2d const&
Foliage::GetShape () const
{
  return m_shape;
}

Box2d const&
Foliage::GetBoundingBox () const
{
  return m_boundingBox;
}

double
Foliage::GetArea () const
{
  return m_area;
}

}  // namespace gemv2
}  // namespace ns3
