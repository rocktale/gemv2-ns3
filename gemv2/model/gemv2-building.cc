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
#include "gemv2-building.h"

#include <boost/geometry/io/wkt/wkt.hpp>
#include <ns3/log.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("Gemv2Building");

namespace gemv2 {

Building::Building (Polygon2d const& shape)
  : m_shape (shape),
    m_relativePermittivity (4.5)
{
  // fix potential issues with shape
  boost::geometry::correct (m_shape);
  NS_LOG_LOGIC ("Created building with outline: " << boost::geometry::wkt (m_shape));

  // calculate bounding box
  boost::geometry::envelope (m_shape, m_boundingBox);
  NS_LOG_LOGIC ("Building bounding box: " << boost::geometry::wkt (m_boundingBox));
}

Polygon2d const&
Building::GetShape () const
{
  return m_shape;
}

Box2d const&
Building::GetBoundingBox () const
{
  return m_boundingBox;
}

double
Building::GetRelativePermittivity () const
{
  return m_relativePermittivity;
}

void
Building::SetRelativePermittivity (double perm)
{
  m_relativePermittivity = perm;
}

}  // namespace gemv2
}  // namespace ns3
