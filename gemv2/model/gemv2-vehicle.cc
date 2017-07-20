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
#include "gemv2-vehicle.h"

#include <ns3/log.h>
#include <boost/geometry/io/wkt/wkt.hpp>

namespace {

//! Default relative permittiviy for vehicles
constexpr double DEFAULT_RELATIVE_PERMITTIVITY_VEHICLES = 6.0;

}

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("Gemv2Vehicle");

namespace gemv2 {

Vehicle::Vehicle (double length, double width, double height)
  : m_height (height),
    m_position (0, 0, 0),
    m_heading (0),
    m_shapeUpdated (true),
    m_relativePermittivity (DEFAULT_RELATIVE_PERMITTIVITY_VEHICLES)
{
  // generate basic shape based on width and length
  // for now, this is just a box
  std::vector<Point2d> pts =
      {
	  {-width/2, -length/2},
	  {-width/2, length/2},
	  {width/2, length/2},
	  {width/2, -length/2},
	  {-width/2, -length/2}
      };

  boost::geometry::append(m_initialShape, pts);
  boost::geometry::correct (m_initialShape);
  NS_LOG_LOGIC ("Created vehicle shape: " << boost::geometry::wkt (m_initialShape));

  // note current shape and bounding box will be updated on first access
}


Vehicle::Vehicle (const Polygon2d& shape, double height)
: m_height (height),
  m_position (0, 0, 0),
  m_heading (0),
  m_initialShape (shape),
  m_shapeUpdated (true),
  m_relativePermittivity (DEFAULT_RELATIVE_PERMITTIVITY_VEHICLES)
{
  boost::geometry::correct (m_initialShape);
  NS_LOG_LOGIC ("Created vehicle shape: " << boost::geometry::wkt (m_initialShape));

  // note current shape and bounding box will be updated on first access
}

void
Vehicle::SetPosition (const Vector& position)
{
  NS_LOG_FUNCTION (this << position);
  m_position = position;
  m_shapeUpdated = true;
}

void
Vehicle::SetHeading (double heading)
{
  NS_LOG_FUNCTION (this << heading);
  m_heading = heading;
  m_shapeUpdated = true;
}

double
Vehicle::GetHeight () const
{
  return m_height;
}

Polygon2d const&
Vehicle::GetShape ()
{
  CheckUpdateShape ();
  return m_currentShape;
}

Box2d const&
Vehicle::GetBoundingBox ()
{
  CheckUpdateShape ();
  return m_boundingBox;
}


double
Vehicle::GetRelativePermittivity () const
{
  return m_relativePermittivity;
}

void
Vehicle::SetRelativePermittivity (double perm)
{
  NS_LOG_FUNCTION (this << perm);
  m_relativePermittivity = perm;
}

void
Vehicle::CheckUpdateShape ()
{
  NS_LOG_FUNCTION (this);
  if (m_shapeUpdated)
    {
      NS_LOG_LOGIC ("Updating vehicle shape");
      // first step: rotate by heading
      RotateDegree2d rotate (m_heading);
      Polygon2d rotatedShape;
      boost::geometry::transform (m_initialShape, rotatedShape, rotate);

      // second step: translate to position
      Translate2d translate (m_position.x, m_position.y);
      boost::geometry::transform (rotatedShape, m_currentShape, translate);

      NS_LOG_LOGIC ("New shape: " << boost::geometry::wkt (m_currentShape));

      // third step: update bounding box
      boost::geometry::envelope (m_currentShape, m_boundingBox);
      NS_LOG_LOGIC (
	  "New vehicle bounding box: " << boost::geometry::wkt (m_boundingBox));

      m_shapeUpdated = false;
    }
}


}  // namespace gemv2
}  // namespace ns3
