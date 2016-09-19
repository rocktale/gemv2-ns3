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
#include "gemv2-environment.h"

#include <algorithm>
#include <boost/geometry/io/wkt/wkt.hpp>

#include <ns3/log.h>
#include <ns3/assert.h>
#include <ns3/gemv2-rtree-queries.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("Gemv2Environment");

namespace gemv2 {

Environment::Environment()
{
}

Ptr<Environment>
Environment::GetGlobal ()
{
  static Ptr<Environment> e;
  if (!e)
    {
      e = Create<Environment> ();
    }

  return e;
}

void
Environment::AddBuilding (Ptr<Building> building)
{
  NS_ASSERT_MSG (building, "building must not be null");
  m_buildings.insert (building);
}

void
Environment::AddBuildings (const BuildingList& buildings)
{
  for (auto const& b : buildings)
    {
      NS_ASSERT_MSG (b, "building must not be null");
    }
  m_buildings.insert (buildings.begin (), buildings.end ());
}


void
Environment::AddFoliage (Ptr<Foliage> foliage)
{
  NS_ASSERT_MSG (foliage, "foliage must not be null");
  m_foliage.insert (foliage);
}

bool
Environment::IntersectsBuildings (const LineSegment2d& line) const
{
  NS_LOG_FUNCTION (this << boost::geometry::wkt (line));
  return IntersectsAny (m_buildings, line);
}

bool
Environment::IntersectsFoliage (const LineSegment2d& line) const
{
  NS_LOG_FUNCTION (this << boost::geometry::wkt (line));
  return IntersectsAny (m_foliage, line);
}


void
Environment::Intersect (const LineSegment2d& line, BuildingList& outBuildings)
{
  NS_LOG_FUNCTION (this << boost::geometry::wkt (line));

  m_buildings.query (
      boost::geometry::index::intersects (line),
      std::back_inserter (outBuildings));

  NS_LOG_LOGIC ("Found " << outBuildings.size ()
		<< " intersections with buildings");
}

void
Environment::Intersect (const LineSegment2d& line, FoliageList& outFoliage)
{
  NS_LOG_FUNCTION (this << boost::geometry::wkt (line));

  m_foliage.query (
      boost::geometry::index::intersects (line),
      std::back_inserter (outFoliage));

  NS_LOG_LOGIC ("Found " << outFoliage.size ()
		<< " intersections with foliage");
}

void
Environment::Intersect (const LineSegment2d& line, VehicleList& outVehicles)
{
  NS_LOG_FUNCTION (this << boost::geometry::wkt (line));

  // TODO: implement this
  NS_LOG_WARN ("Not implemented (yet)");
}

void
Environment::FindInEllipse (const Point2d& p1, const Point2d& p2, double range,
			    BuildingList& outBuildings)
{
  NS_LOG_FUNCTION (
      this << boost::geometry::wkt (p1) << boost::geometry::wkt (p2) << range);

  FindObjectsInEllipse (m_buildings, p1, p2, range, outBuildings);

  NS_LOG_LOGIC ("Found " << outBuildings.size () << " buildings in ellipse r="
		<< range << "m around " << boost::geometry::wkt (p1)
		<< " and " << boost::geometry::wkt (p2));
}

void
Environment::FindInEllipse (const Point2d& p1, const Point2d& p2, double range,
		 FoliageList& outFoliage)
{
  NS_LOG_FUNCTION (
      this << boost::geometry::wkt (p1) << boost::geometry::wkt (p2) << range);

  FindObjectsInEllipse (m_foliage, p1, p2, range, outFoliage);

  NS_LOG_LOGIC ("Found " << outFoliage.size () << " foliage objects in ellipse r="
		<< range << "m around " << boost::geometry::wkt (p1)
		<< " and " << boost::geometry::wkt (p2));
}


void
Environment::FindInEllipse (const Point2d& p1, const Point2d& p2, double range,
		 VehicleList& outVehicles)
{
  NS_LOG_FUNCTION (
      this << boost::geometry::wkt (p1) << boost::geometry::wkt (p2) << range);

  // TODO: implement this
  NS_LOG_WARN ("Not implemented (yet)");
}

void
Environment::FindAllInEllipse (const Point2d& p1, const Point2d& p2, double range,
			       ObjectCollection& outObjects)
{
  NS_LOG_FUNCTION (
      this << boost::geometry::wkt (p1) << boost::geometry::wkt (p2) << range);

  // Calculate bounding box around ellipse
  auto bBox = MakeBoundingBoxEllipse (p1, p2, range);

  NS_LOG_LOGIC ("Bounding box: " << boost::geometry::wkt (bBox));

  // collect buildings
  FindObjectsInEllipse (m_buildings, bBox, p1, p2, range, outObjects.buildings);

  // collect foliage
  FindObjectsInEllipse (m_foliage, bBox, p1, p2, range, outObjects.foliage);

  // TODO: collect vehicles

}

}  // namespace gemv2
}  // namespace ns3
