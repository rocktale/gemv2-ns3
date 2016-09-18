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

#include <boost/geometry/io/wkt/wkt.hpp>

#include <ns3/log.h>
#include <ns3/assert.h>


namespace {

/*!
 * @brief Generic find in range for various tree types.
 * @param tree		Tree to query
 * @param position	Position to query for
 * @param range		Maximum distance from @a position
 * @param result	Found objects
 */
template<typename TreeType, typename OutputType>
void
GenericFindInRange (const TreeType& tree, const ns3::gemv2::Point2d& position,
		    double range, OutputType& result)
{
  // make bounding box around circle
  ns3::gemv2::Box2d bBox =
      {{position.x () - range/2, position.y () - range/2},
       {position.x () + range/2, position.y () + range/2}};

  // query the tree with bounding box and range condition
  tree.query (
      boost::geometry::index::intersects (bBox) &&
      boost::geometry::index::satisfies (
	  [range, position](const typename TreeType::value_type& v)
	  {
	    return boost::geometry::distance (position, v->GetShape ()) <= range;
	  }),

	  std::back_inserter (result));
}

}

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
}

void
Environment::FindInRange (const Point2d& position, double range,
			  BuildingList& outBuildings)
{
  NS_LOG_FUNCTION (this << boost::geometry::wkt (position) << range);
  GenericFindInRange (m_buildings, position, range, outBuildings);
  NS_LOG_LOGIC ("Found " << outBuildings.size () << " buildings within "
		<< range << "m around " << boost::geometry::wkt (position));
}

void
Environment::FindInRange (const Point2d& position, double range,
			  FoliageList& outFoliage)
{
  NS_LOG_FUNCTION (this << boost::geometry::wkt (position) << range);
  GenericFindInRange (m_foliage, position, range, outFoliage);
  NS_LOG_LOGIC ("Found " << outFoliage.size () << " foliage objects within "
		<< range << "m around " << boost::geometry::wkt (position));
}


void
Environment::FindInRange (const Point2d& position, double range,
			  VehicleList& outVehicles)
{
  NS_LOG_FUNCTION (this << boost::geometry::wkt (position) << range);
  // TODO: implement this
}

}  // namespace gemv2
}  // namespace ns3
