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

#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>
#include <boost/function_output_iterator.hpp>

#include <ns3/log.h>
#include <ns3/assert.h>
#include <ns3/simulator.h>

#include <ns3/gemv2-rtree-queries.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("Gemv2Environment");

namespace gemv2 {

/*
 * Tree definitions and data structures.
 */
struct Environment::Data
{
  // small indexer for the trees to allow using pointers directly
  template <typename T>
  struct PtrIndex
  {
    using V = ns3::Ptr<T>;
    using result_type = ns3::gemv2::Box2d const&;	// required for rtree
    result_type operator()(V const& v) const { return v->GetBoundingBox (); }
  };

  /*!
   * @brief Adapter to access the shape directly on the provided tree object.
   *
   * This uses ptr->GetShape () to access the shape of the object.
   */
  template<typename TreeType>
  struct DirectShapeAdapter
  {
    auto
    operator() (const typename TreeType::value_type& v) -> decltype (v->GetShape ())
    {
      return v->GetShape ();
    }
  };

  /*!
   * @brief Type of a range tree for buildings
   *
   * The tree will only be build on startup. Thus, we use the more
   * expensive r-star algorithm here since it provides better performance
   * for queries.
   */
  using BuildingTree =
      boost::geometry::index::rtree<
      Ptr<Building>, boost::geometry::index::rstar<16>, PtrIndex<Building>>;

  //! The range tree containing all buildings
  BuildingTree buildings;

  /*!
   * @brief Type of a range tree for foliage
   *
   * The tree will only be build on startup. Thus, we use the more
   * expensive r-star algorithm here since it provides better performance
   * for queries.
   */
  using FoliageTree =
      boost::geometry::index::rtree<
      Ptr<Foliage>, boost::geometry::index::rstar<16>, PtrIndex<Foliage>>;

  //! The range tree containing all foliage objects
  FoliageTree foliage;

  //! Set of all registered vehicles
  std::set<Ptr<Vehicle>> vehicles;

  //! Vehicle and a bounding box
  using BoxedVehicle = std::pair<Box2d, Ptr<Vehicle>>;

  //! Type of the tree for vehicle search
  using VehicleTree =
      boost::geometry::index::rtree<
      BoxedVehicle, boost::geometry::index::quadratic<16>>;

  //! Shape adapter for vehicles
  struct VehicleShapeAdapter
  {
    auto
    operator() (const VehicleTree::value_type& v)
      -> decltype (v.second->GetShape ())
    {
      return v.second->GetShape ();
    }
  };

  //! Current tree of assigned vehicles
  VehicleTree vehicleTree;
};

/*
 * Adapter definitions for tree queries
 */
namespace detail
{

template<>
struct ShapeAdapterTrait<Environment::Data::BuildingTree>
{
  using AdapterType =
      Environment::Data::DirectShapeAdapter<Environment::Data::BuildingTree>;
};

template<>
struct ShapeAdapterTrait<Environment::Data::FoliageTree>
{
  using AdapterType =
      Environment::Data::DirectShapeAdapter<Environment::Data::FoliageTree>;
};

template<>
struct ShapeAdapterTrait<Environment::Data::VehicleTree>
{
  using AdapterType = Environment::Data::VehicleShapeAdapter;
};

}  // namespace detail


/*
 * And now the actual Environment implementation
 */

Environment::Environment()
  : m_data (new Data),
    m_lastVehicleTreeRebuild (-1.0),
    m_vehicleTreeRebuildInterval (Seconds (1.0)),
    m_forceVehicleTreeRebuild (false)
{
}

// Should be created here to have the deleter of the data object
Environment::~Environment() = default;

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
Environment::SetVehicleTreeRebuildInterval (Time t)
{
  m_vehicleTreeRebuildInterval = t;
}

void
Environment::AddBuilding (Ptr<Building> building)
{
  NS_ASSERT_MSG (building, "building must not be null");
  m_data->buildings.insert (building);
}

void
Environment::AddBuildings (const BuildingList& buildings)
{
  for (auto const& b : buildings)
    {
      NS_ASSERT_MSG (b, "building must not be null");
    }
  m_data->buildings.insert (buildings.begin (), buildings.end ());
}


void
Environment::AddFoliage (Ptr<Foliage> foliage)
{
  NS_ASSERT_MSG (foliage, "foliage must not be null");
  m_data->foliage.insert (foliage);
}

void
Environment::AddVehicle (Ptr<Vehicle> vehicle)
{
  NS_ASSERT_MSG (vehicle, "vehicle must not be null");
  m_data->vehicles.insert (vehicle);
  m_forceVehicleTreeRebuild = true;
}

void
Environment::RemoveVehicle (Ptr<Vehicle> vehicle)
{
  NS_ASSERT_MSG (vehicle, "vehicle must not be null");
  m_data->vehicles.erase (vehicle);
  m_forceVehicleTreeRebuild = true;
}

void
Environment::ForceVehicleTreeRebuild ()
{
  m_forceVehicleTreeRebuild = true;
}

bool
Environment::IntersectsAnyBuildings (const LineSegment2d& line) const
{
  NS_LOG_FUNCTION (this << boost::geometry::wkt (line));
  return IntersectsAny (m_data->buildings, line);
}

bool
Environment::IntersectsAnyFoliage (const LineSegment2d& line) const
{
  NS_LOG_FUNCTION (this << boost::geometry::wkt (line));
  return IntersectsAny (m_data->foliage, line);
}

Environment::BuildingList
Environment::IntersectBuildings (const LineSegment2d& line) const
{
  NS_LOG_FUNCTION (this << boost::geometry::wkt (line));
  BuildingList intersectingBuildings;
  FindObjectsThatIntersect (m_data->buildings, line,
			    std::back_inserter(intersectingBuildings));
  NS_LOG_LOGIC ("Found " << intersectingBuildings.size ()
		<< " intersections with buildings");
  return intersectingBuildings;
}

Environment::FoliageList
Environment::IntersectFoliage (const LineSegment2d& line) const
{
  NS_LOG_FUNCTION (this << boost::geometry::wkt (line));
  FoliageList intersectingFoliage;
  FindObjectsThatIntersect (m_data->foliage, line,
			    std::back_inserter(intersectingFoliage));
  NS_LOG_LOGIC ("Found " << intersectingFoliage.size ()
		<< " intersections with foliage");
  return intersectingFoliage;
}

Environment::VehicleList
Environment::IntersectVehicles (const LineSegment2d& line)
{
  NS_LOG_FUNCTION (this << boost::geometry::wkt (line));

  // update the vehicle tree if necessary
  CheckVehcileTree ();

  VehicleList intersectingVehicles;

  FindObjectsThatIntersect (
      m_data->vehicleTree, line,
      boost::make_function_output_iterator(
	  [&intersectingVehicles](const typename Data::VehicleTree::value_type& v)
	  { intersectingVehicles.push_back (v.second); }));

  NS_LOG_LOGIC ("Found " << intersectingVehicles.size ()
		<< " intersections with vehicles");

  return intersectingVehicles;
}

void
Environment::FindInEllipse (const Point2d& p1, const Point2d& p2, double range,
			    BuildingList& outBuildings)
{
  NS_LOG_FUNCTION (
      this << boost::geometry::wkt (p1) << boost::geometry::wkt (p2) << range);

  FindObjectsInEllipse (
      m_data->buildings, p1, p2, range,
      std::back_inserter(outBuildings));

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

  FindObjectsInEllipse (
      m_data->foliage, p1, p2, range,
      std::back_inserter(outFoliage));

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

  CheckVehcileTree ();
  FindObjectsInEllipse (
      m_data->vehicleTree, p1, p2, range,
      boost::make_function_output_iterator(
      	  [&outVehicles](const typename Data::VehicleTree::value_type& v)
      	  { outVehicles.push_back (v.second); })
      );

  NS_LOG_LOGIC ("Found " << outVehicles.size () << " vehicles in ellipse r="
		<< range << "m around " << boost::geometry::wkt (p1)
		<< " and " << boost::geometry::wkt (p2));
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
  FindObjectsInEllipse (
      m_data->buildings, bBox, p1, p2, range,
      std::back_inserter(outObjects.buildings));

  // collect foliage
  FindObjectsInEllipse (
      m_data->foliage, bBox, p1, p2, range,
      std::back_inserter(outObjects.foliage));

  // collect vehicles
  CheckVehcileTree ();
  FindObjectsInEllipse (
      m_data->vehicleTree, bBox, p1, p2, range,
      boost::make_function_output_iterator(
      	  [&outObjects](const typename Data::VehicleTree::value_type& v)
      	  { outObjects.vehicles.push_back (v.second); })
      );
}

void
Environment::CheckVehcileTree ()
{
  if (m_forceVehicleTreeRebuild ||
      (m_lastVehicleTreeRebuild + m_vehicleTreeRebuildInterval < Simulator::Now ()))
    {
      NS_LOG_LOGIC ("Rebuilding vehicle tree");

      // clear existing tree
      m_data->vehicleTree.clear ();

      // add all vehicles to the tree
      for (auto v : m_data->vehicles)
	{
	  // insert vehicle with updated bounding box
	  // TODO: enlarge bounding box to compensate for movement between updates
	  m_data->vehicleTree.insert (std::make_pair (v->GetBoundingBox (), v));
	}

      m_lastVehicleTreeRebuild = Simulator::Now ();
      m_forceVehicleTreeRebuild = false;
    }
}

}  // namespace gemv2
}  // namespace ns3
