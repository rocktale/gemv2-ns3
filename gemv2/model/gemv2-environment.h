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
#ifndef GEMV2_ENVIRONMENT_H
#define GEMV2_ENVIRONMENT_H

/*
 * Note: iostream is required due to a boost bug in 1.61
 *
 * rtree.hpp includes some other header files, one of them is
 * 'boost/geometry/algorithms/detail/overlay/handle_colocations.hpp' which
 * uses std::cout for some strange reason. Most probably, somebody forgot to
 * add the defines for debug output.
 *
 * There is already an open bug report:
 * https://svn.boost.org/trac/boost/ticket/12443
 */
#include <iostream>
#include <vector>

#include <ns3/ptr.h>
#include <ns3/simple-ref-count.h>
#include <ns3/nstime.h>

#include <ns3/gemv2-building.h>
#include <ns3/gemv2-foliage.h>
#include <ns3/gemv2-vehicle.h>

namespace ns3 {
namespace gemv2 {

/*!
 * @brief Class to manage the GEMV^2 environment.
 *
 * This class manages all objects (buildings, foliage, vehicles)
 * that influence the propagation behavior.
 */
class Environment : public SimpleRefCount<Environment>
{
public:

  /*
   * Some alias definitions
   */

  //! Generic list of objects stored using ns-3 pointers
  template<typename T>
  using PointerList = std::vector<Ptr<T>>;

  //! List of buildings
  using BuildingList = PointerList<Building>;

  //! List of foliage objects
  using FoliageList = PointerList<Foliage>;

  //! List of vehicles
  using VehicleList = PointerList<Vehicle>;

  //! Collection of objects
  struct ObjectCollection
  {
    BuildingList buildings;
    FoliageList foliage;
    VehicleList vehicles;
  };

  /*
   * Class members
   */

  /*!
   * @brief Create empty environment.
   */
  Environment();

  //! Destructor
  ~Environment();

  /*!
   * @brief Get global instance of the environment.
   * @return Global instance of the environment
   */
  static Ptr<Environment>
  GetGlobal ();


  /*!
   * @brief Set the rebuild interval for the vehicle tree.
   * @param t	Time after which the tree needs to be rebuild
   */
  void
  SetVehicleTreeRebuildInterval (Time t);

  /*!
   * @brief Add a building to the environment.
   * @param building	Building to add, must not be null
   */
  void
  AddBuilding (Ptr<Building> building);

  /*!
   * @brief Add buildings to the environment.
   * @param buildings	Buildings to add, must not be null
   */
  void
  AddBuildings (const BuildingList& buildings);

  /*!
   * @brief Add a foliage object to the environment.
   * @param foliage	Foliage to add, must not be null
   */
  void
  AddFoliage (Ptr<Foliage> foliage);

  /*!
   * @brief Add a vehicle to the environment.
   * @param vehicle	Vehicle to add, must not be null
   */
  void
  AddVehicle (Ptr<Vehicle> vehicle);

  /*!
   * @brief Remove vehicle from environment.
   * @param vehicle	Vehicle to remove, must not be null
   */
  void
  RemoveVehicle (Ptr<Vehicle> vehicle);

  /*!
   * @brief Force rebuild of the vehicle tree
   */
  void
  ForceVehicleTreeRebuild ();

  /*!
   * @brief Test if line intersects with any buildings
   * @param line 	Line to test
   * @return True if line intersects at least with one building
   */
  bool
  IntersectsAnyBuildings (const LineSegment2d& line) const;

  /*!
   * @brief Test if line intersects with any foliage
   * @param line 	Line to test
   * @return True if line intersects at least with one foliage object
   */
  bool
  IntersectsAnyFoliage (const LineSegment2d& line) const;

  /*!
   * @brief Calculate intersection of a line segment with buildings.
   * @param line		Line to calculate the intersections for
   * @return Buildings intersecting with @a line
   */
  BuildingList
  IntersectBuildings (const LineSegment2d& line) const;

  /*!
   * @brief Calculate intersection of a line segment with foliage objects.
   * @param line		Line to calculate the intersections for
   * @return Foliage objects intersecting with @a line
   */
  FoliageList
  IntersectFoliage (const LineSegment2d& line) const;

  /*!
   * @brief Calculate intersection of a line segment with vehicles.
   *
   * @note This method is not @c const since it may trigger rebuilding
   * 	   the internal vehicle tree.
   *
   * @param line		Line to calculate the intersections for
   * @return Vehicles intersecting with @a line
   */
  VehicleList
  IntersectVehicles (const LineSegment2d& line);

  /*!
   * @brief Find all buildings in ellipse.
   * @param p1			First focal point of the ellipse
   * @param p2			Second focal point of the ellipse
   * @param range		Length of the major diameter
   * @return Buildings where the sum of distance to @a p1 and @a p2 is less than @a range.
   */
  BuildingList
  FindBuildingsInEllipse (const Point2d& p1, const Point2d& p2, double range) const;

  /*!
   * @brief Find all foliage in ellipse.
   * @param p1			First focal point of the ellipse
   * @param p2			Second focal point of the ellipse
   * @param range		Length of the major diameter
   * @return Foliage where the sum of distance to @a p1 and @a p2 is less than @a range.
   */
  FoliageList
  FindFoliageInEllipse (const Point2d& p1, const Point2d& p2, double range) const;

  /*!
   * @brief Find all vehicles in ellipse.
   * @param p1			First focal point of the ellipse
   * @param p2			Second focal point of the ellipse
   * @param range		Length of the major diameter
   * @return Vehicles where the sum of distance to @a p1 and @a p2 is less than @a range.
   */
  VehicleList
  FindVehiclesInEllipse (const Point2d& p1, const Point2d& p2, double range);

  /*!
   * @brief Find all objects in ellipse.
   * @param p1			First focal point
   * @param p2			Second focal point
   * @param range		Maximum combined distance to @a p1 and @a p2
   * @return All objects where the sum of distance to @a p1 and @a p2 is less than @a range.
   */
  ObjectCollection
  FindAllObjectsInEllipse (const Point2d& p1, const Point2d& p2, double range);


  //! Internal data structures moved to the implementation file.
  struct Data;

private:

  /*!
   * @brief Check the status of the vehicle tree and rebuild if necessary.
   */
  void
  CheckVehcileTree ();

  // The environmental data
  std::unique_ptr<Data> m_data;

  //! Time of the last vehicle tree rebuild
  Time m_lastVehicleTreeRebuild;

  //! Interval for vehicle tree rebuilds
  Time m_vehicleTreeRebuildInterval;

  //! Force rebuild of the vehicle tree
  bool m_forceVehicleTreeRebuild;
};

}  // namespace gemv2
}  // namespace ns3

#endif /* GEMV2_ENVIRONMENT_H */
