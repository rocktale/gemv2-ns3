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
#ifndef GEMV2_VEHICLE_H
#define GEMV2_VEHICLE_H

#include <ns3/ptr.h>
#include <ns3/simple-ref-count.h>
#include <ns3/vector.h>
#include <ns3/gemv2-geometry.h>

namespace ns3 {
namespace gemv2 {

/*!
 * @brief A single vehicle within the GEMV^2 environment.
 *
 * Vehicles appear independent from nodes. Since there may be many
 * more vehicles in a scenario than actually communicating (equipped)
 * ones.
 */
class Vehicle : public SimpleRefCount<Vehicle>
{
public:
  /*!
   * @brief Create vehicle with the provided dimensions.
   *
   * Position and bearing are initialized to 0. The permittivity
   * is set to 6.0 for an approximation of glass, metal and rubber.
   *
   * @param length	Length of the vehicle [m]
   * @param width	Width of the vehicle [m]
   * @param height	Height of the vehicle [m]
   */
  Vehicle (double length, double width, double height);

  /*!
   * @brief Create vehicle with the provided shape.
   *
   * Position and bearing are initialized to 0. Thus, the shape should be
   * provided around the center point (0, 0, 0). The permittivity
   * is set to 6.0 for an approximation of glass, metal and rubber.
   *
   * @param shape	Shape of the vehicle, center must be at (0, 0, 0)
   * @param height	Height of the vehicle [m]
   */
  Vehicle (const Polygon2d& shape, double height);

  /*!
   * @brief Update the position of the vehicle.
   *
   * Shape and bounding box have to be recalculated after the update.
   *
   * @param position	New position
   */
  void
  SetPosition (const Vector& position);

  /*!
   * @brief Update heading of the vehicle.
   *
   * Shape and bounding box have to recalculated after the update.
   *
   * @param heading	Heading of the car in degrees from north
   */
  void
  SetHeading (double heading);

  /*!
   * @brief Get the height of the vehicle.
   * @return Height of the vehicle [m]
   */
  double
  GetHeight () const;

  /*!
   * @brief Get the shape of the vehicle
   * @return Shape of the vehicle
   */
  Polygon2d const&
  GetShape ();

  /*!
   * @brief Get the bounding box of the vehicle
   * @return Bounding box surrounding the current shape of the vehicle
   */
  Box2d const&
  GetBoundingBox ();

  /*!
   * @brief Get the relative permittivity of the vehicle surface
   * @return Relative permittivity value for the vehicle surface
   */
  double
  GetRelativePermittivity () const;

  /*!
   * @brief Set the relative permittivity of the vehicle surface
   * @param perm   Relative permittivity value for the vehicle surface
   */
  void
  SetRelativePermittivity (double perm);

private:

  /*!
   * @brief Check and update the value of the current shape.
   */
  void
  CheckUpdateShape ();

  //! Height of the vehicle
  double m_height;

  //! Position of the vehicle
  Vector m_position;

  //! Current heading of the vehicle
  double m_heading;

  //! Shape of the vehicle at the origin
  Polygon2d m_initialShape;

  /*!
   * @brief Shape of the vehicle at the current location and rotation
   *
   * This value will be recalculated on demand when it is accessed and
   * heading or position changed since the last time. This should reduce
   * the number of calculations if not needed.
   */
  Polygon2d m_currentShape;

  //! Indicate that the current shape needs to be recalculated.
  bool m_shapeUpdated;

  //! Bounding box of the building
  Box2d m_boundingBox;

  //! Relative permittivity of the vehicle surface
  double m_relativePermittivity;
};

}  // namespace gemv2
}  // namespace ns3

#endif /* GEMV2_VEHICLE_H */
