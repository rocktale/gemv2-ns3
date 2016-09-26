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
#ifndef GEMV2_BUILDING_H
#define GEMV2_BUILDING_H

#include <ns3/simple-ref-count.h>
#include <ns3/gemv2-geometry.h>

namespace ns3 {
namespace gemv2 {

/*!
 * @brief A single building within the GEMV^2 environment.
 */
class Building : public SimpleRefCount<Building>
{
public:
  /*!
   * @brief Create a building from the provided shape.
   *
   * The relative permittivity value is set to 4.5 for concrete.
   */
  explicit Building (Polygon2d const& shape);

  /*!
   * @brief Get the shape of the building
   * @return Shape of the building
   */
  Polygon2d const&
  GetShape () const;

  /*!
   * @brief Get the bounding box of the building
   * @return Bounding box of the building
   */
  Box2d const&
  GetBoundingBox () const;

  /*!
   * @brief Get the area covered by the building
   * @return Area of the building in m^2
   */
  double
  GetArea () const;

  /*!
   * @brief Get the relative permittivity of the building surface
   * @return Relative permittivity value for the building surface
   */
  double
  GetRelativePermittivity () const;

  /*!
   * @brief Set the relative permittivity of the building surface
   * @param perm   Relative permittivity value for the building surface
   */
  void
  SetRelativePermittivity (double perm);

private:

  //! Shape of the building
  Polygon2d m_shape;

  //! Bounding box of the building
  Box2d m_boundingBox;

  //! Area covered by the building
  double m_area;

  //! Relative permittivity of the buildings surface
  double m_relativePermittivity;
};

}  // namespace gemv2
}  // namespace ns3

#endif /* GEMV2_BUILDING_H */
