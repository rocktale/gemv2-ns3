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
#ifndef GEMV2_FOLIAGE_H
#define GEMV2_FOLIAGE_H

#include <ns3/simple-ref-count.h>
#include <ns3/gemv2-geometry.h>

namespace ns3 {
namespace gemv2 {

/*!
 * @brief A single foliage (tree, bush, ...) within the GEMV^2 environment.
 */
class Foliage : public SimpleRefCount<Foliage>
{
public:
  /*!
   * @brief Create foliage from the provided shape.
   */
  explicit Foliage (Polygon2d const& shape);

  /*!
   * @brief Get the shape of the foliage
   * @return Shape of the foliage
   */
  Polygon2d const&
  GetShape () const;

  /*!
   * @brief Get the bounding box of the foliage
   * @return Bounding box of the foliage
   */
  Box2d const&
  GetBoundingBox () const;

private:

  //! Shape of the foliage
  Polygon2d m_shape;

  //! Bounding box of the foliage
  Box2d m_boundingBox;
};

}  // namespace gemv2
}  // namespace ns3

#endif /* GEMV2_FOLIAGE_H */
