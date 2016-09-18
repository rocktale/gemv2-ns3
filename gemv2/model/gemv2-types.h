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
#ifndef GEMV2_TYPES_H
#define GEMV2_TYPES_H

#include <tuple>

namespace ns3 {
namespace gemv2 {

/*!
 * @brief Different models for links obstructed by vehicles.
 *
 * @note We cannot use a class enum here because it is not supported
 *       by the EnumValue provided with ns3.
 */
enum NLOSvModelType
{
  //! Simple model with attenuation based on the number of obstruction vehicles
  NLOSV_MODEL_SIMPLE = 0,
  //! Bullington method for knife-edge diffraction
  NLOSV_MODEL_BULLINGTON_KNIFE_EDGE,
  //! Multiple knife-edge diffraction based on ITU-R
  NLOSV_MODEL_ITU_R_MULTIPLE_KNIFE_EDGE
};

/*!
 * @brief Different models for links obstructed by buildings
 * @note We cannot use a class enum here because it is not supported
 *       by the EnumValue provided with ns3.
 */
enum NLOSbModelType
{
  //! Simple model using log-distance path loss
  NLOSB_MODEL_LOG_DISTANCE = 0,
  //! Model with reflections and diffractions
  NLOSB_MODEL_REFLECTION_DIFFRACTION
};

//! A tuple consisting of a min/med/max value.
typedef std::tuple<double, double, double> MinMedMaxDoubleValue;

}  // namespace gemv2 {
}  // namespace ns3

#endif /* GEMV2_TYPES_H */
