/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 Karsten Roscher
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
#ifndef GEMV2_VEHICLE_ADAPTER_H
#define GEMV2_VEHICLE_ADAPTER_H

#include <ns3/object.h>
#include <ns3/ptr.h>

#include "gemv2-vehicle.h"

namespace ns3 {

/*!
 * @brief This is an object linking an ns-3 node to a GEMV^2 vehicle.
 */
class Gemv2VehicleAdapter : public Object {
public:
  static TypeId GetTypeId (void);

  Gemv2VehicleAdapter ();

  /*!
   * @brief Get the assigned vehicle if any
   * @return	Pointer to the vehicle, null if not assigned
   */
  Ptr<gemv2::Vehicle>
  GetVehicle () const;

  /*!
   * @brief Set the assigned vehicle, null to disable
   * @param vehicle 	Vehicle to assign
   */
  void
  SetVehicle (Ptr<gemv2::Vehicle> vehicle);

private:

  //! Assigned GEMV^2 vehicle object
  Ptr<gemv2::Vehicle> m_vehicle;
};


} // namespace ns3

#endif /* GEMV2_VEHICLE_ADAPTER_H */
