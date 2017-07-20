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
#include "gemv2-vehicle-adapter.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (Gemv2VehicleAdapter);

TypeId
Gemv2VehicleAdapter::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Gemv2VehicleAdapter")
    .SetParent<Object> ()
    .AddConstructor<Gemv2VehicleAdapter> ();
  return tid;
}

Gemv2VehicleAdapter::Gemv2VehicleAdapter () = default;

Ptr<gemv2::Vehicle>
Gemv2VehicleAdapter::GetVehicle () const
{
  return m_vehicle;
}

void
Gemv2VehicleAdapter::SetVehicle (Ptr<gemv2::Vehicle> vehicle)
{
  m_vehicle = vehicle;
}

}  // namespace ns3
