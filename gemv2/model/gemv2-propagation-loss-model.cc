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
#include "gemv2-propagation-loss-model.h"

#include <limits>

#include <ns3/assert.h>
#include <ns3/log.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("Gemv2PropagationLossModel");
NS_OBJECT_ENSURE_REGISTERED(Gemv2PropagationLossModel);

TypeId
Gemv2PropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Gemv2PropagationLossModel")
      .SetParent<PropagationLossModel> ()
      .SetGroupName ("Propagation")
      .AddConstructor<Gemv2PropagationLossModel> ()
      ;
  return tid;
}

Gemv2PropagationLossModel::Gemv2PropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

Gemv2PropagationLossModel::~Gemv2PropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

/*
 * PropagationLossModel methods
 */

double
Gemv2PropagationLossModel::DoCalcRxPower (double txPowerDbm,
					  Ptr<MobilityModel> a,
					  Ptr<MobilityModel> b) const
{
  NS_LOG_FUNCTION (this << txPowerDbm);

  // TODO: calculate real value
  return -std::numeric_limits<double>::max ();
}

int64_t
Gemv2PropagationLossModel::DoAssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  return 0;
}

}
  // namespace ns3
