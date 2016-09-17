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
#ifndef GEMV2_PROPAGATION_LOSS_MODEL_H
#define GEMV2_PROPAGATION_LOSS_MODEL_H

#include <ns3/propagation-loss-model.h>

namespace ns3 {

class Gemv2PropagationLossModel : public PropagationLossModel
{
public:

  /*!
   * @brief Get the type id of this  object.
   * @return Object type id
   */
  static TypeId
  GetTypeId (void);

  /*!
   * @brief Default constructor.
   */
  Gemv2PropagationLossModel ();

  /*!
   * @brief Destructor.
   */
  ~Gemv2PropagationLossModel ();

private:

  /*
   * Disable copy construction and assigment to avoid misuse.
   */
  Gemv2PropagationLossModel (const Gemv2PropagationLossModel&) = delete;
  void operator= (const Gemv2PropagationLossModel &) = delete;

  /*
   * PropagationLossModel methods
   */

  double
  DoCalcRxPower (double txPowerDbm, Ptr<MobilityModel> a,
		 Ptr<MobilityModel> b) const override;

  int64_t
  DoAssignStreams (int64_t stream) override;

};

}  // namespace ns3

#endif /* GEMV2_PROPAGATION_LOSS_MODEL_H */
