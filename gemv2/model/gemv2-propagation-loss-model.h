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

#include "gemv2-types.h"
#include "gemv2-propagation-parameters.h"

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


  /*
   * Parameters of the model
   */

  //! Frequency of the signal
  double m_frequency;

  // Communication ranges

  //! Maximum communication range for LOS [m]
  double m_maxLOSCommRange;
  //! Maximum communication range for NLOSv [m]
  double m_maxNLOSvCommRange;
  //! Maximum communication range for NLOSb [m]
  double m_maxNLOSbCommRange;

  // Propagation parameters

  //! V2V propagation parameters
  gemv2::PropagationParameters m_v2vPropagation;

  //! Model for the NLOSv links
  gemv2::NLOSvModelType m_modelNLOSv;

  //! Loss due to obstructing vehicles (simple model) [db]
  gemv2::MinMedMaxDoubleValue m_lossPerVehicleNLOSvSimple;

  //! Model for NLOSb links
  gemv2::NLOSbModelType m_modelNLOSb;

  // Reflection parameters

  //! Relative permittivity of buildings
  double m_permittivityBuildings;

  //! Relative permittivity of vehicles (approximation)
  double m_permittivityVehicles;
};

}  // namespace ns3

#endif /* GEMV2_PROPAGATION_LOSS_MODEL_H */
