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

#include <ns3/ptr.h>
#include <ns3/propagation-loss-model.h>

#include "gemv2-types.h"
#include "gemv2-propagation-parameters.h"
#include "gemv2-geometry.h"
#include "gemv2-environment.h"

namespace ns3 {

/*!
 * @brief Propagation loss model based on GEMV^2.
 *
 * This is the main class for the propagation loss model. It provides
 * most of the parameters available for GEMV^2 and calculates the channel
 * loss for a link.
 *
 * More information can be found here: vehicle2x.net
 */
class Gemv2PropagationLossModel : public PropagationLossModel
{
public:
  /*
   * Some internal types to make function calls more readable
   */
  using VehiclePair = std::pair<Ptr<gemv2::Vehicle>, Ptr<gemv2::Vehicle>>;

  /*
   * Methods provided by the model
   */

  /*!
   * @brief Get the type id of this  object.
   * @return Object type id
   */
  static TypeId
  GetTypeId (void);

  /*!
   * @brief Default constructor.
   *
   * Initializes all parameters with the default values and set
   * the used environment to the global instance.
   */
  Gemv2PropagationLossModel ();

  /*!
   * @brief Destructor.
   */
  ~Gemv2PropagationLossModel ();

  /*
   * Disable copy construction and assignment to avoid misuse.
   */
  Gemv2PropagationLossModel (const Gemv2PropagationLossModel&) = delete;
  void operator= (const Gemv2PropagationLossModel &) = delete;

  /*!
   * @brief Set a custom environment instance to use.
   * @param environment		Environment used for calculations
   */
  void
  SetEnviroment (Ptr<gemv2::Environment> environment);

  /*!
   * @brief Enable/disable deterministic mode of the model
   *
   * If enabled, all random components like small scale variations will be
   * disabled (set to 0). Thus, the model will behave deterministically yet
   * less realistic.
   *
   * @param determinstic Set to true to disable the random components of the model
   */
  void
  ForceDeterminstic (bool determinstic);

private:

  /*!
   * @brief Calculate the small scale variations for a link.
   * @param distance2d	Distance between sender and receiver (2d)
   * @param comRange	Communication range
   * @param objects	Objects in the ellipse around sender and receiver
   * @param sigmaMin	Minimum value for sigma (depends on the link type)
   * @param sigmaMax	Maximum value for sigma (depends on the link type)
   * @return Small scale variations in dBm
   */
  double
  CalculateSmallScaleVariations (
      double distance2d, double comRange,
      const gemv2::Environment::ObjectCollection& objects,
      double sigmaMin, double sigmaMax) const;


  /*!
   * @brief Calculate some noise for out of range links
   * @param txPower	Transmit power [dBm]
   * @param distance	Distance between sender and receiver [m]
   * @param linkType 	Type of the link
   * @return Power at receiver [dBm]
   */
  double
  CalculateOutOfRangeNoise (double txPower,
			    double distance,
			    gemv2::LinkType linkType) const;

  /*!
   * @brief Calculate the loss for NLOSv links with the simple model.
   *
   * This model uses free-space propagation together with additional
   * attenuation based on the number of vehicles in the LOS path.
   *
   * @param distance		Distance between sender and receiver [m]
   * @param vehiclesInLos	Number of vehicles in the LOS path
   * @return Path loss (dBm)
   */
  double
  CalculateSimpleNlosvLoss (double distance,
			    std::size_t vehiclesInLos) const;

  /*!
   * @brief Get the environment objects in the communication ellipse.
   * @param lineOfSight		Line of sight between sender and receiver
   * @param comRange		Communication range (meters)
   * @param involvedVehicles	Sender/receiver vehicles
   * @return Collection of found objects
   */
  gemv2::Environment::ObjectCollection
  GetObjectsInComEllipse (const gemv2::LineSegment2d& lineOfSight,
			  double comRange,
			  const VehiclePair& involvedVehicles) const;

  /*
   * PropagationLossModel methods
   */


  double
  DoCalcRxPower (double txPowerDbm, Ptr<MobilityModel> a,
		 Ptr<MobilityModel> b) const override;

  int64_t
  DoAssignStreams (int64_t stream) override;



  /*
   * Internal methods for the different link types
   */

  /*!
   * @brief Check if the link is within range for calculation
   * @param txPowerDbm	Transmission power in dBm
   * @param distance2d	Distance between sender and receiver in m
   * @return True if in range, false otherwise
   */
  bool
  IsLinkInRange (double txPowerDbm, double distance) const;

  double
  CalcNlosbRxPower (double txPowerDbm, double distance,
		    const gemv2::LineSegment2d& lineOfSight,
		    const VehiclePair& involvedVehicles,
		    double txGainDbi, double rxGainDbi) const;

  double
  CalcNlosfRxPower (double txPowerDbm, double distance) const;

  double
  CalcNlosvRxPower (double txPowerDbm, double distance,
		    const gemv2::LineSegment2d& lineOfSight,
		    const gemv2::Environment::VehicleList& vehiclesInLos,
		    const VehiclePair& involvedVehicles,
		    double txGainDbi, double rxGainDbi) const;

  double
  CalcLosRxPower (double txPowerDbm, double distance,
		  const gemv2::LineSegment2d& lineOfSight,
		  const Vector& txPos, const Vector& rxPos,
		  const VehiclePair& involvedVehicles,
		  double txGainDbi, double rxGainDbi) const;

  /*
   * Internal data
   */

  //! Pointer to the environment description
  Ptr<gemv2::Environment> m_environment;

  /*
   * Parameters of the model
   */

  //! Frequency of the signal
  double m_frequency;

  //! Polarization of the antennas
  gemv2::AntennaPolarization m_antennaPolarization;

  //! Relative permittivity for ground reflections
  double m_groundPermittivity;

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

  // Maximum values for vehicle and object density

  //! Maximum number of vehicles per km^2
  double m_maxVehicleDensity;

  //! Maximum area (km^2) occupied by objects per km^2
  double m_maxObjectDensity;

  //! Disable all random components of the propagation model
  bool m_forceDeterminstic;

  /*
   * Random variables
   */

  Ptr<NormalRandomVariable> m_normalRand;
};

}  // namespace ns3

#endif /* GEMV2_PROPAGATION_LOSS_MODEL_H */
