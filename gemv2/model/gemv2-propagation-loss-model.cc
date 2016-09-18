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

#include <ns3/double.h>
#include <ns3/enum.h>

// Default value constants are in local scope only
namespace
{

// Frequency - default is 5.9 GHz
constexpr double DEFAULT_FREQUENCY = 5.9e9;

// Communication ranges
constexpr double DEFAULT_MAX_LOS_COMM_RANGE = 1000.0;
constexpr double DEFAULT_MAX_NLOSV_COMM_RANGE = 500.0;
constexpr double DEFAULT_MAX_NLOSB_COMM_RANGE = 300.0;

// Model for NLOSv links
constexpr ns3::gemv2::NLOSvModelType DEFAULT_NLOSV_MODEL =
    ns3::gemv2::NLOSV_MODEL_SIMPLE;

// NLOSv vehicle loss for the simple model
constexpr ns3::gemv2::MinMedMaxDoubleValue
  DEFAULT_LOSS_PER_VEHICLE_NLOSV_SIMPLE = { 2.0, 6.0, 10.0 };

// Model for NLOSb links
constexpr ns3::gemv2::NLOSbModelType DEFAULT_NLOSB_MODEL =
    ns3::gemv2::NLOSB_MODEL_LOG_DISTANCE;

// Material parameters for reflections
constexpr double DEFAULT_PERMITTIVITY_BUILDINGS = 4.5;  // concrete
constexpr double DEFAULT_PERMITTIVITY_VEHICLES = 6.0; 	// mix of metal, glass, ...

}  // namespace

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
      .AddAttribute ("Frequency",
		     "The carrier frequency at which propagation occurs [Hz].",
                     DoubleValue (DEFAULT_FREQUENCY),
                     MakeDoubleAccessor (&Gemv2PropagationLossModel::m_frequency),
                     MakeDoubleChecker<double> ())
      .AddAttribute ("MaxLOSCommunicationRange",
		     "Maximum LOS communication range [m].",
		     DoubleValue (DEFAULT_MAX_LOS_COMM_RANGE),
		     MakeDoubleAccessor (&Gemv2PropagationLossModel::m_maxLOSCommRange),
		     MakeDoubleChecker<double> ())
      .AddAttribute ("MaxNLOSvCommunicationRange",
		     "Maximum NLOSv communication range [m].",
		     DoubleValue (DEFAULT_MAX_NLOSV_COMM_RANGE),
		     MakeDoubleAccessor (&Gemv2PropagationLossModel::m_maxNLOSvCommRange),
		     MakeDoubleChecker<double> ())
      .AddAttribute ("MaxNLOSbCommunicationRange",
		     "Maximum NLOSb communication range [m].",
		     DoubleValue (DEFAULT_MAX_NLOSB_COMM_RANGE),
		     MakeDoubleAccessor (&Gemv2PropagationLossModel::m_maxNLOSbCommRange),
		     MakeDoubleChecker<double> ())
      .AddAttribute ("NLOSvModel",
		     "Model used for NLOSv links",
		     EnumValue (DEFAULT_NLOSV_MODEL),
		     MakeEnumAccessor (&Gemv2PropagationLossModel::m_modelNLOSv),
		     MakeEnumChecker (
			 gemv2::NLOSV_MODEL_SIMPLE, "simple",
			 gemv2::NLOSV_MODEL_BULLINGTON_KNIFE_EDGE, "bullington",
			 gemv2::NLOSV_MODEL_ITU_R_MULTIPLE_KNIFE_EDGE, "multiple-knife-edge"))
      .AddAttribute ("NLOSbModel",
		     "Model used for NLOSb links",
		     EnumValue (DEFAULT_NLOSB_MODEL),
		     MakeEnumAccessor (&Gemv2PropagationLossModel::m_modelNLOSb),
		     MakeEnumChecker (
			 gemv2::NLOSB_MODEL_LOG_DISTANCE, "log-distance",
			 gemv2::NLOSB_MODEL_REFLECTION_DIFFRACTION, "reflection-diffraction"))
      .AddAttribute ("RelativePermittivityBuildings",
		     "Relative permittivity for buildings.",
		     DoubleValue (DEFAULT_PERMITTIVITY_BUILDINGS),
		     MakeDoubleAccessor (&Gemv2PropagationLossModel::m_permittivityBuildings),
		     MakeDoubleChecker<double> ())
      .AddAttribute ("RelativePermittivityVehicles",
		     "Relative permittivity of vehicles.",
		     DoubleValue (DEFAULT_PERMITTIVITY_VEHICLES),
		     MakeDoubleAccessor (&Gemv2PropagationLossModel::m_permittivityVehicles),
		     MakeDoubleChecker<double> ())
       ;
  return tid;
}

Gemv2PropagationLossModel::Gemv2PropagationLossModel ()
  : m_frequency (DEFAULT_FREQUENCY),
    m_maxLOSCommRange (DEFAULT_MAX_LOS_COMM_RANGE),
    m_maxNLOSvCommRange (DEFAULT_MAX_NLOSV_COMM_RANGE),
    m_maxNLOSbCommRange (DEFAULT_MAX_NLOSB_COMM_RANGE),
    m_modelNLOSv (DEFAULT_NLOSV_MODEL),
    m_lossPerVehicleNLOSvSimple (DEFAULT_LOSS_PER_VEHICLE_NLOSV_SIMPLE),
    m_modelNLOSb (DEFAULT_NLOSB_MODEL),
    m_permittivityBuildings (DEFAULT_PERMITTIVITY_BUILDINGS),
    m_permittivityVehicles (DEFAULT_PERMITTIVITY_VEHICLES)
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
