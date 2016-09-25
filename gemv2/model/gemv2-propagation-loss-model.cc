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

#include <ns3/mobility-model.h>

#include "gemv2-environment.h"

/*
 * Definition of default values used in the attributes and the default
 * constructor. Most of them are directly based on the matlab implementation
 * of GEMV^2.
 *
 * We put them in local scope to limit access to the current module.
 */
namespace
{

// Frequency - default is 5.9 GHz
constexpr double DEFAULT_FREQUENCY = 5.9e9;

// Antenna polarization
constexpr ns3::gemv2::AntennaPolarization DEFAULT_ANTENNA_POLARIZATION =
    ns3::gemv2::ANTENNA_POLARIZATION_VERTICAL;

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



/*
 * Some small helper functions
 */

//! Remove @a v.first and @a v.second from the provided list.
void
RemoveVehicles (ns3::gemv2::Environment::VehicleList& l,
		const std::pair<ns3::Ptr<ns3::gemv2::Vehicle>,
			        ns3::Ptr<ns3::gemv2::Vehicle>>& v)
{
  if (v.first)
    {
      l.erase (std::remove (l.begin (), l.end (), v.first), l.end ());
    }

  if (v.second)
    {
      l.erase (std::remove (l.begin (), l.end (), v.second), l.end ());
    }
}


}  // namespace


/*
 * And now the implementation of the propagation loss model.
 */

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
      .AddAttribute ("AntennaPolarization",
		     "Polarization of the antennas (vertical or horizontal)",
		     EnumValue (DEFAULT_ANTENNA_POLARIZATION),
		     MakeEnumAccessor (&Gemv2PropagationLossModel::m_antennaPolarization),
		     MakeEnumChecker (
			 gemv2::ANTENNA_POLARIZATION_VERTICAL, "vertical",
			 gemv2::ANTENNA_POLARIZATION_HORIZONTAL, "horizontal"))
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
       ;
  return tid;
}

Gemv2PropagationLossModel::Gemv2PropagationLossModel ()
  : m_environment (gemv2::Environment::GetGlobal ()),
    m_frequency (DEFAULT_FREQUENCY),
    m_antennaPolarization (DEFAULT_ANTENNA_POLARIZATION),
    m_maxLOSCommRange (DEFAULT_MAX_LOS_COMM_RANGE),
    m_maxNLOSvCommRange (DEFAULT_MAX_NLOSV_COMM_RANGE),
    m_maxNLOSbCommRange (DEFAULT_MAX_NLOSB_COMM_RANGE),
    m_modelNLOSv (DEFAULT_NLOSV_MODEL),
    m_lossPerVehicleNLOSvSimple (DEFAULT_LOSS_PER_VEHICLE_NLOSV_SIMPLE),
    m_modelNLOSb (DEFAULT_NLOSB_MODEL)
{
  NS_LOG_FUNCTION (this);
}

Gemv2PropagationLossModel::~Gemv2PropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

void
Gemv2PropagationLossModel::SetEnviroment (Ptr<gemv2::Environment> environment)
{
  NS_LOG_FUNCTION (this);
  m_environment = environment;
}

double
Gemv2PropagationLossModel::CalculateSmallScaleVariations (
    const gemv2::Point2d& a, const gemv2::Point2d& b) const
{
  // TODO: implement this
  return 0;
}

double
Gemv2PropagationLossModel::CalculateOutOfRangeNoise (
    double txPower, double distance, gemv2::LinkType linkType) const
{
  NS_LOG_FUNCTION (this << txPower << distance << static_cast<int> (linkType));
  /*
   * TODO: check if we can model this better
   *
   * It might be interesting to return some randomized noise based
   * on the distance to model the long distance interference of multiple
   * ongoing transmissions.
   */
  return -std::numeric_limits<double>::max ();
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
  NS_ASSERT (a);
  NS_ASSERT (b);

  // Get positions
  auto positions = std::make_pair (a->GetPosition (), b->GetPosition ());
  double distance = CalculateDistance (positions.first, positions.second);

  if (distance > m_maxLOSCommRange)
    {
      NS_LOG_LOGIC ("Nodes are out of range.");
      return CalculateOutOfRangeNoise (
	  txPowerDbm, distance, gemv2::LinkType::UNKNOWN);
    }

  // Get involved vehicles (if set and available)
  auto involvedVehicles = std::make_pair (a->GetObject<gemv2::Vehicle> (),
					  b->GetObject<gemv2::Vehicle> ());

  // Make line segment between points
  gemv2::LineSegment2d lineOfSight (gemv2::MakePoint2d (positions.first),
				    gemv2::MakePoint2d (positions.second));

  // Lets start without attenuation
  double attenuationDbm = 0;

  // communication range based on link type
  double effectiveComRange = -1;
  gemv2::LinkType linkType = gemv2::LinkType::UNKNOWN;

  // vehicles in LOS - will only be filled for NLOSv links
  gemv2::Environment::VehicleList vehiclesInLos;

  // First we check for obstructing buildings
  if (m_environment->IntersectsBuildings (lineOfSight))
    {
      NS_LOG_LOGIC ("LOS intersects with buildings -> link type: NLOSb");
      if (distance > m_maxNLOSbCommRange)
	{
	  NS_LOG_LOGIC ("NLOSb link out of range: " << distance);
	  return CalculateOutOfRangeNoise (
	      txPowerDbm, distance, gemv2::LinkType::NLOSb);
	}

      effectiveComRange = m_maxNLOSbCommRange;
      linkType = gemv2::LinkType::NLOSb;
    }
  else if (m_environment->IntersectsFoliage (lineOfSight))
    {
      NS_LOG_LOGIC ("LOS intersects with foliage -> link type: NLOSf");
      if (distance > m_maxNLOSbCommRange)
	{
	  NS_LOG_LOGIC ("NLOSf link out of range: " << distance);
	  return CalculateOutOfRangeNoise (
	      txPowerDbm, distance, gemv2::LinkType::NLOSf);
	}

      effectiveComRange = m_maxNLOSbCommRange;
      linkType = gemv2::LinkType::NLOSf;
    }
  else
    {
      // No buildings or foliage, check for obstructing vehicles
      m_environment->Intersect (lineOfSight, vehiclesInLos);

      // remove involved vehicles from list
      RemoveVehicles (vehiclesInLos, involvedVehicles);

      // check if there are other vehicles in the LOS
      if (!vehiclesInLos.empty ())
	{
	  NS_LOG_LOGIC (""
	      "LOS intersects with other vehicles -> link type: NLOSv");

	  if (distance > m_maxNLOSvCommRange)
	    {
	      NS_LOG_LOGIC ("NLOSv link out of range: " << distance);
	      return CalculateOutOfRangeNoise (
		  txPowerDbm, distance, gemv2::LinkType::NLOSv);
	    }

	  effectiveComRange = m_maxNLOSvCommRange;
	  linkType = gemv2::LinkType::NLOSv;
	}
      else
	{
	  NS_LOG_LOGIC ("LOS is clear -> link type: LOS");

	  /*
	   * Note: No need to check the distance here since we checked
	   *       this in the beginning to avoid searching the environment
	   *       for links beyond the maximum communication range.
	   */

	  effectiveComRange = m_maxLOSCommRange;
	  linkType = gemv2::LinkType::LOS;
	}
    }

  NS_ASSERT_MSG (effectiveComRange > 0,
		 "Effective communication range must be larger than 0");

  // Find all objects in joint communication ellipse
  gemv2::Environment::ObjectCollection jointObjects;
  m_environment->FindAllInEllipse (
      lineOfSight.first, lineOfSight.second, effectiveComRange, jointObjects);

  // remove sender and receiver from list
  RemoveVehicles (jointObjects.vehicles, involvedVehicles);

  // TODO: calculate large scale variations based on link type
  (void) linkType;

  // add small scale variations
  attenuationDbm += CalculateSmallScaleVariations (lineOfSight.first, lineOfSight.second);

  return txPowerDbm - attenuationDbm;
}

int64_t
Gemv2PropagationLossModel::DoAssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  return 0;
}

}
  // namespace ns3
