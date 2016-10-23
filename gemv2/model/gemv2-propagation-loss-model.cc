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
#include <boost/math/constants/constants.hpp>

#include <ns3/assert.h>
#include <ns3/log.h>
#include <ns3/double.h>
#include <ns3/enum.h>

#include <ns3/mobility-model.h>
#include <ns3/gemv2-models.h>


/*
 * Definition of default values used in the attributes and the default
 * constructor. Most of them are directly based on the matlab implementation
 * of GEMV^2.
 *
 * We put them in local scope to limit access to the current module.
 */
namespace
{
// Factor to convert square meters to square kilometers
constexpr double SQR_METERS_TO_SQR_KILOMETERS = 1e-6;

// Frequency - default is 5.9 GHz
constexpr double DEFAULT_FREQUENCY = 5.9e9;

// Antenna polarization
constexpr ns3::gemv2::AntennaPolarization DEFAULT_ANTENNA_POLARIZATION =
    ns3::gemv2::ANTENNA_POLARIZATION_HORIZONTAL;

// Permittivity for ground reflections (from the GEMV^2 measurements in Porto)
constexpr double DEFAULT_GROUND_PERMITTIVITY = 1.003;

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

// Maximum density values - currently just a guess
constexpr double DEFAULT_MAX_VEHICLE_DENSITY = 500.0; // 500 vehicles/km^2
constexpr double DEFAULT_MAX_OBJECT_DENSITY = 0.8;    // 80% covered with objects

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
       .AddAttribute ("GroundPermittivity",
		      "Relative permittivity for ground reflections",
		      DoubleValue (DEFAULT_GROUND_PERMITTIVITY),
		      MakeDoubleAccessor (&Gemv2PropagationLossModel::m_groundPermittivity),
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
       ;
  return tid;
}

Gemv2PropagationLossModel::Gemv2PropagationLossModel ()
  : m_environment (gemv2::Environment::GetGlobal ()),
    m_frequency (DEFAULT_FREQUENCY),
    m_antennaPolarization (DEFAULT_ANTENNA_POLARIZATION),
    m_groundPermittivity (DEFAULT_GROUND_PERMITTIVITY),
    m_maxLOSCommRange (DEFAULT_MAX_LOS_COMM_RANGE),
    m_maxNLOSvCommRange (DEFAULT_MAX_NLOSV_COMM_RANGE),
    m_maxNLOSbCommRange (DEFAULT_MAX_NLOSB_COMM_RANGE),
    m_modelNLOSv (DEFAULT_NLOSV_MODEL),
    m_lossPerVehicleNLOSvSimple (DEFAULT_LOSS_PER_VEHICLE_NLOSV_SIMPLE),
    m_modelNLOSb (DEFAULT_NLOSB_MODEL),
    m_maxVehicleDensity (DEFAULT_MAX_VEHICLE_DENSITY),
    m_maxObjectDensity (DEFAULT_MAX_OBJECT_DENSITY),
    m_normalRand (CreateObject<NormalRandomVariable> ())
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
    double distance, double comRange,
    const gemv2::Environment::ObjectCollection& objects,
    gemv2::LinkType linkType) const
{
  NS_LOG_FUNCTION (this);

  // calculate area of the ellipse
  double a = comRange / 2.0;
  double b = std::sqrt ((a*a) - (distance*distance/4.0));
  double ellipseArea = a * b * boost::math::constants::pi<double>();

  NS_LOG_LOGIC ("Ellipse area: " << ellipseArea << " m^2");

  double objectArea = 0;

  // collect area covered by objects
  for (auto const& b : objects.buildings)
    {
      objectArea += b->GetArea ();
    }
  for (auto const& f : objects.foliage)
    {
      objectArea += f->GetArea ();
    }

  NS_LOG_LOGIC ("Area covered by objects: " << objectArea << " m^2");

  double weight =
      std::min(
	  1.0,
	  std::sqrt(
	      objects.vehicles.size () /
	      (m_maxVehicleDensity * ellipseArea * SQR_METERS_TO_SQR_KILOMETERS)
	    )
	) +
      std::min(
	  1.0,
	  std::sqrt(
	      objectArea /
	      (m_maxObjectDensity * ellipseArea)
	    )
      );

  NS_LOG_LOGIC ("Occupancy weight: " << weight);

  auto it = m_v2vPropagation.smallScaleFading.find (linkType);
  if (it != m_v2vPropagation.smallScaleFading.end ())
    {
      double sigma =
          it->second.first +
          0.5 * weight * (it->second.second - it->second.first);

      NS_ASSERT (m_normalRand);
      double attenuation = m_normalRand->GetValue (0, sigma);

      NS_LOG_LOGIC ("sigma=" << sigma << ", attenuation=" << attenuation);
      return attenuation;
    }
  else
    {
      NS_ASSERT_MSG (false, "Unknown link type");
      return 0;
    }
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

double
Gemv2PropagationLossModel::CalculateSimpleNlosvLoss (
    double distance, std::size_t vehiclesInLos) const
{
  NS_LOG_FUNCTION (this << distance << vehiclesInLos);

  NS_ASSERT_MSG (vehiclesInLos > 0,
		 "There has to be at least one vehicle in the LOS for a NLOSv link");

  double freeSpaceLoss = gemv2::FreeSpaceLoss (distance, m_frequency);

  /*
   * This implementation follows the matlab code where only cases
   * with 1, 2 or more than 2 cars
   */
  if (vehiclesInLos == 1)
    {
      return freeSpaceLoss + std::get<0>(m_lossPerVehicleNLOSvSimple);
    }
  else if (vehiclesInLos == 2)
    {
      return freeSpaceLoss + std::get<1>(m_lossPerVehicleNLOSvSimple);
    }
  else
    {
      return freeSpaceLoss + std::get<2>(m_lossPerVehicleNLOSvSimple);
    }
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

  // calculate 2d distance between both peers
  double distance2d =
      std::sqrt(
	  std::pow(positions.first.x - positions.second.x, 2) +
	  std::pow(positions.first.y - positions.second.y, 2)
      );

  if (distance2d > m_maxLOSCommRange)
    {
      NS_LOG_LOGIC ("Nodes are out of range.");
      return CalculateOutOfRangeNoise (
	  txPowerDbm, distance2d, gemv2::LinkType::UNKNOWN);
    }

  // Get involved vehicles (if set and available)
  auto involvedVehicles = std::make_pair (a->GetObject<gemv2::Vehicle> (),
					  b->GetObject<gemv2::Vehicle> ());

  // Make line segment between points
  gemv2::LineSegment2d lineOfSight (gemv2::MakePoint2d (positions.first),
				    gemv2::MakePoint2d (positions.second));

  // Lets start without attenuation
  double rxPowerDbm = txPowerDbm;

  // communication range based on link type
  double effectiveComRange = -1;
  gemv2::LinkType linkType = gemv2::LinkType::UNKNOWN;

  // vehicles in LOS - will only be filled for NLOSv links
  gemv2::Environment::VehicleList vehiclesInLos;

  // First we check for obstructing buildings
  if (m_environment->IntersectsBuildings (lineOfSight))
    {
      NS_LOG_LOGIC ("LOS intersects with buildings -> link type: NLOSb");
      if (distance2d > m_maxNLOSbCommRange)
	{
	  NS_LOG_LOGIC ("NLOSb link out of range: " << distance2d);
	  return CalculateOutOfRangeNoise (
	      txPowerDbm, distance2d, gemv2::LinkType::NLOSb);
	}

      effectiveComRange = m_maxNLOSbCommRange;
      linkType = gemv2::LinkType::NLOSb;
    }
  else if (m_environment->IntersectsFoliage (lineOfSight))
    {
      NS_LOG_LOGIC ("LOS intersects with foliage -> link type: NLOSf");
      if (distance2d > m_maxNLOSbCommRange)
	{
	  NS_LOG_LOGIC ("NLOSf link out of range: " << distance2d);
	  return CalculateOutOfRangeNoise (
	      txPowerDbm, distance2d, gemv2::LinkType::NLOSf);
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

	  if (distance2d > m_maxNLOSvCommRange)
	    {
	      NS_LOG_LOGIC ("NLOSv link out of range: " << distance2d);
	      return CalculateOutOfRangeNoise (
		  txPowerDbm, distance2d, gemv2::LinkType::NLOSv);
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

  double txGainDbi = 0.0;	// TODO: get tx gain from antenna model
  double rxGainDbi = 0.0;	// TODO: get rx gain from antenna model

  // TODO: calculate large scale variations based on link type
  switch (linkType)
  {
    case gemv2::LinkType::LOS:
      {
	/*
	 * LOS links use the two-ray-ground loss model for the large
	 * scale propagation loss.
	 */
	double eTot = gemv2::TwoRayGroundLoss(
	    distance2d,
	    positions.first.z, positions.second.z,
	    m_frequency, txPowerDbm, txGainDbi,
	    m_antennaPolarization,
	    m_groundPermittivity);

	rxPowerDbm = gemv2::EfieldToPowerDbm(
	    eTot, rxGainDbi,
	    m_frequency);
	NS_LOG_LOGIC ("Two-ray-ground loss: " << (txPowerDbm - rxPowerDbm));
	break;
      }
    case gemv2::LinkType::NLOSv:
      switch (m_modelNLOSv)
      {
	case gemv2::NLOSV_MODEL_SIMPLE:
	  rxPowerDbm = txPowerDbm + txGainDbi + rxGainDbi -
	      CalculateSimpleNlosvLoss (
		  CalculateDistance (positions.first, positions.second),
		  vehiclesInLos.size ());
	  NS_LOG_LOGIC ("Simple NLOSv model loss: " << (txPowerDbm - rxPowerDbm));
	  break;
	case gemv2::NLOSV_MODEL_ITU_R_MULTIPLE_KNIFE_EDGE:
	case gemv2::NLOSV_MODEL_BULLINGTON_KNIFE_EDGE:
	  NS_ASSERT_MSG (false, "NLOSv model not implemented (yet)");
	  break;
	default:
	  NS_ASSERT_MSG (false, "Unknown NLOSv model");
	  break;
      }
      break;

    case gemv2::LinkType::NLOSb:
      break;

    case gemv2::LinkType::NLOSf:
      break;

    default:
      NS_ASSERT_MSG (false, "Link type must not be undefined at this point");
  }


  // add small scale variations based on link type
  rxPowerDbm -= CalculateSmallScaleVariations (
      distance2d, effectiveComRange, jointObjects, linkType);

  return rxPowerDbm;
}

int64_t
Gemv2PropagationLossModel::DoAssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  if (m_normalRand)
    {
      m_normalRand->SetStream(stream++);
    }
  return 1;
}

}
  // namespace ns3
