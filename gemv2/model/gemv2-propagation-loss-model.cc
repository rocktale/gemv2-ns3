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
namespace {
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
  DEFAULT_LOSS_PER_VEHICLE_NLOSV_SIMPLE{ 2.0, 6.0, 10.0 };

// Model for NLOSb links
constexpr ns3::gemv2::NLOSbModelType DEFAULT_NLOSB_MODEL =
    ns3::gemv2::NLOSB_MODEL_LOG_DISTANCE;

// Maximum density values - currently just a guess
constexpr double DEFAULT_MAX_VEHICLE_DENSITY = 500.0; // 500 vehicles/km^2
constexpr double DEFAULT_MAX_OBJECT_DENSITY = 0.8;   // 80% covered with objects

/*
 * Some small helper functions
 */

//! Remove @a v.first and @a v.second from the provided list.
void
RemoveVehicles (
    ns3::gemv2::Environment::VehicleList& l,
    const ns3::Gemv2PropagationLossModel::VehiclePair& v)
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
  static TypeId tid =
      TypeId ("ns3::Gemv2PropagationLossModel").SetParent<PropagationLossModel> ().SetGroupName (
	  "Propagation").AddConstructor<Gemv2PropagationLossModel> ().AddAttribute (
	  "Frequency",
	  "The carrier frequency at which propagation occurs [Hz].",
	  DoubleValue (DEFAULT_FREQUENCY),
	  MakeDoubleAccessor (&Gemv2PropagationLossModel::m_frequency),
	  MakeDoubleChecker<double> ()).AddAttribute (
	  "AntennaPolarization",
	  "Polarization of the antennas (vertical or horizontal)",
	  EnumValue (DEFAULT_ANTENNA_POLARIZATION),
	  MakeEnumAccessor (&Gemv2PropagationLossModel::m_antennaPolarization),
	  MakeEnumChecker (gemv2::ANTENNA_POLARIZATION_VERTICAL, "vertical",
			   gemv2::ANTENNA_POLARIZATION_HORIZONTAL,
			   "horizontal")).AddAttribute (
	  "GroundPermittivity", "Relative permittivity for ground reflections",
	  DoubleValue (DEFAULT_GROUND_PERMITTIVITY),
	  MakeDoubleAccessor (&Gemv2PropagationLossModel::m_groundPermittivity),
	  MakeDoubleChecker<double> ()).AddAttribute (
	  "MaxLOSCommunicationRange", "Maximum LOS communication range [m].",
	  DoubleValue (DEFAULT_MAX_LOS_COMM_RANGE),
	  MakeDoubleAccessor (&Gemv2PropagationLossModel::m_maxLOSCommRange),
	  MakeDoubleChecker<double> ()).AddAttribute (
	  "MaxNLOSvCommunicationRange",
	  "Maximum NLOSv communication range [m].",
	  DoubleValue (DEFAULT_MAX_NLOSV_COMM_RANGE),
	  MakeDoubleAccessor (&Gemv2PropagationLossModel::m_maxNLOSvCommRange),
	  MakeDoubleChecker<double> ()).AddAttribute (
	  "MaxNLOSbCommunicationRange",
	  "Maximum NLOSb communication range [m].",
	  DoubleValue (DEFAULT_MAX_NLOSB_COMM_RANGE),
	  MakeDoubleAccessor (&Gemv2PropagationLossModel::m_maxNLOSbCommRange),
	  MakeDoubleChecker<double> ()).AddAttribute (
	  "NLOSvModel",
	  "Model used for NLOSv links",
	  EnumValue (DEFAULT_NLOSV_MODEL),
	  MakeEnumAccessor (&Gemv2PropagationLossModel::m_modelNLOSv),
	  MakeEnumChecker (gemv2::NLOSV_MODEL_SIMPLE, "simple",
			   gemv2::NLOSV_MODEL_BULLINGTON_KNIFE_EDGE,
			   "bullington",
			   gemv2::NLOSV_MODEL_ITU_R_MULTIPLE_KNIFE_EDGE,
			   "multiple-knife-edge")).AddAttribute (
	  "NLOSbModel",
	  "Model used for NLOSb links",
	  EnumValue (DEFAULT_NLOSB_MODEL),
	  MakeEnumAccessor (&Gemv2PropagationLossModel::m_modelNLOSb),
	  MakeEnumChecker (gemv2::NLOSB_MODEL_LOG_DISTANCE, "log-distance",
			   gemv2::NLOSB_MODEL_REFLECTION_DIFFRACTION,
			   "reflection-diffraction"));
  return tid;
}

Gemv2PropagationLossModel::Gemv2PropagationLossModel () :
    m_environment (gemv2::Environment::GetGlobal ()),
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
    m_forceDeterminstic (false),
    m_normalRand (CreateObject<NormalRandomVariable> ())
{
  NS_LOG_FUNCTION(this);
}

Gemv2PropagationLossModel::~Gemv2PropagationLossModel ()
{
  NS_LOG_FUNCTION(this);
}

void
Gemv2PropagationLossModel::SetEnviroment (Ptr<gemv2::Environment> environment)
{
  NS_LOG_FUNCTION(this);
  m_environment = environment;
}

void
Gemv2PropagationLossModel::ForceDeterminstic (bool determinstic)
{
  NS_LOG_FUNCTION (this << determinstic);
  m_forceDeterminstic = determinstic;
}

double
Gemv2PropagationLossModel::CalculateSmallScaleVariations (
    double distance2d, double comRange,
    const gemv2::Environment::ObjectCollection& objects,
    double sigmaMin, double sigmaMax) const
{
  NS_LOG_FUNCTION(this);

  if (m_forceDeterminstic)
    {
      // no need to calculate anything
      return 0;
    }

  // calculate area of the ellipse
  double a = comRange / 2.0;
  double b = std::sqrt ((a * a) - (distance2d * distance2d / 4.0));
  double ellipseArea = a * b * boost::math::constants::pi<double> ();

  NS_LOG_LOGIC("Ellipse area: " << ellipseArea << " m^2");

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

  NS_LOG_LOGIC("Area covered by objects: " << objectArea << " m^2");

  double weight = std::min (
      1.0,
      std::sqrt (
	  objects.vehicles.size ()
	      / (m_maxVehicleDensity * ellipseArea
		  * SQR_METERS_TO_SQR_KILOMETERS)))
      + std::min (1.0,
		  std::sqrt (objectArea / (m_maxObjectDensity * ellipseArea)));

  NS_LOG_LOGIC("Occupancy weight: " << weight);

  double sigma = sigmaMin + 0.5 * weight * (sigmaMax - sigmaMin);

  NS_ASSERT(m_normalRand);
  double attenuation = m_normalRand->GetValue (0, sigma);

  NS_LOG_LOGIC("sigma=" << sigma << ", attenuation=" << attenuation);
  return attenuation;
}

double
Gemv2PropagationLossModel::CalculateOutOfRangeNoise (
    double txPower, double distance, gemv2::LinkType linkType) const
{
  NS_LOG_FUNCTION(this << txPower << distance << static_cast<int> (linkType));
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
  NS_LOG_FUNCTION(this << distance << vehiclesInLos);

  NS_ASSERT_MSG(
      vehiclesInLos > 0,
      "There has to be at least one vehicle in the LOS for a NLOSv link");

  double freeSpaceLoss = gemv2::FreeSpaceLoss (distance, m_frequency);

  /*
   * This implementation follows the matlab code where only cases
   * with 1, 2 or more than 2 cars
   */
  if (vehiclesInLos == 1)
    {
      return freeSpaceLoss + std::get<0> (m_lossPerVehicleNLOSvSimple);
    }
  else if (vehiclesInLos == 2)
    {
      return freeSpaceLoss + std::get<1> (m_lossPerVehicleNLOSvSimple);
    }
  else
    {
      return freeSpaceLoss + std::get<2> (m_lossPerVehicleNLOSvSimple);
    }
}

gemv2::Environment::ObjectCollection
Gemv2PropagationLossModel::GetObjectsInComEllipse (
    const gemv2::LineSegment2d& lineOfSight,
    double comRange,
    const VehiclePair& involvedVehicles) const
{
  // Find all objects in joint communication ellipse
  gemv2::Environment::ObjectCollection jointObjects;
  m_environment->FindAllInEllipse (lineOfSight.first, lineOfSight.second,
				   comRange, jointObjects);

  // remove sender and receiver from list
  RemoveVehicles (jointObjects.vehicles, involvedVehicles);

  /*
   * A quick note on efficiency here: This should not trigger a
   * copy operation. Most compilers will use NRVO (named return value
   * optimization) and create the return value direct in the place
   * of the return value skipping any copies. If this is not possible
   * the return value will at least be moved instead of copying it.
   */
  return jointObjects;
}


/*
 * PropagationLossModel methods
 */

double
Gemv2PropagationLossModel::DoCalcRxPower (double txPowerDbm,
					  Ptr<MobilityModel> a,
					  Ptr<MobilityModel> b) const
{
  NS_LOG_FUNCTION(this << txPowerDbm);
  NS_ASSERT(a);
  NS_ASSERT(b);


  NS_LOG_LOGIC ("Positions: a=" << a->GetPosition () << ", b=" << b->GetPosition ());

  // Get positions
  auto positions = std::make_pair (a->GetPosition (), b->GetPosition ());

  // calculate distance between both peers
  double distanceLos = CalculateDistance (positions.first, positions.second);

  NS_LOG_LOGIC ("LOS distance: " << distanceLos);

  // check if link is in range
  if (!IsLinkInRange(txPowerDbm, distanceLos))
    {
      NS_LOG_LOGIC("Nodes are out of range.");
      return CalculateOutOfRangeNoise (txPowerDbm, distanceLos,
				       gemv2::LinkType::UNKNOWN);
    }

  // Make line segment between points
  gemv2::LineSegment2d lineOfSight (gemv2::MakePoint2d (positions.first),
				    gemv2::MakePoint2d (positions.second));

  // Get involved vehicles (if set and available)
  auto involvedVehicles = VehiclePair (a->GetObject<gemv2::Vehicle> (),
				       b->GetObject<gemv2::Vehicle> ());

  double txGainDbi = 0.0;	// TODO: get tx gain from antenna model
  double rxGainDbi = 0.0;	// TODO: get rx gain from antenna model

  // First we check for obstructing buildings
  if (m_environment->IntersectsBuildings (lineOfSight))
    {
      NS_LOG_LOGIC("LOS intersects with buildings -> link type: NLOSb");
      return CalcNlosbRxPower (txPowerDbm, distanceLos,
			       lineOfSight, involvedVehicles,
			       txGainDbi, rxGainDbi);
    }
  else if (m_environment->IntersectsFoliage (lineOfSight))
    {
      NS_LOG_LOGIC("LOS intersects with foliage -> link type: NLOSf");
      return CalcNlosfRxPower (txPowerDbm, distanceLos);
    }
  else
    {
      // vehicles in LOS - will only be filled for NLOSv links
      gemv2::Environment::VehicleList vehiclesInLos;

      // No buildings or foliage, check for obstructing vehicles
      m_environment->Intersect (lineOfSight, vehiclesInLos);

      // remove involved vehicles from list
      RemoveVehicles (vehiclesInLos, involvedVehicles);

      // check if there are other vehicles in the LOS
      if (!vehiclesInLos.empty ())
	{
	  NS_LOG_LOGIC(
	      ""
	      "LOS intersects with other vehicles -> link type: NLOSv");

	  return CalcNlosvRxPower (txPowerDbm, distanceLos, lineOfSight,
				   vehiclesInLos, involvedVehicles,
				   txGainDbi, rxGainDbi);
	}
      else
	{
	  NS_LOG_LOGIC("LOS is clear -> link type: LOS");
	  return CalcLosRxPower(txPowerDbm, distanceLos, lineOfSight,
				positions.first, positions.second,
				involvedVehicles,
				txGainDbi, rxGainDbi);
	}
    }
}

int64_t
Gemv2PropagationLossModel::DoAssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION(this << stream);
  if (m_normalRand)
    {
      m_normalRand->SetStream (stream++);
    }
  return 1;
}

bool
Gemv2PropagationLossModel::IsLinkInRange (double /* txPowerDbm */,
					  double distance) const
{
  /*
   * This is the simple version from the original GEMV^2 implementation.
   * However, we might replace this later with an estimation based
   * on the txpower to get rid of the static distance threshold.
   */
  return distance <= m_maxLOSCommRange;
}

double
Gemv2PropagationLossModel::CalcNlosbRxPower (
    double txPowerDbm, double distance,
    const gemv2::LineSegment2d& lineOfSight,
    const VehiclePair& involvedVehicles,
    double txGainDbi, double rxGainDbi) const
{
  // check range
  if (distance > m_maxNLOSbCommRange)
    {
      NS_LOG_LOGIC("NLOSb link out of range: " << distance);
      return CalculateOutOfRangeNoise (txPowerDbm, distance,
				       gemv2::LinkType::NLOSb);
    }

  NS_LOG_LOGIC ("NLOSb link is in range: " << distance);

  double rxPowerLargeScaleDbm = -std::numeric_limits<double>::max ();

  switch (m_modelNLOSb)
    {
    case gemv2::NLOSB_MODEL_LOG_DISTANCE:
      rxPowerLargeScaleDbm = txPowerDbm + txGainDbi + rxGainDbi
	  - gemv2::LogDistanceLoss(distance,
				   m_frequency,
				   m_v2vPropagation.pathLossExpNLOSb);
      NS_LOG_LOGIC(
	  "Log distance NLOSb model large scale loss: " <<
	  (txPowerDbm - rxPowerLargeScaleDbm));
      break;
    case gemv2::NLOSB_MODEL_REFLECTION_DIFFRACTION:
      NS_ASSERT_MSG(false, "NLOSb model not implemented (yet)");
      break;
    default:
      NS_ASSERT_MSG(false, "Unknown NLOSb model");
      break;
    }

  /*
   * And now the small scale loss...
   */

  auto objectsInRange =
      GetObjectsInComEllipse(lineOfSight, m_maxNLOSvCommRange, involvedVehicles);

  double smallScaleVariation =
      CalculateSmallScaleVariations (
	  boost::geometry::length (lineOfSight),
	  m_maxNLOSbCommRange,
	  objectsInRange,
	  m_v2vPropagation.smallScaleSigmaMinNLOSb,
	  m_v2vPropagation.smallScaleSigmaMaxNLOSb);

  NS_LOG_LOGIC ("Small scale variation: " << smallScaleVariation);

  auto rxPowerDbm = rxPowerLargeScaleDbm - smallScaleVariation;
  NS_LOG_LOGIC ("Received power: " << rxPowerDbm);
  return rxPowerDbm;
}

double
Gemv2PropagationLossModel::CalcNlosfRxPower (
    double txPowerDbm, double distance) const
{
  if (distance > m_maxNLOSbCommRange)
    {
      NS_LOG_LOGIC("NLOSf link out of range: " << distance);
      return CalculateOutOfRangeNoise (txPowerDbm, distance,
				       gemv2::LinkType::NLOSf);
    }

  NS_LOG_LOGIC ("NLOSf link is in range: " << distance);

  // TODO: implement precise calculation
  return -std::numeric_limits<double>::max ();
}

double
Gemv2PropagationLossModel::CalcNlosvRxPower (
    double txPowerDbm, double distance,
    const gemv2::LineSegment2d& lineOfSight,
    const gemv2::Environment::VehicleList& vehiclesInLos,
    const VehiclePair& involvedVehicles,
    double txGainDbi, double rxGainDbi) const
{
  if (distance > m_maxNLOSvCommRange)
    {
      NS_LOG_LOGIC("NLOSv link out of range: " << distance);
      return CalculateOutOfRangeNoise (txPowerDbm, distance,
				       gemv2::LinkType::NLOSv);
    }

  NS_LOG_LOGIC ("NLOSv link is in range: " << distance);

  /*
   * Determine large scale loss according to specific model
   */
  double rxPowerLargeScaleDbm = -std::numeric_limits<double>::max ();

  auto objectsInRange =
      GetObjectsInComEllipse(lineOfSight, m_maxNLOSvCommRange, involvedVehicles);

  switch (m_modelNLOSv)
    {
    case gemv2::NLOSV_MODEL_SIMPLE:
      rxPowerLargeScaleDbm = txPowerDbm + txGainDbi + rxGainDbi
	  - CalculateSimpleNlosvLoss (
	      distance, vehiclesInLos.size ());
      NS_LOG_LOGIC(
	  "Simple NLOSv model large scale loss: " <<
	  (txPowerDbm - rxPowerLargeScaleDbm));
      break;
    case gemv2::NLOSV_MODEL_ITU_R_MULTIPLE_KNIFE_EDGE:
      // TODO: implement me
    case gemv2::NLOSV_MODEL_BULLINGTON_KNIFE_EDGE:
      // TODO: implement me
      NS_ASSERT_MSG(false, "NLOSv model not implemented (yet)");
      break;
    default:
      NS_ASSERT_MSG(false, "Unknown NLOSv model");
      break;
    }

  /*
   * And now the small scale loss...
   */
  double smallScaleVariation =
      CalculateSmallScaleVariations (
	  boost::geometry::length (lineOfSight),
	  m_maxNLOSvCommRange,
	  objectsInRange,
	  m_v2vPropagation.smallScaleSigmaMinNLOSv,
	  m_v2vPropagation.smallScaleSigmaMaxNLOSv);
  NS_LOG_LOGIC ("Small scale variation: " << smallScaleVariation);

  auto rxPowerDbm = rxPowerLargeScaleDbm - smallScaleVariation;
  NS_LOG_LOGIC ("Received power: " << rxPowerDbm);
  return rxPowerDbm;
}

double
Gemv2PropagationLossModel::CalcLosRxPower (
    double txPowerDbm, double distance,
    const gemv2::LineSegment2d& lineOfSight,
    const Vector& txPos, const Vector& rxPos,
    const VehiclePair& involvedVehicles,
    double txGainDbi, double rxGainDbi) const
{
  /*
   * Note: No need to check the distance here since we checked
   *       this in the beginning to avoid searching the environment
   *       for links beyond the maximum communication range.
   */

  /*
   * LOS links use the two-ray-ground loss model for the large
   * scale propagation loss.
   */
  double eTot = gemv2::TwoRayGroundLoss (distance, txPos, rxPos,
					 m_frequency, txPowerDbm, txGainDbi,
					 m_antennaPolarization,
					 m_groundPermittivity);

  double rxPowerLargeScaleDbm =
      gemv2::EfieldToPowerDbm (eTot, rxGainDbi, m_frequency);
  NS_LOG_LOGIC("Two-ray-ground loss: " << (txPowerDbm - rxPowerLargeScaleDbm));


  /*
   * And now the small scale loss...
   */
  double smallScaleVariation =
      CalculateSmallScaleVariations (
	  boost::geometry::length (lineOfSight),
	  m_maxLOSCommRange,
	  GetObjectsInComEllipse(lineOfSight, m_maxLOSCommRange, involvedVehicles),
	  m_v2vPropagation.smallScaleSigmaMinLOS,
	  m_v2vPropagation.smallScaleSigmaMaxLOS);
  NS_LOG_LOGIC ("Small scale variation: " << smallScaleVariation);

  auto rxPowerDbm = rxPowerLargeScaleDbm - smallScaleVariation;
  NS_LOG_LOGIC ("Received power: " << rxPowerDbm);
  return rxPowerDbm;}


}
// namespace ns3
