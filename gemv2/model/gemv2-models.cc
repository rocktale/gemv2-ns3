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
#include "gemv2-models.h"

#include <cmath>
#include <boost/math/constants/constants.hpp>
#include <ns3/assert.h>

namespace constants = boost::math::constants;

namespace
{
// Speed of light [m/s]
constexpr double speedOfLight = 299792458.0;
}

namespace ns3 {
namespace gemv2 {

double
FreeSpaceLoss (double distance, double frequency)
{
  return 20.0 * std::log10 (
      4.0 * constants::pi<double> () * distance * frequency / speedOfLight);
}


double
TwoRayGroundLoss (
    double dLos, const Vector& txPos, const Vector& rxPos,
    double frequency, double txPower, double txGain,
    AntennaPolarization polarization,
    double permittivity)
{
  // 2d distance in the horizontal plane
  double distance2d =
      std::sqrt(
	  std::pow (txPos.x - rxPos.x, 2) +
	  std::pow (txPos.y - rxPos.y, 2));

  // ground reflected distance
  double dGround =
      std::sqrt (std::pow (txPos.z + rxPos.z, 2) + std::pow (distance2d, 2));

  // sine and cosine of the incident angle
  double sinTheta = (txPos.z + rxPos.z) / dGround;
  double cosTheta = distance2d / dGround;

  // calculate effective reflection coefficient
  double reflectionCoefficient = 0;
  double sqrtPermMinusCosThetaSquared =
      std::sqrt (permittivity - std::pow (cosTheta, 2));

  switch(polarization)
  {
    case ANTENNA_POLARIZATION_VERTICAL:
      /*
       * This is the version also used in the matlab script. This leads
       * to a strange behavior for large distances, where the received power
       * does not drop by the distance to the power of 4. Literature (and
       * also wikipedia:
       * https://en.wikipedia.org/wiki/Two-ray_ground-reflection_model
       * provide a slight variation that leads to different results that
       * are much closer to the horizontal polarization case. We'll stick
       * to the matlab code for now, but it might be worth checking
       * if this is really the intended behavior.
       */
      reflectionCoefficient =
	  ((-permittivity) * sinTheta + sqrtPermMinusCosThetaSquared) /
	  (permittivity * sinTheta + sqrtPermMinusCosThetaSquared);

      // alternative from wikipedia
//      reflectionCoefficient =
//	  (sinTheta - sqrtPermMinusCosThetaSquared/permittivity) /
//	  (sinTheta + sqrtPermMinusCosThetaSquared/permittivity);
      break;
    case ANTENNA_POLARIZATION_HORIZONTAL:
      reflectionCoefficient =
	  (sinTheta - sqrtPermMinusCosThetaSquared) /
	  (sinTheta + sqrtPermMinusCosThetaSquared);
      break;
    default:
      NS_ASSERT_MSG (false, "Unknown antenna polarization");
      break;
  }

  // power to W
  double txPowerInW = std::pow (10.0, txPower/10.0) / 1000.0;

  // reference distance in meters
  double d0 = 1.0;

  // gain from dBi to factor
  double txGainFactor = std::pow (10.0, txGain/10.0);

  // reference power flux density at distance d0
  double Pd0 =
      txPowerInW * txGainFactor /
      (4.0 * constants::pi<double> () * std::pow (d0, 2));

  // reference E-field
  double E0 = std::sqrt (Pd0 * 120.0 * constants::pi<double> ());

  // angular frequency
  double frequencyAngular = constants::two_pi<double> () * frequency;

  // E-field
  /*
   * Note: In the first part we skipped the cosine term because its argument
   *       always evaluates to 0 and the factor is therefore always 1. In the
   *       original GEMV^2 matlab code it stated: (d1/c - d1/c) which is
   *       obviously always 0.
   */
  return (E0 * d0 / dLos) +
	  reflectionCoefficient * (E0 * d0 / dGround) *
	  std::cos (frequencyAngular * (dLos/speedOfLight - dGround/speedOfLight));
}

double
EfieldToPowerDbm (double eTot, double rxGain, double frequency)
{
  // gain from dBi to factor
  double rxGainFactor = std::pow (10.0, rxGain/10.0);

  // received power in W
  double rxPowerW =
      std::pow (eTot, 2) * rxGainFactor * std::pow (speedOfLight / frequency, 2) /
      (480.0 * constants::pi_sqr<double> ());

  // return received power in dBm
  return 10.0 * std::log10 (rxPowerW * 1000.0);

}


}  // namespace gemv2
}  // namespace ns3
