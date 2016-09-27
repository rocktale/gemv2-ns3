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
TwoRayGroundLoss (
    double distance, double txHeight, double rxHeight,
    double frequency, double txPower, double txGain,
    AntennaPolarization polarization,
    double permittivity)
{
  // LOS distance
  double dLos =
      std::sqrt (std::pow (txHeight-rxHeight, 2) + std::pow (distance, 2));

  // sine and cosine of the incident angle
  double sinTheta = (txHeight + rxHeight) / dLos;
  double cosTheta = distance / dLos;

  // calculate effective reflection coefficient
  double reflectionCoefficient = 0;
  double sqrtPermMinusCosThetaSquared =
      std::sqrt (permittivity - std::pow (cosTheta, 2));

  switch(polarization)
  {
    case ANTENNA_POLARIZATION_VERTICAL:
      reflectionCoefficient =
	  (-permittivity * sinTheta + sqrtPermMinusCosThetaSquared) /
	  (permittivity * sinTheta + sqrtPermMinusCosThetaSquared);
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
  txPower = std::pow (10.0, txPower/10.0) / 1000.0;

  // reference distance in meters
  double d0 = 1.0;

  // gain from dBi to factor
  txGain = std::pow (10.0, txGain/10.0);

  // reference power flux density at distance d0
  double Pd0 =
      txPower * txGain /
      (4.0 * constants::pi<double> () * std::pow (d0, 2));

  // reference E-field
  double E0 = std::sqrt (Pd0 * 120.0 * constants::pi<double> ());

  // ground reflected distance
  double dGround =
      std::sqrt (std::pow (txHeight+rxHeight, 2) + std::pow (distance, 2));

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
	  reflectionCoefficient * E0 * d0 / dGround *
	  std::cos (frequencyAngular * (dLos/speedOfLight - dGround/speedOfLight));
}

double
EfieldToPowerDbm (double eTot, double rxGain, double frequency)
{
  // gain from dBi to factor
  rxGain = std::pow (10.0, rxGain/10.0);

  // received power in W
  double rxPowerW =
      std::pow (eTot, 2) * rxGain * std::pow (speedOfLight / frequency, 2) /
      (480.0 * constants::pi_sqr<double> ());

  // return received power in dBm
  return 10.0 * std::log10 (rxPowerW * 1000.0);

}


}  // namespace gemv2
}  // namespace ns3
