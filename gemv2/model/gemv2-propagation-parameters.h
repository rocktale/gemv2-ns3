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
#ifndef GEMV2_PROPAGATION_PARAMETERS_H
#define GEMV2_PROPAGATION_PARAMETERS_H

namespace ns3
{

/*!
 * @brief Propagation parameters for GEMV^2.
 *
 * This is only a subset intended to be used with different
 * values for V2V and V2I links.
 */
struct Gemv2PropagationParameters
{
  // Path loss exponents

  //! Path loss exponent NLOSb
  double pathLossExpNLOSb;
  //! Path loss exponent NLOSf
  double pathLossExpNLOSf;

  // Small-scale signal variation parameters

  //! Minimum standard deviation of small scale variation for LOS
  double smallScaleMinFadingLOS;
  //! Minimum standard deviation of small scale variation for NLOSv
  double smallScaleMinFadingNLOSv;
  //! Minimum standard deviation of small scale variation for NLOSb
  double smallScaleMinFadingNLOSb;
  //! Minimum standard deviation of small scale variation for NLOSf
  double smallScaleMinFadingNLOSf;

  //! Maximum standard deviation of small scale variation for LOS
  double smallScaleMaxFadingLOS;
  //! Maximum standard deviation of small scale variation for NLOSv
  double smallScaleMaxFadingNLOSv;
  //! Maximum standard deviation of small scale variation for NLOSb
  double smallScaleMaxFadingNLOSb;
  //! Maximum standard deviation of small scale variation for NLOSf
  double smallScaleMaxFadingNLOSf;


  /*!
   * @brief Default constructor using V2V values from GEMV^2.
   *
   * Default values taken from the measurements described in the paper
   * "Geometry-Based Vehicle-to-Vehicle Channel Modeling for Large-Scale
   * Simulation" by Boban, Barros, and Tonguz.
   */
  Gemv2PropagationParameters ()
    : pathLossExpNLOSb (2.9), pathLossExpNLOSf (2.7),
      smallScaleMinFadingLOS (3.3),
      smallScaleMinFadingNLOSv (3.8),
      smallScaleMinFadingNLOSb (4.1),
      smallScaleMinFadingNLOSf (smallScaleMinFadingNLOSb),
      smallScaleMaxFadingLOS (5.2),
      smallScaleMaxFadingNLOSv (5.3),
      smallScaleMaxFadingNLOSb (6.8),
      smallScaleMaxFadingNLOSf (smallScaleMaxFadingNLOSb)
  {
  }

};

}  // namespace ns3

#endif /* GEMV2_PROPAGATION_PARAMETERS_H */
