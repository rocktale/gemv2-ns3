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
#ifndef GEMV2_MODELS_H
#define GEMV2_MODELS_H

#include <utility>
#include <ns3/gemv2-types.h>

namespace ns3 {
namespace gemv2 {

/*!
 * @brief Calculate the two ray ground path loss.
 * @param distance	 Distance between sender and receiver (2d) [m]
 * @param txHeight	 Height of the transmitter antenna [m]
 * @param rxHeight	 Height of the receiver antenna [m]
 * @param frequency	 Frequency of the signal
 * @param txPower	 Transmit power [dBm]
 * @param txGain	 Gain of the transmitter antenna [dBi]
 * @param polarization	 Polarization of the antennas
 * @param permittivity	 Relative permittivity of the ground
 * @return E-field
 */
double
TwoRayGroundLoss (double distance, double txHeight, double rxHeight,
		  double frequency, double txPower, double txGain,
		  AntennaPolarization polarization,
		  double permittivity);

/*!
 * @brief Calculate the received power from the E-Field.
 * @param eTot		Received E-Field [V/m]
 * @param rxGain	Gain of the receiver antenna [dBi]
 * @param frequency	Frequency of the signal
 * @return Received power [dBm]
 */
double
EfieldToPowerDbm (double eTot, double rxGain, double frequency);

}  // namespace gemv2
}  // namespace ns3

#endif /* GEMV2_MODELS_H */
