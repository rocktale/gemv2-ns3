/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/core-module.h"
#include "ns3/gemv2-models.h"

#include <iostream>
#include <fstream>

using namespace ns3;

// Configuration of the experiment
struct Configuration
{
  double freqInGhz = 5.9;		// GHz
  double maxDistanceInMeters = 1000.0; 	// meters
  double distanceStepInMeters = 0.01; 	// meters
  double txPowerInDbm = 20;		// dBm
  double txHeightInMeters = 1.5;	// meters
  double rxHeightInMeters = 1.5;	// meters

  // Default value from GEMV^2 paper.
  // However, literature suggests 15 for solid ground.
  double permittivity = 1.003;
  gemv2::AntennaPolarization polarization = gemv2::ANTENNA_POLARIZATION_HORIZONTAL;

  void
  ConfigureCommandLine(CommandLine& cmd)
  {
    cmd.AddValue ("frequency", "Carrier frequency in GHz", freqInGhz);
    cmd.AddValue ("max-distance", "Maximum distance in meters", maxDistanceInMeters);
    cmd.AddValue ("distance-step", "Distance step in meters", distanceStepInMeters);
    cmd.AddValue ("tx-power", "Transmit power in dBm", txPowerInDbm);
    cmd.AddValue ("tx-height", "Height of the TX antenna in meters", txHeightInMeters);
    cmd.AddValue ("rx-height", "Height of the RX antenna in meters", rxHeightInMeters);
    cmd.AddValue ("permittivity", "Relative permittivity e_r", permittivity);

    cmd.AddValue ("polarization", "Antenna polarization (horizontal, vertical)",
		  MakeCallback(&Configuration::ParsePolarization, this));
  }

  bool
  ParsePolarization(std::string polString)
  {
    if (polString == "horizontal")
      {
        polarization = gemv2::ANTENNA_POLARIZATION_HORIZONTAL;
        return true;
      }

    if (polString == "vertical")
      {
        polarization = gemv2::ANTENNA_POLARIZATION_VERTICAL;
        return true;
      }

    return false;
  }
};

/*!
 * @brief Run the experiment and write values to output stream
 * @param config	Configuration to run
 * @param os		Stream to write the values to
 */
void
RunExperiment(const Configuration& config, std::ostream& os)
{
  os << "distance rxpower" << std::endl;

  for (double d = 0;
       d <= config.maxDistanceInMeters;
       d += config.distanceStepInMeters)
    {
      double frequency = config.freqInGhz * 1e09;
      double eTot = gemv2::TwoRayGroundLoss(
	  d, config.txHeightInMeters, config.rxHeightInMeters,
	  frequency, config.txPowerInDbm, 0,
	  config.polarization, config.permittivity);

      double rxPower = gemv2::EfieldToPowerDbm(eTot, 0, frequency);

      os << d << " " << rxPower << std::endl;
    }
}

int 
main (int argc, char *argv[])
{
  Configuration config;
  std::string outputFile = "";

  CommandLine cmd;
  config.ConfigureCommandLine(cmd);
  cmd.AddValue ("output", "File to write the output data to", outputFile);
  cmd.Parse (argc,argv);

  if (outputFile.empty())
    {
      RunExperiment (config, std::cout);
    }
  else
    {
      std::fstream fs(outputFile.c_str(), std::fstream::out | std::fstream::trunc);
      if (fs.is_open())
      	{
      	  RunExperiment (config, fs);
      	  fs.close();
      	}
      else
	{
	  std::cerr << "Failed to open output file: " << outputFile << std::endl;
	  return 1;
	}
    }

  return 0;
}


