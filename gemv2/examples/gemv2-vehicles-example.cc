/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/gemv2-module.h"
#include "ns3/stats-module.h"


#include <iostream>
#include <sstream>
#include <fstream>

using namespace ns3;

void
CreateVehicles(std::size_t number, double distance, gemv2::Environment& env)
{
  for (std::size_t i = 0; i < number; ++i)
    {
      auto v = Create<gemv2::Vehicle> (5.0, 2.0, 1.5);
      v->SetPosition(Vector(distance * (i+1), distance * (i+1), 0));
      v->SetHeading(90);
      env.AddVehicle(v);
    }
}

// Configuration of the experiment
struct Configuration
{
  double maxXInMeters = 1000.0; 	// meters
  double maxYInMeters = 1000.0; 	// meters
  double distanceStepInMeters = 10.0; 	// meters
  double txPowerInDbm = 23;		// dBm
  double txHeightInMeters = 1.5;	// meters
  double rxHeightInMeters = 1.5;	// meters
  std::size_t numOfsamples = 20;

  std::size_t numOfVehicles = 1; 	// vehicles
  double vehicleDistance = 50.0;	// distance between vehicles in meters

  void
  ConfigureCommandLine(CommandLine& cmd)
  {
    cmd.AddValue ("max-x", "Maximum x distance in meters", maxXInMeters);
    cmd.AddValue ("max-y", "Maximum y distance in meters", maxYInMeters);
    cmd.AddValue ("distance-step", "Distance step in meters", distanceStepInMeters);
    cmd.AddValue ("tx-power", "Transmit power in dBm", txPowerInDbm);
    cmd.AddValue ("tx-height", "Height of the TX antenna in meters", txHeightInMeters);
    cmd.AddValue ("rx-height", "Height of the RX antenna in meters", rxHeightInMeters);
    cmd.AddValue ("samples", "Number of samples per position", numOfsamples);
    cmd.AddValue ("vehicles", "Number of vehicles", numOfVehicles);
    cmd.AddValue ("vehicle-distance", "Distance between vehicles", vehicleDistance);
  }
};

/*!
 * @brief Run the experiment and write values to output stream
 * @param config	Configuration to run
 * @param os		Stream to write the values to
 */
void
RunExperiment(const Configuration& config,
		std::ostream& os)
{
  auto propagation = CreateObject<Gemv2PropagationLossModel> ();
  Ptr<gemv2::Environment> env = Create<gemv2::Environment> ();
  CreateVehicles(config.numOfVehicles, config.vehicleDistance, *env);
  propagation->SetEnviroment (env);

  os << "x y rxpower_mean rxpower_var rxpower_min rxpower_max" << std::endl;

  auto sender = CreateObject<ConstantPositionMobilityModel> ();
  sender->SetPosition(Vector(0,0,config.txHeightInMeters));

  auto receiver = CreateObject<ConstantPositionMobilityModel> ();

  for (double x = 0; x <= config.maxXInMeters; x += config.distanceStepInMeters)
    {
      for (double y = 0; y <= config.maxYInMeters; y += config.distanceStepInMeters)
	{
	  receiver->SetPosition(Vector(x,y,config.rxHeightInMeters));


	  // calculate received power samples
	  Average<double> rxPower;
	  for (std::size_t c = 0; c < config.numOfsamples; ++c)
	    {
	      rxPower.Update(
		propagation->CalcRxPower(config.txPowerInDbm, sender, receiver));
	    }

	  os << x << " " << y << " "
	     << rxPower.Avg() << " " << rxPower.Var () << " "
	     << rxPower.Min() << " " << rxPower.Max () << std::endl;
	}
    }
}

int
main (int argc, char *argv[])
{
  Configuration config;
  bool verbose = false;
  std::string outputFile = "";
  std::string buildingFile = "";

  CommandLine cmd;
  config.ConfigureCommandLine(cmd);
  cmd.AddValue ("output", "File to write the output data to", outputFile);
  cmd.AddValue ("buildings", "File to read buildings from (as WKT polygons", buildingFile);
  cmd.AddValue ("verbose", "Generate verbose logging output", verbose);
  cmd.Parse (argc,argv);

  if (verbose)
    {
      LogComponentEnable (
	  "Gemv2PropagationLossModel",
	  LogLevel (LOG_LEVEL_ALL | LOG_PREFIX_FUNC | LOG_PREFIX_TIME));

      LogComponentEnable (
	  "Gemv2Building",
	  LogLevel (LOG_LEVEL_ALL | LOG_PREFIX_FUNC | LOG_PREFIX_TIME));
    }

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


