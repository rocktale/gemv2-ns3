/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/gemv2-module.h"
#include "ns3/stats-module.h"


#include <boost/geometry/io/wkt/read.hpp>

#include <iostream>
#include <sstream>
#include <fstream>

using namespace ns3;

using BuildingList = gemv2::Environment::BuildingList;

std::string defaultBuildings =
   // std::string("POLYGON((100 100, 200 100, 200 200, 100 200, 100 100))\n");
    std::string("POLYGON((20 20, 40 20, 40 100, 20 100, 20 20))\n");

BuildingList
createBuildings(std::istream& is)
{
  BuildingList buildings;

  std::string line;
  while (is.good())
    {
      std::getline(is, line);
      if (is.good() && !line.empty())
	{
	  gemv2::Polygon2d polygon;
	  // TODO: handle error
	  boost::geometry::read_wkt(line, polygon);

	  buildings.push_back(Create<gemv2::Building> (polygon));
	}
    }

  return buildings;
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
  }
};

/*!
 * @brief Run the experiment and write values to output stream
 * @param config	Configuration to run
 * @param buildings Buildings to consider
 * @param os		Stream to write the values to
 */
void
RunExperiment(const Configuration& config,
		const BuildingList& buildings,
		std::ostream& os)
{
  auto propagation = CreateObject<Gemv2PropagationLossModel> ();
  Ptr<gemv2::Environment> env = Create<gemv2::Environment> ();
  env->AddBuildings (buildings);
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

  BuildingList buildings;

  if (buildingFile.empty())
    {
      std::stringstream ss(defaultBuildings);
      buildings = createBuildings(ss);
    }
  else
    {
      std::fstream inFile(buildingFile, std::fstream::in);
      buildings = createBuildings(inFile);
    }


  if (outputFile.empty())
    {
      RunExperiment (config, buildings, std::cout);
    }
  else
    {
      std::fstream fs(outputFile.c_str(), std::fstream::out | std::fstream::trunc);
      if (fs.is_open())
      	{
      	  RunExperiment (config, buildings, fs);
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


