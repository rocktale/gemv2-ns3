// An essential include is test.h
#include "ns3/test.h"

#include "ns3/gemv2-environment.h"
#include <boost/geometry/io/wkt/read.hpp>

// Do not put your test classes in namespace ns3.  You may find it useful
// to use the using directive to access the ns3 namespace directly
using namespace ns3;


// This will just test basic intersection checks on buildings
class Gemv2BuildingIntersectionTestCase : public TestCase
{
public:
  Gemv2BuildingIntersectionTestCase ();

private:
  void DoRun (void) override;

  Ptr<gemv2::Environment> env;
};

// Add some help text to this case to describe what it is intended to test
Gemv2BuildingIntersectionTestCase::Gemv2BuildingIntersectionTestCase ()
  : TestCase ("GEMV^2 building intersection test case")
{
  // create environment
  env = Create<gemv2::Environment> ();

  // create some polygons
  gemv2::Polygon2d p1, p2, p3;

  boost::geometry::read_wkt("POLYGON((10 10, 10 20, 20 20, 20 10, 10 10))", p1);
  boost::geometry::read_wkt("POLYGON((30 30, 30 50, 50 50, 50 30, 30 30))", p2);
  boost::geometry::read_wkt("POLYGON((30 15, 40 25, 50 15, 40 5, 30 15))", p3);

  // add polygons as objects
  env->AddBuilding(Create<gemv2::Building> (p1));
  env->AddBuilding(Create<gemv2::Building> (p2));
  env->AddBuilding(Create<gemv2::Building> (p3));
}

void
Gemv2BuildingIntersectionTestCase::DoRun (void)
{
  gemv2::LineSegment2d none ({40,0}, {55, 15});
  gemv2::LineSegment2d one ({25,0}, {50, 25});
  gemv2::LineSegment2d two ({0,0}, {100, 100});

  NS_TEST_ASSERT_MSG_EQ (env->IntersectsBuildings (none), false,
			 "Should not intersect any building");
  NS_TEST_ASSERT_MSG_EQ (env->IntersectsBuildings (one), true,
			 "Should intersect at least one building");
  NS_TEST_ASSERT_MSG_EQ (env->IntersectsBuildings (two), true,
			 "Should intersect at least one building");

  gemv2::Environment::BuildingList buildings;

  env->Intersect (none, buildings);
  NS_TEST_ASSERT_MSG_EQ (buildings.size (), 0, "Should not intersect");
  buildings.clear ();

  env->Intersect (one, buildings);
  NS_TEST_ASSERT_MSG_EQ (buildings.size (), 1, "Should intersect once");
  buildings.clear ();

  env->Intersect (two, buildings);
  NS_TEST_ASSERT_MSG_EQ (buildings.size (), 2, "Should intersect twice");
  buildings.clear ();
}


// This will just test basic intersection checks on vehicles
class Gemv2VehicleIntersectionTestCase : public TestCase
{
public:
  Gemv2VehicleIntersectionTestCase ();

private:
  void DoRun (void) override;

  Ptr<gemv2::Environment> env;

  std::vector<Ptr<gemv2::Vehicle>> vehicles;
};

Gemv2VehicleIntersectionTestCase::Gemv2VehicleIntersectionTestCase ()
  : TestCase ("GEMV^2 vehicle intersection test case")
{
  // create environment
  env = Create<gemv2::Environment> ();

  // create a few vehicles
  vehicles.push_back (Create<gemv2::Vehicle> (4.5, 1.8, 1.5));	// small
  vehicles.back()->SetPosition (Vector (25, 40, 0));
  vehicles.back()->SetHeading (0);

  vehicles.push_back (Create<gemv2::Vehicle> (10, 2.5, 2.2));	// truck
  vehicles.back()->SetPosition (Vector (10, 40, 0));
  vehicles.back()->SetHeading (90);

  // add vehicles to the environment
  for (auto const& v : vehicles)
    {
      env->AddVehicle(v);
    }
}

void
Gemv2VehicleIntersectionTestCase::DoRun (void)
{
  gemv2::LineSegment2d truck ({6, 10}, {6, 50});
  gemv2::LineSegment2d small ({24.5, 0}, {25.5, 70});

  // list of vehicles, used to store results
  gemv2::Environment::VehicleList iv;

  env->Intersect (truck, iv);
  NS_TEST_ASSERT_MSG_EQ (iv.size (), 1, "Should intersect one vehicle");
  NS_TEST_ASSERT_MSG_EQ (iv.front (), vehicles.at(1), "Should intersect the truck");
  iv.clear ();

  // update truck heading and rebuild tree
  vehicles[1]->SetHeading (180);
  env->ForceVehicleTreeRebuild ();

  env->Intersect (truck, iv);
  NS_TEST_ASSERT_MSG_EQ (iv.size (), 0, "Should intersect no vehicle anymore");
  iv.clear ();

  // check small vehicle
  env->Intersect (small, iv);
  NS_TEST_ASSERT_MSG_EQ (iv.size (), 1, "Should intersect one vehicle");
  NS_TEST_ASSERT_MSG_EQ (iv.front (), vehicles.at(0), "Should intersect the small vehicle");
  iv.clear ();

  // move  both vehicles and rebuild tree
  vehicles[0]->SetPosition (Vector (25, 50, 0));
  vehicles[1]->SetPosition (Vector (24, 30, 0));
  vehicles[1]->SetHeading (45);
  env->ForceVehicleTreeRebuild ();

  env->Intersect (small, iv);
  NS_TEST_ASSERT_MSG_EQ (iv.size (), 2, "Should intersect both vehicles");
  iv.clear ();

  // test with ray slightly off the tilted truck
  env->Intersect (gemv2::LineSegment2d ({13, 21}, {33, 41}), iv);
  NS_TEST_ASSERT_MSG_EQ (iv.size (), 0, "Should not intersect any vehicle");
}


// The TestSuite class names the TestSuite, identifies what type of TestSuite,
// and enables the TestCases to be run.  Typically, only the constructor for
// this class must be defined
//
class Gemv2EnvironmentTestSuite : public TestSuite
{
public:
  Gemv2EnvironmentTestSuite ();
};

Gemv2EnvironmentTestSuite::Gemv2EnvironmentTestSuite ()
  : TestSuite ("gemv2-environment", UNIT)
{
  // TestDuration for TestCase can be QUICK, EXTENSIVE or TAKES_FOREVER
  AddTestCase (new Gemv2BuildingIntersectionTestCase, TestCase::QUICK);
  AddTestCase (new Gemv2VehicleIntersectionTestCase, TestCase::QUICK);
}

// Do not forget to allocate an instance of this TestSuite
static Gemv2EnvironmentTestSuite gemv2EnvironmentTestSuite;
