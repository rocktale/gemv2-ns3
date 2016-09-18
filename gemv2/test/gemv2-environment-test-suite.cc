// An essential include is test.h
#include "ns3/test.h"

#include "ns3/gemv2-environment.h"
#include <boost/geometry/io/wkt/read.hpp>

// Do not put your test classes in namespace ns3.  You may find it useful
// to use the using directive to access the ns3 namespace directly
using namespace ns3;


// This will just test basic geometry operations
class Gemv2BasicEnvironmentTestCase : public TestCase
{
public:
  Gemv2BasicEnvironmentTestCase ();
//  virtual ~Gemv2BasicEnvironmentTestCase ();

private:
  virtual void DoRun (void);

  Ptr<gemv2::Environment> env;
};

// Add some help text to this case to describe what it is intended to test
Gemv2BasicEnvironmentTestCase::Gemv2BasicEnvironmentTestCase ()
  : TestCase ("GEMV^2 environment test case")
{
  // create environment
  env = Create<gemv2::Environment> ();

  // create some objects
  gemv2::Polygon2d p1, p2, p3;

  boost::geometry::read_wkt("POLYGON((10 10, 10 20, 20 20, 20 10, 10 10))", p1);
  boost::geometry::read_wkt("POLYGON((30 30, 30 50, 50 50, 50 30, 30 30))", p2);
  boost::geometry::read_wkt("POLYGON((30 15, 40 25, 50 15, 40 5, 30 15))", p3);


  env->AddBuilding(Create<gemv2::Building> (p1));
  env->AddBuilding(Create<gemv2::Building> (p2));
  env->AddBuilding(Create<gemv2::Building> (p3));
}

//
// This method is the pure virtual method from class TestCase that every
// TestCase must implement
//
void
Gemv2BasicEnvironmentTestCase::DoRun (void)
{
  gemv2::LineSegment2d none ({25, 0}, {25, 100});
  gemv2::LineSegment2d one ({25,0}, {50, 25});
  gemv2::LineSegment2d two ({0,0}, {100, 100});

  gemv2::Environment::BuildingList buildings;
  env->intersect (none, buildings);
  NS_TEST_ASSERT_MSG_EQ (buildings.size (), 0, "Should not intersect");
  buildings.clear ();

  env->intersect (one, buildings);
  NS_TEST_ASSERT_MSG_EQ (buildings.size (), 1, "Should intersect once");
  buildings.clear ();

  env->intersect (two, buildings);
  NS_TEST_ASSERT_MSG_EQ (buildings.size (), 2, "Should intersect twice");
  buildings.clear ();

  env->findInRange (gemv2::Point2d (0, 0), 20, buildings);
  NS_TEST_ASSERT_MSG_EQ (buildings.size (), 1, "Should find one building");
  buildings.clear ();

  env->findInRange (gemv2::Point2d (25, 25), 20, buildings);
  NS_TEST_ASSERT_MSG_EQ (buildings.size (), 3, "Should find three buildings");
  buildings.clear ();
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
  AddTestCase (new Gemv2BasicEnvironmentTestCase, TestCase::QUICK);
}

// Do not forget to allocate an instance of this TestSuite
static Gemv2EnvironmentTestSuite gemv2EnvironmentTestSuite;
