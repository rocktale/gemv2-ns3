// An essential include is test.h
#include "ns3/test.h"

#include "ns3/gemv2-geometry.h"

// Do not put your test classes in namespace ns3.  You may find it useful
// to use the using directive to access the ns3 namespace directly
using namespace ns3;


// This will just test basic geometry operations
class Gemv2BasicGeometryTestCase : public TestCase
{
public:
  Gemv2BasicGeometryTestCase ();
//  virtual ~Gemv2BasicGeometryTestCase ();

private:
  virtual void DoRun (void);
};

// Add some help text to this case to describe what it is intended to test
Gemv2BasicGeometryTestCase::Gemv2BasicGeometryTestCase ()
  : TestCase ("Gemv2 geometry test case")
{
}

//
// This method is the pure virtual method from class TestCase that every
// TestCase must implement
//
void
Gemv2BasicGeometryTestCase::DoRun (void)
{
  gemv2::LineSegment2d line1({0,0}, {10, 10});
  gemv2::LineSegment2d line2({0,10}, {10, 0});

  NS_TEST_ASSERT_MSG_EQ (boost::geometry::intersects (line1, line2), true, "");
}




// The TestSuite class names the TestSuite, identifies what type of TestSuite,
// and enables the TestCases to be run.  Typically, only the constructor for
// this class must be defined
//
class Gemv2GeometryTestSuite : public TestSuite
{
public:
  Gemv2GeometryTestSuite ();
};

Gemv2GeometryTestSuite::Gemv2GeometryTestSuite ()
  : TestSuite ("gemv2-geometry", UNIT)
{
  // TestDuration for TestCase can be QUICK, EXTENSIVE or TAKES_FOREVER
  AddTestCase (new Gemv2BasicGeometryTestCase, TestCase::QUICK);
}

// Do not forget to allocate an instance of this TestSuite
static Gemv2GeometryTestSuite gemv2GeometryTestSuite;
