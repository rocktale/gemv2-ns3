# GEMV^2 for ns-3
Implementation of the GEMV^2 propagation model for the network simulator ns-3.

For more information visit http://vehicle2x.net.
However, this project is not affiliated with the original author of the model.

*NOTE: This is currently work in progress. Don't expect it to work (yet). Please come back later.*

## Status

### Implemented (but not fully tested):
* V2V links only
* Distinction between LOS, NLOSv (vehicles), NLOSf (foliage) and NLOSb (buildings) links
* LOS links based on two-ray ground
* NLOSv links based on the simple model (free space + attentuation based on obstructing vehicles)
* NLOSb links based on log-distance model
* Small scale variations based on the number of objects in the ellipse around sender and receiver

### Open:
* NLOSf links
* NLOSv with diffraction
* NLOSb reflection and diffraction model
* Import functions for building and foliage outlines
* Automated management of vehicles, e.g. through the  `MobilityModel` of a `Node`
* Tests, tests, tests, ...
* Optimization...


## Requirements

* ns-3 has to be compiled with at least C++11
* `boost/geometry` is used for all the geometric stuff  like rtrees, bounding box search and object/line intersections
