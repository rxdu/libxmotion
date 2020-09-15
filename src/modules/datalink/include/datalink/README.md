# Datalink

## LCMLink

* This module is to ensure a single-point dependency on the LCM library for the whole project
* Since the C++ wrapper is now copied over to the project, only the C library dependency of LCM is used
* Namespace is changed to "librav" instead of "lcm" to avoid a name conflict with CGAL (interanally with GNU MP library which includes a math function called lcm)

Changelog:

* Copied "lcm-cpp.hpp" and "lcm-cpp-impl.hpp" from the original lcm-proj
* Renamed namespace from "lcm" to "librav"
* Renamed LCM to LCMLink 