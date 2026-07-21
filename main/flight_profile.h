#pragma once

#define FLIGHT_PROFILE_TUNE       0
#define FLIGHT_PROFILE_NORMAL     1
#define FLIGHT_PROFILE_TEST       2

#define FLIGHT_PROFILE_INDOOR_STABLE  10

////////////////////////////////////////////////////////////////////////////////////
// Flight profile selection
////////////////////////////////////////////////////////////////////////////////////

#ifndef FLIGHT_PROFILE
#define FLIGHT_PROFILE FLIGHT_PROFILE_INDOOR_STABLE
#endif