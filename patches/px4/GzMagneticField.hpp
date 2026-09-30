// SPDX-License-Identifier: Apache-2.0
// Body-vector conversion for explicitly configured Gazebo magnetometers.
#pragma once

#include <array>

namespace px4_gz
{
inline std::array<float, 3> magnetometer_frd(float x, float y, float z, bool enu_flu)
{

	if (enu_flu) {
		// ENU world field is rotated by gz-sensors into sensor/body FLU.
		// FLU -> FRD changes left/up to right/down; it does not swap X/Y.
		return {x, -y, -z};
	}

	// Preserve PX4's legacy Gazebo NED-field compatibility for other worlds.
	return {-y, -x, z};
}
}
