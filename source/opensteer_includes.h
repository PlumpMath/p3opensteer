/**
 * \file opensteer_includes.h
 *
 * \date 2016-05-13
 * \author consultit
 */

#ifndef OPENSTEER_INCLUDES_H_
#define OPENSTEER_INCLUDES_H_

#ifdef CPPPARSER
//Panda3d interrogate fake declarations

namespace OpenSteer
{
	struct AbstractObstacle;
	struct AbstractPlugIn;
	struct ObstacleGroup;
	struct AbstractVehicle;
}
namespace ossup
{
	struct VehicleSettings;
	template<typename T1, typename T2> struct VehicleAddOnMixin;
}

#endif //CPPPARSER

#endif /* OPENSTEER_INCLUDES_H_ */
