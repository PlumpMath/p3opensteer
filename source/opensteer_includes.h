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

namespace rnsup
{
	struct InputGeom;
	struct NavMeshType;
	struct NavMeshSettings;
	struct NavMeshTileSettings;
	struct BuildContext;
	struct NavMeshPolyAreaFlags;
	struct NavMeshPolyAreaCost;
	struct DebugDrawPanda3d;
	struct DebugDrawMeshDrawer;
}

struct dtNavMesh;
struct dtNavMeshQuery;
struct dtCrowd;
struct dtTileCache;
struct dtCrowdAgentParams;
typedef unsigned int dtObstacleRef;

#endif //CPPPARSER

#endif /* OPENSTEER_INCLUDES_H_ */
