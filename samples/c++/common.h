/**
 * \file common.h
 *
 * \date 2016-06-20
 * \author consultit
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <pandaFramework.h>
#include <load_prc_file.h>
#include <geoMipTerrain.h>
#include <texturePool.h>
#include <mouseWatcher.h>
#include <cardMaker.h>
#include <osSteerManager.h>
#include <osSteerPlugIn.h>
#include <osSteerVehicle.h>

extern string dataDir;

///global data declaration
extern PandaFramework framework;
extern WindowFramework *window;
extern CollideMask mask;

///functions' declarations
NodePath loadPlane();
NodePath loadTerrain();
PT(CollisionEntry)getCollisionEntryFromCamera();

#endif /* COMMON_H_ */
