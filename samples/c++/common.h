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
#include <osSteerManager.h>
#include <osSteerPlugIn.h>
#include <osSteerVehicle.h>

extern string dataDir;

///global data
extern PandaFramework framework;
extern WindowFramework *window;
//models and animations
extern GeoMipTerrain* terrain;
extern LPoint3f terrainRootNetPos;

///functions' declarations
GeoMipTerrain* loadTerrain();
AsyncTask::DoneStatus terrainUpdate(GenericAsyncTask*, void*);

#endif /* COMMON_H_ */
