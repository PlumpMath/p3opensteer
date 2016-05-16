/**
 * \file common.h
 *
 * \date 2016-06-20
 * \author consultit
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <pandaFramework.h>
#include <auto_bind.h>
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
extern AsyncTask* updateTask;
extern bool toggleDebugFlag;
//models and animations
extern string vehicleFile[2];
extern string vehicleAnimFiles[2][2];
extern const float rateFactor[2];
//obstacle model
extern string obstacleFile;
//bam file
extern string bamFileName;
//support
extern random_device rd;

///functions' declarations
void startFramework(int argc, char *argv[]);
NodePath loadPlane();
NodePath loadTerrain();
PT(CollisionEntry)getCollisionEntryFromCamera();
void printCreationParameters();
void handleVehicleEvent(const Event*, void*);
void toggleDebugDraw(const Event*, void*);
void changeVehicleMaxSpeed(const Event*, void*);
void changeVehicleMaxForce(const Event*, void*);
LPoint3f getRandomPos(NodePath);
void getVehiclesModelsAnims(int, const NodePath&, NodePath vehicleNP[],
		PT(OSSteerPlugIn)steerPlugIn, PT(OSSteerVehicle)steerVehicle[],
		PT(AnimControl)vehicleAnimCtls[][2]);
bool readFromBamFile(string);
void writeToBamFileAndExit(const Event*, void*);

#endif /* COMMON_H_ */
