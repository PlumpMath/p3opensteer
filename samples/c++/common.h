/**
 * \file common.h
 *
 * \date 2016-06-20
 * \author consultit
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <random>
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
void startFramework(int argc, char *argv[], const string&);
NodePath loadPlane(const string&);
NodePath loadTerrain(const string&);
PT(CollisionEntry)getCollisionEntryFromCamera();
void printCreationParameters();
void handleVehicleEvent(const Event*, void*);
void toggleDebugDraw(const Event*, void*);
void changeVehicleMaxSpeed(const Event*, void*);
void changeVehicleMaxForce(const Event*, void*);
LPoint3f getRandomPos(NodePath);
void getVehicleModelAnims(float, int, const string&, const NodePath& ,
		PT(OSSteerPlugIn), vector<PT(OSSteerVehicle)>&,
		vector<vector<PT(AnimControl)> >&, const LPoint3f& pos = LPoint3f());
bool readFromBamFile(string);
void writeToBamFileAndExit(const Event*, void*);
//  data passed to obstacle's handling callback
struct HandleObstacleData
{
	HandleObstacleData(bool addObstacle, const NodePath& sceneNP,
			PT(OSSteerPlugIn)steerPlugIn):
	addObstacle(addObstacle), sceneNP(sceneNP), steerPlugIn(steerPlugIn)
	{
	}
	//
	bool addObstacle;
	NodePath sceneNP;
	PT(OSSteerPlugIn)steerPlugIn;
};
void handleObstacles(const Event*, void*);
// data passed to vehicle's handling callback
struct HandleVehicleData
{
	HandleVehicleData(float meanScale, int vehicleFileIdx,
			const string& moveType, const NodePath& sceneNP,
			PT(OSSteerPlugIn)steerPlugIn,
			vector<PT(OSSteerVehicle)>&steerVehicles,
			vector<vector<PT(AnimControl)> >& vehicleAnimCtls):
		meanScale(meanScale), vehicleFileIdx(vehicleFileIdx), moveType(moveType),
		sceneNP(sceneNP), steerPlugIn(steerPlugIn),
		steerVehicles(steerVehicles), vehicleAnimCtls(vehicleAnimCtls)
	{
	}
	//
	float meanScale;
	int vehicleFileIdx;
	string moveType;
	NodePath sceneNP;
	PT(OSSteerPlugIn) steerPlugIn;
	vector<PT(OSSteerVehicle)>& steerVehicles;
	vector<vector<PT(AnimControl)> >& vehicleAnimCtls;
};
void handleVehicles(const Event*, void*);

#endif /* COMMON_H_ */
