/**
 * \file osSteerPlugIn.cxx
 *
 * \date 2016-05-13
 * \author consultit
 */

#include "osSteerPlugIn.h"

#include "osSteerVehicle.h"
#include "osSteerManager.h"
#include "camera.h"

#ifndef CPPPARSER
#include "support/PlugIn_OneTurning.h"
#include "support/PlugIn_Pedestrian.h"
#include "support/PlugIn_Boids.h"
#include "support/PlugIn_MultiplePursuit.h"
#include "support/PlugIn_Soccer.h"
#include "support/PlugIn_CaptureTheFlag.h"
#include "support/PlugIn_LowSpeedTurn.h"
#include "support/PlugIn_MapDrive.h"
#endif //CPPPARSER

/**
 *
 */
OSSteerPlugIn::OSSteerPlugIn(const string& name) :
		PandaNode(name)
{
	do_reset();
}

/**
 *
 */
OSSteerPlugIn::~OSSteerPlugIn()
{
}

/**
 * Sets the OSSteerPlugIn type.
 * \note OSSteerPlugIn's type can only be changed if there are no attached
 * OSSteerVehicle(s).
 */
void OSSteerPlugIn::set_plug_in_type(OSSteerPlugInType type)
{
	CONTINUE_IF_ELSE_V(mSteerVehicles.size() == 0)

	//create the new OpenSteer plug-in
	do_create_plug_in(type);
	//(re)set pathway for the new OpenSteer plug-in
	set_pathway(mPathwayPoints, mPathwayRadii, mPathwaySingleRadius,
			mPathwayClosedCycle);
	//(re)set the new OpenSteer plug-in's local obstacles reference
	static_cast<ossup::PlugIn*>(mPlugIn)->localObstacles =
			&mLocalObstacles.first();
	//(re)set the new OpenSteer plug-in's global obstacles reference
	static_cast<ossup::PlugIn*>(mPlugIn)->obstacles =
			&OSSteerManager::get_global_ptr()->get_global_obstacles().first();
	//open the new OpenSteer plug-in
	mPlugIn->open();
}

/**
 * Initializes the OSSteerPlugIn with starting settings.
 * \note Internal use only.
 */
void OSSteerPlugIn::do_initialize()
{
	WPT(OSSteerManager)mTmpl = OSSteerManager::get_global_ptr();
	//
	//set OSSteerPlugIn parameters (store internally for future use)
	//type
	string mPlugInTypeParam = mTmpl->get_parameter_value(OSSteerManager::STEERPLUGIN,
			string("plugin_type"));
	//pathway (will be used on setup())
	string mPathwayParam = mTmpl->get_parameter_value(OSSteerManager::STEERPLUGIN,
			string("pathway"));
	//obstacles (will be used on setup())
	plist<string> mObstacleListParam = mTmpl->get_parameter_values(OSSteerManager::STEERPLUGIN,
			string("obstacles"));
	//
	//create the steer plug in
	if (mPlugInTypeParam == string("pedestrian"))
	{
		do_create_plug_in(PEDESTRIAN);
	}
	else if (mPlugInTypeParam == string("boid"))
	{
		do_create_plug_in(BOID);
	}
	else if (mPlugInTypeParam == string("multiple_pursuit"))
	{
		do_create_plug_in(MULTIPLE_PURSUIT);
	}
	else if (mPlugInTypeParam == string("soccer"))
	{
		do_create_plug_in(SOCCER);
	}
	else if (mPlugInTypeParam == string("capture_the_flag"))
	{
		do_create_plug_in(CAPTURE_THE_FLAG);
	}
	else if (mPlugInTypeParam == string("low_speed_turn"))
	{
		do_create_plug_in(LOW_SPEED_TURN);
	}
	else if (mPlugInTypeParam == string("map_drive"))
	{
		do_create_plug_in(MAP_DRIVE);
	}
	else
	{
		//default: "one_turning"
		do_create_plug_in(ONE_TURNING);
	}
	//build pathway
	do_build_pathway(mPathwayParam);
	//set the plugin local obstacles reference
	static_cast<ossup::PlugIn*>(mPlugIn)->localObstacles = &mLocalObstacles.first();
	//set the plugin global obstacles reference
	static_cast<ossup::PlugIn*>(mPlugIn)->obstacles = &mTmpl->get_global_obstacles().first();
	//add its own obstacles
	do_add_obstacles(mObstacleListParam);
	//open the steer plug in
	mPlugIn->open();
}

/**
 * Creates actually the steer plug-in.
 * \note Internal use only.
 */
void OSSteerPlugIn::do_create_plug_in(OSSteerPlugInType type)
{
	//remove current steer plug-in if any
	if (mPlugIn)
	{
		//close the current steer plug in
		mPlugIn->close();
		//delete the current steer plug in
		delete mPlugIn;
		mPlugIn = NULL;
	}
	//create the steer plug in
	mPlugInType = type;
	if (mPlugInType == PEDESTRIAN)
	{
		mPlugIn = new ossup::PedestrianPlugIn<OSSteerVehicle>;
	}
	else if (mPlugInType == BOID)
	{
		mPlugIn = new ossup::BoidsPlugIn<OSSteerVehicle>;
	}
	else if (mPlugInType == MULTIPLE_PURSUIT)
	{
		mPlugIn = new ossup::MpPlugIn<OSSteerVehicle>;
	}
	else if (mPlugInType == SOCCER)
	{
		mPlugIn = new ossup::MicTestPlugIn<OSSteerVehicle>;
	}
	else if (mPlugInType == CAPTURE_THE_FLAG)
	{
		mPlugIn = new ossup::CtfPlugIn<OSSteerVehicle>;
	}
	else if (mPlugInType == LOW_SPEED_TURN)
	{
		mPlugIn = new ossup::LowSpeedTurnPlugIn<OSSteerVehicle>;
	}
	else if (mPlugInType == MAP_DRIVE)
	{
		mPlugIn = new ossup::MapDrivePlugIn<OSSteerVehicle>;
	}
	else
	{
		//default: "one_turning"
		mPlugIn = new ossup::OneTurningPlugIn<OSSteerVehicle>;
		mPlugInType = ONE_TURNING;
	}
}

/**
 * Builds the pathway.
 * \note Internal use only.
 */
void OSSteerPlugIn::do_build_pathway(const string& pathwayParam)
{
	CONTINUE_IF_ELSE_V(!pathwayParam.empty())

	pvector<string> paramValues1Str, paramValues2Str, paramValues3Str;
	unsigned int idx, valueNum;
	//build pathway
	paramValues1Str = parseCompoundString(pathwayParam, '$');
	valueNum = paramValues1Str.size();
	if (valueNum != 3)
	{
		//there aren't all mandatory parameters: set hardcoded defaults
		paramValues1Str = parseCompoundString(
				string("0.0,0.0,0.0:1.0,1.0,1.0$1.0$false"), '$');
	}
	//pathway::points (forced to at least 2)
	paramValues2Str = parseCompoundString(paramValues1Str[0], ':');
	valueNum = paramValues2Str.size();
	if (valueNum == 0)
	{
		paramValues2Str = parseCompoundString(
				string("0.0,0.0,0.0:1.0,1.0,1.0"), ':');
	}
	else if (valueNum == 1)
	{
		paramValues2Str.push_back(string("1.0,1.0,1.0"));
	}
	unsigned int numPoints = paramValues2Str.size();
	OpenSteer::Vec3* points = new OpenSteer::Vec3[numPoints];
	for (idx = 0; idx < numPoints; ++idx)
	{
		paramValues3Str = parseCompoundString(paramValues2Str[idx], ',');
		if (paramValues3Str.size() < 3)
		{
			paramValues3Str.resize(3, "0.0");
		}
		LVector3f values;
		for (unsigned int i = 0; i < 3; ++i)
		{
			values[i] = strtof(paramValues3Str[i].c_str(), NULL);
		}
		points[idx] = ossup::LVecBase3fToOpenSteerVec3(values);
	}
	//get pathway::closedCycle
	bool closedCycle =
			(paramValues1Str[2] == string("true") ? true : false);
	//get pathway::radii (forced to at least 1) and set pathway
	paramValues2Str = parseCompoundString(paramValues1Str[1], ':');
	valueNum = paramValues2Str.size();
	if (valueNum == 0)
	{
		paramValues2Str.push_back(string("1.0"));
	}
	unsigned int numRadii = paramValues2Str.size();	//radii specified
	//point list
	ValueList<LPoint3f> pointList;
	for(unsigned int i=0; i<numPoints;++i )
	{
		pointList.add_value(ossup::OpenSteerVec3ToLVecBase3f(points[i]));
	}
	//radius list
	ValueList<float> radiusList;
	if (numRadii == 1)
	{
		//single radius
		float radius = strtof(paramValues2Str[0].c_str(), NULL);
		if (radius < 0.0)
		{
			radius = -radius;
		}
		else if (radius == 0.0)
		{
			radius = 1.0;
		}
		//set pathway: single radius
		radiusList.add_value(radius);
		set_pathway(pointList, radiusList, true, closedCycle);
	}
	else
	{
		//several radii
		unsigned int numRadiiAllocated = (
				closedCycle ? numPoints : numPoints - 1);
		float *radii = new float[numRadiiAllocated];
		for (idx = 0; idx < numRadiiAllocated; ++idx)
		{
			float value;
			if (idx < numRadii)
			{
				value = strtof(paramValues2Str[idx].c_str(), NULL);
				if (value < 0.0)
				{
					value = -value;
				}
				else if (value == 0.0)
				{
					value = 1.0;
				}
			}
			else
			{
				//radii allocated > radii specified
				value = 1.0;
			}
			radii[idx] = value;
		}
		//set pathway: several radius
		for(unsigned int i=0; i<numRadiiAllocated;++i )
		{
			radiusList.add_value(radii[i]);
		}
		set_pathway(pointList, radiusList, false, closedCycle);
		delete[] radii;
	}
	delete[] points;
}

/**
 * Adds the initial set of obstacles.
 * \note Obstacles' NodePaths are searched as descendants of the reference node
 * (and directly reparented to it if necessary).
 * \note Internal use only.
 */
void OSSteerPlugIn::do_add_obstacles(const plist<string>& obstacleListParam)
{
	CONTINUE_IF_ELSE_V(!obstacleListParam.empty())

	pvector<string> paramValues1Str, paramValues2Str;
	//add obstacles
	plist<string>::const_iterator iterList;
	for (iterList = obstacleListParam.begin();
			iterList != obstacleListParam.end(); ++iterList)
	{
		//any "obstacles" string is a "compound" one, i.e. could have the form:
		// "objectId1@shape1@seenFromState1:objectId2@shape2@seenFromState2:...:objectIdN@shapeN@seenFromStateN"
		paramValues1Str = parseCompoundString(*iterList, ':');
		pvector<string>::const_iterator iter;
		for (iter = paramValues1Str.begin(); iter != paramValues1Str.end();
				++iter)
		{
			//any obstacle string must have the form:
			//"objectId@shape@seenFromState"
			paramValues2Str = parseCompoundString(*iter, '@');
			if (paramValues2Str.size() < 3)
			{
				continue;
			}
			//get obstacle object: search reference node's descendants
			NodePath obstacleObject = mReferenceNP.find(
					string("**/" + paramValues2Str[0]));
			if (!obstacleObject.is_empty())
			{
				continue;
			}
			//add the obstacle
			add_obstacle(obstacleObject, paramValues2Str[1], paramValues2Str[2]);
		}
	}
}

/**
 * On destruction cleanup.
 * Gives an OSSteerPlugIn the ability to do any cleaning is necessary when
 * destroyed.
 * \note Internal use only.
 */
void OSSteerPlugIn::do_finalize()
{
	//disable debug drawing if enabled
	disable_debug_drawing();
	//remove all local obstacles from the global
	OpenSteer::ObstacleGroup::iterator iterLocal;
	OSSteerManager::GlobalObstacles& globalObstacles =
			OSSteerManager::get_global_ptr()->get_global_obstacles();
	for (iterLocal = mLocalObstacles.first().begin();
			iterLocal != mLocalObstacles.first().end(); ++iterLocal)
	{
		if(	globalObstacles.first().size()
						!= globalObstacles.second().size())
		{
			//VERY BAD!
			abort();
		}

		//find in global obstacles and remove it
		//1: remove the OpenSteer obstacle's pointer from the global list
		OpenSteer::ObstacleGroup::iterator iterO = find(
				globalObstacles.first().begin(), globalObstacles.first().end(),
				(*iterLocal));
		globalObstacles.first().erase(iterO);
		//2: remove the obstacle's attributes from the global list
		//NOTE: the i-th obstacle has pointer and attributes placed into the
		//i-th places of their respective lists.
		unsigned int pointerIdx = iterO - globalObstacles.first().begin();
		pvector<OSSteerManager::ObstacleAttributes>::iterator iterA =
				globalObstacles.second().begin() + pointerIdx;
		/*for (iterA = globalObstacles.second().begin();
				iterA != globalObstacles.second().end(); ++iterA)
		{
			if ((*iterA).first().get_obstacle() == (*iterLocal))
			{
				break;
			}
		}*/
		globalObstacles.second().erase(iterA);
		//3: deallocate the OpenSteer obstacle
		delete *iterLocal;
	}
	//remove all added OSSteerVehicle(s) (if any) from update
	PTA(PT(OSSteerVehicle))::const_iterator iter;
	for (iter = mSteerVehicles.begin(); iter != mSteerVehicles.end(); ++iter)
	{
		//set steerVehicle reference to null
		(*iter)->mSteerPlugIn.clear();
		//do remove from real update list
		static_cast<ossup::PlugIn*>(mPlugIn)->removeVehicle(
				&(*iter)->get_abstract_vehicle());
	}
	//close the steer plug in
	mPlugIn->close();
	//delete the steer plug in
	delete mPlugIn;
	do_reset();
}

typedef ossup::VehicleAddOnMixin<ossup::SimpleVehicle, OSSteerVehicle> VehicleAddOn;

/**
 * Adds a OSSteerVehicle to this OSSteerPlugIn (ie to the underlying OpenSteer
 * management mechanism).
 * Returns a negative value on error.
 */
int OSSteerPlugIn::add_steer_vehicle(NodePath steerVehicleNP)
{
	CONTINUE_IF_ELSE_R(
			(!steerVehicleNP.is_empty())
					&& (steerVehicleNP.node()->is_of_type(
							OSSteerVehicle::get_class_type())), OS_ERROR)

	PT(OSSteerVehicle)steerVehicle = DCAST(OSSteerVehicle, steerVehicleNP.node());

	// continue if steerVehicle doesn't belong to any steer plug-in
	CONTINUE_IF_ELSE_R(!steerVehicle->mSteerPlugIn, OS_ERROR)

	// continue if steerVehicle is compatible
	CONTINUE_IF_ELSE_R(check_steer_vehicle_compatibility(steerVehicleNP),
			OS_ERROR)

	bool result = false;
	//add to update list
	PTA(PT(OSSteerVehicle))::iterator iter;
	//check if OSSteerVehicle has not been already added
	iter = find(mSteerVehicles.begin(), mSteerVehicles.end(), steerVehicle);
	if (iter == mSteerVehicles.end())
	{
		LPoint3f pos;
		LVector3f forward, up;
		LVecBase3f modelDims;
		LVector3f modelDeltaCenter;
		float modelRadius;

		//OSSteerVehicle needs to be added
		//get the actual pos
		pos = steerVehicleNP.get_pos();
		forward = mReferenceNP.get_relative_vector(steerVehicleNP,
				LVector3f::forward());
		up = mReferenceNP.get_relative_vector(steerVehicleNP, LVector3f::up());
		//get steerVehicle dimensions
		modelRadius = OSSteerManager::get_global_ptr()->get_bounding_dimensions(
				steerVehicleNP, modelDims, modelDeltaCenter);
		//set orientation
		steerVehicleNP.heads_up(pos + forward, up);
		//set height correction
		steerVehicle->mHeigthCorrection = LVector3f(0.0, 0.0,
				modelDims.get_z());
		//update OpenSteer vehicle's settings
		OSVehicleSettings settings = steerVehicle->get_settings();
		settings.set_forward(forward);
		settings.set_up(up);
		settings.set_position(pos);
		//update radius
		settings.set_radius(modelRadius);
		//set actually OpenSteer vehicle's settings
		steerVehicle->set_settings(settings);
		//set steerVehicle reference to this steer plug-in
		steerVehicle->mSteerPlugIn = this;
		//add to the list of SteerVehicles
		mSteerVehicles.push_back(steerVehicle);
		//do add to real update list
		static_cast<ossup::PlugIn*>(mPlugIn)->addVehicle(
				&steerVehicle->get_abstract_vehicle());
		//
		result = true;
	}
	//
	return (result ? OS_SUCCESS : OS_ERROR);
}

/**
 * Removes a OSSteerVehicle from this OSSteerPlugIn (ie from the OpenSteer
 * handling mechanism).
 * Returns a negative value on error.
 */
int OSSteerPlugIn::remove_steer_vehicle(NodePath steerVehicleNP)
{
	CONTINUE_IF_ELSE_R(
			(!steerVehicleNP.is_empty())
					&& (steerVehicleNP.node()->is_of_type(
							OSSteerVehicle::get_class_type())), OS_ERROR)

	PT(OSSteerVehicle)steerVehicle = DCAST(OSSteerVehicle, steerVehicleNP.node());

	// continue if steerVehicle belongs to this steer plug-in
	CONTINUE_IF_ELSE_R(steerVehicle->mSteerPlugIn == this, OS_ERROR)

	bool result = false;
	//remove from the list of SteerVehicles
	PTA(PT(OSSteerVehicle))::iterator iter;
	//check if OSSteerVehicle has been already removed or not
	iter = find(mSteerVehicles.begin(), mSteerVehicles.end(), steerVehicle);
	if (iter != mSteerVehicles.end())
	{

		//set steerVehicle reference to null
		steerVehicle->mSteerPlugIn.clear();
		//do remove from real update list
		static_cast<ossup::PlugIn*>(mPlugIn)->removeVehicle(
				&steerVehicle->get_abstract_vehicle());
		//OSSteerVehicle needs to be removed
		mSteerVehicles.erase(iter);
		//
		result = true;
	}
	//
	return (result ? OS_SUCCESS : OS_ERROR);
}

/**
 * Checks if an OSSteerVehicle could be handled by this OSSteerPlugIn.
 */
bool OSSteerPlugIn::check_steer_vehicle_compatibility(
		NodePath steerVehicleNP) const
{
	CONTINUE_IF_ELSE_R(
			(!steerVehicleNP.is_empty())
					&& (steerVehicleNP.node()->is_of_type(
							OSSteerVehicle::get_class_type())), false)

	OSSteerVehicle::OSSteerVehicleType vehicleType = DCAST(OSSteerVehicle,
			steerVehicleNP.node())->get_vehicle_type();

	bool result = false;
	switch (mPlugInType)
	{
	case ONE_TURNING:
		if (vehicleType == OSSteerVehicle::ONE_TURNING)
		{
			result = true;
		}
		break;
	case PEDESTRIAN:
		if (vehicleType == OSSteerVehicle::PEDESTRIAN)
		{
			result = true;
		}
		break;
	case BOID:
		if (vehicleType == OSSteerVehicle::BOID)
		{
			result = true;
		}
		break;
	case MULTIPLE_PURSUIT:
		if ((vehicleType == OSSteerVehicle::MP_WANDERER)
				|| (vehicleType == OSSteerVehicle::MP_PURSUER))
		{
			result = true;
		}
		break;
	case SOCCER:
		if ((vehicleType == OSSteerVehicle::PLAYER)
				|| (vehicleType == OSSteerVehicle::BALL))
		{
			result = true;
		}
		break;
	case CAPTURE_THE_FLAG:
		if ((vehicleType == OSSteerVehicle::CTF_SEEKER)
				|| (vehicleType == OSSteerVehicle::CTF_ENEMY))
		{
			result = true;
		}
		break;
	case LOW_SPEED_TURN:
		if (vehicleType == OSSteerVehicle::LOW_SPEED_TURN)
		{
			result = true;
		}
		break;
	case MAP_DRIVE:
		if (vehicleType == OSSteerVehicle::MAP_DRIVER)
		{
			result = true;
		}
		break;
	default:
		break;
	}
	//
	return result;
}

/**
 * Sets the pathway of this OSSteerPlugin.
 * \note pointList and radiusList should have the same number of elements; in
 * any case, the number of segments is equal to the number of points if the
 * cycle is closed otherwise to the number of points - 1.
 * \note PEDESTRIAN OSSteerPlugIn supports currently only single radius
 * pathway.
 */
void OSSteerPlugIn::set_pathway(const ValueList<LPoint3f>& pointList,
		const ValueList<float>& radiusList, bool singleRadius, bool closedCycle)
{
	unsigned long int numOfPoints = (unsigned long int) pointList.size();
	//create vectors and convert to OpenSteer points
	OpenSteer::Vec3* osPoints = new OpenSteer::Vec3[numOfPoints];
	float* radii = new float[numOfPoints];
	int numOfRadii = radiusList.size();
	float lastRadius = 1.0;
	for (unsigned int idx = 0; idx < numOfPoints; ++idx)
	{
		osPoints[idx] = ossup::LVecBase3fToOpenSteerVec3(pointList[idx]);
		if (numOfRadii < (int) (idx + 1))
		{
			radii[idx] = lastRadius;
		}
		else
		{
			radii[idx] = radiusList[idx];
			lastRadius = radiusList[idx];
		}
	}
	//exception for PEDESTRIAN type: only single radius
	if (mPlugInType == PEDESTRIAN)
	{
		singleRadius = true;
	}
	//set the actual path
	static_cast<ossup::PlugIn*>(mPlugIn)->setPathway(numOfPoints, osPoints,
			singleRadius, radii, closedCycle);
	//
	delete[] osPoints;
	delete[] radii;
	//save pathway's stuff
	mPathwayPoints = pointList;
	mPathwayRadii = radiusList;
	mPathwaySingleRadius = singleRadius;
	mPathwayClosedCycle = closedCycle;
#ifdef OS_DEBUG
	do_debug_draw_static_geometry();
#endif //OS_DEBUG
}

/**
 * Adds an obstacle given a non empty NodePath (objectNP), which will correspond
 * to the underlying OpenSteer obstacle, and is directly reparented to the
 * reference node. In this case parameters are extracted from the NodePath.
 * objectNP, type and seenFromState parameters must be always specified.\n
 * Returns the obstacle's unique reference (>0), or a negative value on error.
 */
int OSSteerPlugIn::add_obstacle(NodePath& objectNP,
		const string& type, const string& seenFromState)
{
	CONTINUE_IF_ELSE_R(!objectNP.is_empty(), OS_ERROR)

	LPoint3f position;
	LVector3f side, up, forward;
	float width, height, depth, radius;
	//get obstacle dimensions
	LVecBase3f modelDims;
	LVector3f modelDeltaCenter;
	float modelRadius;
	//compute new obstacle dimensions
	modelRadius = OSSteerManager::get_global_ptr()->get_bounding_dimensions(
			objectNP, modelDims, modelDeltaCenter);
	//correct obstacle's parameters
	position = objectNP.get_pos();
	forward = mReferenceNP.get_relative_vector(objectNP, LVector3f::forward());
	up = mReferenceNP.get_relative_vector(objectNP, LVector3f::up());
	side = mReferenceNP.get_relative_vector(objectNP, LVector3f::right());
	width = modelDims.get_x();
	height = modelDims.get_z();
	depth = modelDims.get_y();
	radius = modelRadius;
	//
	return do_add_obstacle(objectNP, type, seenFromState, width, height, depth,
			radius, side, up, forward, position + LPoint3f(0.0, 0.0, height / 2.0));
}

/**
 * Adds actually the obstacle.
 * \note Internal use only.
 */
int OSSteerPlugIn::do_add_obstacle(NodePath objectNP,
		const string& type, const string& seenFromState,
		float width, float height,	float depth, float radius,
		const LVector3f& side, const LVector3f& up,
		const LVector3f& forward, const LPoint3f& position)
{
	//set seen from state
	OpenSteer::AbstractObstacle::seenFromState seenFS;
	if (seenFromState == "outside")
	{
		seenFS = OpenSteer::AbstractObstacle::outside;
	}
	else if (seenFromState == "inside")
	{
		seenFS = OpenSteer::AbstractObstacle::inside;
	}
	else
	{
		//default: both
		seenFS = OpenSteer::AbstractObstacle::both;
	}
	//create actually the obstacle
	OpenSteer::AbstractObstacle* obstacle = NULL;
	if (type == string("box"))
	{
		ossup::BoxObstacle* box = new ossup::BoxObstacle(width, height, depth);
		obstacle = box;
		box->setSide(ossup::LVecBase3fToOpenSteerVec3(side).normalize());
		box->setUp(ossup::LVecBase3fToOpenSteerVec3(up).normalize());
		box->setForward(ossup::LVecBase3fToOpenSteerVec3(forward).normalize());
		box->setPosition(ossup::LVecBase3fToOpenSteerVec3(position));
		obstacle->setSeenFrom(seenFS);
	}
	if (type == string("plane"))
	{
		obstacle = new ossup::PlaneObstacle(
				ossup::LVecBase3fToOpenSteerVec3(side).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(up).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(forward).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(position));
		obstacle->setSeenFrom(seenFS);
	}
	if (type == string("rectangle"))
	{
		obstacle = new ossup::RectangleObstacle(width, height,
				ossup::LVecBase3fToOpenSteerVec3(side).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(up).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(forward).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(position), seenFS);
	}
	if (type == string("sphere"))
	{
		obstacle = new ossup::SphereObstacle(radius,
				ossup::LVecBase3fToOpenSteerVec3(position));
		obstacle->setSeenFrom(seenFS);
	}
	//store obstacle and all settings
	int ref = OS_ERROR;
	if (obstacle)
	{
		OSSteerManager::GlobalObstacles& globalObstacles =
				OSSteerManager::get_global_ptr()->get_global_obstacles();
		nassertr_always(
				globalObstacles.first().size()
						== globalObstacles.second().size(), OS_ERROR);

		//1: set obstacle's settings
		OSObstacleSettings settings;
		settings.set_type(type);
		settings.set_seenFromState(seenFromState);
		settings.set_position(position);
		settings.set_forward(forward);
		settings.set_up(up);
		settings.set_side(side);
		settings.set_width(width);
		settings.set_height(height);
		settings.set_depth(depth);
		settings.set_radius(radius);
		ref = OSSteerManager::get_global_ptr()->unique_ref();
		settings.set_ref(ref);
		settings.set_obstacle(obstacle);
		//2: add OpenSteer obstacle's pointer to global list
		globalObstacles.first().push_back(obstacle);
		//3: add obstacle's attributes to global list
		OSSteerManager::ObstacleAttributes obstacleAttrs(settings, objectNP);
		globalObstacles.second().push_back(obstacleAttrs);
		//4: add OpenSteer obstacle's pointer to local list
		mLocalObstacles.first().push_back(obstacle);
		//5: add obstacle's attributes to local list
		mLocalObstacles.second().push_back(obstacleAttrs);
		//6: reparent to reference node if objectNP is not empty
		if (!objectNP.is_empty())
		{
			objectNP.reparent_to(mReferenceNP);
		}
	}
#ifdef OS_DEBUG
	do_debug_draw_static_geometry();
#endif //OS_DEBUG
	return ref;
}

/**
 * Removes an obstacle given its unique ref (>0).\n
 * Returns the NodePath (possibly empty) that was associated to the underlying
 * obstacle just removed, otherwise an empty NodePath with the ET_fail error
 * type set on error.
 * \note Obstacle will be removed only if it was added by this OSSteerPlugIn.
 */
NodePath OSSteerPlugIn::remove_obstacle(int ref)
{
	NodePath resultNP = NodePath::fail();
	//find in global obstacles
	//get a reference to the global storage
	OSSteerManager::GlobalObstacles& globalObstacles =
			OSSteerManager::get_global_ptr()->get_global_obstacles();
	//find the Obstacle's attributes with the given ref, if any
	pvector<OSSteerManager::ObstacleAttributes>::iterator iterA;
	for (iterA = globalObstacles.second().begin();
			iterA != globalObstacles.second().end(); ++iterA)
	{
		if ((*iterA).first().get_ref() == ref)
		{
			break;
		}
	}
	//continue only if ref is found
	CONTINUE_IF_ELSE_R(iterA != globalObstacles.second().end(), resultNP)

	//remove only if OpenSteer obstacle is in the local list (ie has been
	//added by this OSSteerPlugIn)
	OpenSteer::ObstacleGroup::iterator iterOL = find(
			mLocalObstacles.first().begin(), mLocalObstacles.first().end(),
			(*iterA).first().get_obstacle());
	if (iterOL != mLocalObstacles.first().end())
	{
		//it is in the local list (ie added by this OSSteerPlugIn)
		nassertr_always(
				globalObstacles.first().size()
						== globalObstacles.second().size(), NodePath::fail());

		//get the node path
		resultNP = (*iterA).second();
		//1: remove the OpenSteer obstacle's pointer from the global list
		//NOTE: the i-th obstacle has pointer and attributes placed into the
		//i-th places of their respective lists.
		unsigned int attributesIdx = iterA - globalObstacles.second().begin();
		OpenSteer::ObstacleGroup::iterator iterO =
				globalObstacles.first().begin() + attributesIdx;
		/*= find(globalObstacles.first().begin(), globalObstacles.first().end(),
		 (*iterA).first().get_obstacle());*/
		globalObstacles.first().erase(iterO);
		//2: remove the obstacle's attributes from the global list
		globalObstacles.second().erase(iterA);
		//3: deallocate the OpenSteer obstacle
		delete *iterOL;
		//4: remove the OpenSteer obstacle's pointer from the local list
		nassertr_always(
				mLocalObstacles.first().size()
						== mLocalObstacles.second().size(), NodePath::fail());

		mLocalObstacles.first().erase(iterOL);
		//5: remove the obstacle's attributes from the local list
		//NOTE: the i-th obstacle has pointer and attributes placed into the
		//i-th places of their respective lists.
		unsigned int pointerIdx = iterOL - mLocalObstacles.first().begin();
		pvector<OSSteerManager::ObstacleAttributes>::iterator iterAL =
				mLocalObstacles.second().begin() + pointerIdx;
		mLocalObstacles.second().erase(iterAL);
	}
#ifdef OS_DEBUG
	do_debug_draw_static_geometry();
#endif //OS_DEBUG
	//
	return resultNP;
}

/**
 * Updates the underlying OpenSteer plug-in.
 * It allows the added OSSteerVehicle(s) to perform their "steering behaviors".
 */
void OSSteerPlugIn::update(float dt)
{
	//currentTime
	mCurrentTime += dt;

#ifdef TESTING
	dt = 0.016666667; //60 fps
#endif

#ifdef OS_DEBUG
	{
		if (mEnableDebugDrawUpdate && mDrawer3d && mDrawer2d)
		{
			//set enableAnnotation
			ossup::enableAnnotation = true;

			//drawers' initializations
			mDrawer3d->initialize();
			mDrawer2d->initialize();

			/// invoke PlugIn's Update method
			mPlugIn->update(mCurrentTime, dt);

			// invoke selected PlugIn's Redraw method
			mPlugIn->redraw(mCurrentTime, dt);

			// draw any annotation queued up during selected PlugIn's Update method
			OpenSteer::drawAllDeferredLines();
			OpenSteer::drawAllDeferredCirclesOrDisks();

			//drawers' finalizations
			mDrawer3d->finalize();
			mDrawer2d->finalize();
		}
		else
		{
			//clear enableAnnotation
			ossup::enableAnnotation = false;
#endif //OS_DEBUG

			/// invoke PlugIn's Update method
			mPlugIn->update(mCurrentTime, dt);

#ifdef OS_DEBUG
		}
	}
#endif //OS_DEBUG
}

/**
 * Sets the type of proximity database:
 * - BruteForceProximityDatabase
 * - LQProximityDatabase (default).
 * \note PEDESTRIAN, BOID OSSteerPlugIn only.
 */
void OSSteerPlugIn::set_proximity_database(OSProximityDatabase pd)
{
	if (mPlugInType == PEDESTRIAN)
	{
		ossup::PedestrianPlugIn<OSSteerVehicle>* plugIn =
				static_cast<ossup::PedestrianPlugIn<OSSteerVehicle>*>(mPlugIn);
		switch (pd)
		{
		case LQ_PD:
			plugIn->setPD(0);
			break;
		case BRUTEFORCE_PD:
			plugIn->setPD(1);
			break;
		default:
			break;
		}
	}
	if (mPlugInType == BOID)
	{
		ossup::BoidsPlugIn<OSSteerVehicle>* plugIn =
				static_cast<ossup::BoidsPlugIn<OSSteerVehicle>*>(mPlugIn);
		switch (pd)
		{
		case LQ_PD:
			plugIn->setPD(0);
			break;
		case BRUTEFORCE_PD:
			plugIn->setPD(1);
			break;
		default:
			break;
		}
	}
}

/**
 * Returns the type of proximity database
 * - BruteForceProximityDatabase
 * - LQProximityDatabase (default)
 * or a negative value on error.
 * \note PEDESTRIAN, BOID OSSteerPlugIn only.
 */
OSSteerPlugIn::OSProximityDatabase OSSteerPlugIn::get_proximity_database() const
{
	if (mPlugInType == PEDESTRIAN)
	{
		ossup::PedestrianPlugIn<OSSteerVehicle>* plugIn =
				static_cast<ossup::PedestrianPlugIn<OSSteerVehicle>*>(mPlugIn);
		switch (plugIn->pdIdx)
		{
		case 0:
			return LQ_PD;
			break;
		case 1:
			return BRUTEFORCE_PD;
			break;
		default:
			break;
		}
	}
	if (mPlugInType == BOID)
	{
		ossup::BoidsPlugIn<OSSteerVehicle>* plugIn =
				static_cast<ossup::BoidsPlugIn<OSSteerVehicle>*>(mPlugIn);
		switch (plugIn->pdIdx)
		{
		case 0:
			return LQ_PD;
			break;
		case 1:
			return BRUTEFORCE_PD;
			break;
		default:
			break;
		}
	}
	return (OSProximityDatabase) OS_ERROR;
}

/**
 * Sets the world center point.
 * \note BOID OSSteerPlugIn only.
 */
void OSSteerPlugIn::set_world_center(const LPoint3f& center)
{
	if (mPlugInType == BOID)
	{
		ossup::BoidsPlugIn<OSSteerVehicle>* plugIn =
				static_cast<ossup::BoidsPlugIn<OSSteerVehicle>*>(mPlugIn);
		plugIn->worldCenter = ossup::LVecBase3fToOpenSteerVec3(center);
	}
}

/**
 * Returns the world center point.
 * Returns LPoint3f::zero() on error.
 * \note BOID OSSteerPlugIn only.
 */
LPoint3f OSSteerPlugIn::get_world_center() const
{
	if (mPlugInType == BOID)
	{
		ossup::BoidsPlugIn<OSSteerVehicle>* plugIn =
				static_cast<ossup::BoidsPlugIn<OSSteerVehicle>*>(mPlugIn);
		return ossup::OpenSteerVec3ToLVecBase3f(plugIn->worldCenter);
	}
	return LPoint3f::zero();
}


/**
 * Sets the world radius.
 * \note BOID OSSteerPlugIn only.
 */
void OSSteerPlugIn::set_world_radius(float radius)
{
	if (mPlugInType == BOID)
	{
		ossup::BoidsPlugIn<OSSteerVehicle>* plugIn =
				static_cast<ossup::BoidsPlugIn<OSSteerVehicle>*>(mPlugIn);
		plugIn->worldRadius = radius;
	}
}

/**
 * Returns the world radius.
 * Returns a negative on error.
 * \note BOID OSSteerPlugIn only.
 */
float OSSteerPlugIn::get_world_radius() const
{
	if (mPlugInType == BOID)
	{
		ossup::BoidsPlugIn<OSSteerVehicle>* plugIn =
				static_cast<ossup::BoidsPlugIn<OSSteerVehicle>*>(mPlugIn);
		return plugIn->worldRadius;
	}
	return OS_ERROR;
}

/**
 * Adds a player (OSSteerVehicle) to one of two teams: teamA (= true) or teamB
 * (= false).
 * Returns a negative value on error.
 * \note SOCCER OSSteerPlugIn only.
 */
int OSSteerPlugIn::add_player_to_team(PT(OSSteerVehicle) player,
		OSPlayingTeam team)
{
	if ((mPlugInType == SOCCER) &&
			(player->get_vehicle_type() == OSSteerVehicle::PLAYER) &&
			(team != NO_TEAM))
	{
		ossup::MicTestPlugIn<OSSteerVehicle>* plugIn =
				static_cast<ossup::MicTestPlugIn<OSSteerVehicle>*>(mPlugIn);
		plugIn->addPlayerToTeam(
				&(static_cast<ossup::Player<OSSteerVehicle>&>(
						player->get_abstract_vehicle())), team == TEAM_A);
		//update internal reference
		player->mPlayingTeam_ser = team;
		return OS_SUCCESS;
	}
	return OS_ERROR;
}

/**
 * Removes a player (OSSteerVehicle) from his/her current team: teamA or teamB.
 * Returns a negative value on error.
 * \note SOCCER OSSteerPlugIn only.
 */
int OSSteerPlugIn::remove_player_from_team(PT(OSSteerVehicle) player)
{
	if (mPlugInType == SOCCER &&
			player->get_vehicle_type() == OSSteerVehicle::PLAYER)
	{
		ossup::MicTestPlugIn<OSSteerVehicle>* plugIn =
				static_cast<ossup::MicTestPlugIn<OSSteerVehicle>*>(mPlugIn);
		plugIn->removePlayerFromTeam(
				&(static_cast<ossup::Player<OSSteerVehicle>&>(
						player->get_abstract_vehicle())));
		//update internal reference
		player->mPlayingTeam_ser = NO_TEAM;
		return OS_SUCCESS;
	}
	return OS_ERROR;
}

/**
 * Returns the two extreme points, with respect to reference node, of a playing
 * field.
 * \note SOCCER OSSteerPlugIn only.
 */
ValueList<LPoint3f> OSSteerPlugIn::get_playing_field() const
{
	ValueList<LPoint3f> points;
	if (mPlugInType == SOCCER)
	{
		ossup::MicTestPlugIn<OSSteerVehicle>* plugIn =
				static_cast<ossup::MicTestPlugIn<OSSteerVehicle>*>(mPlugIn);
		points.add_value(
				ossup::OpenSteerVec3ToLVecBase3f(plugIn->m_bbox->getMin()));
		points.add_value(
				ossup::OpenSteerVec3ToLVecBase3f(plugIn->m_bbox->getMax()));
	}
	return points;
}

/**
 * Sets a playing field, given two extreme points with respect to reference
 * node and the goal fraction.
 * \note The field will be planar and axis aligned (xy plane), and placed at
 * medium z-height of the two points; the goal fraction is specified with
 * respect to the field's y-dimension. By default a field with dimensions 40x20
 * and placed at (0,0,0) is created, and minimum field's dimensions are 40x20
 * anyway.
 * \note SOCCER OSSteerPlugIn only.
 */
void OSSteerPlugIn::set_playing_field(const LPoint3f& min, const LPoint3f& max,
		float goalFraction)
{
	if (mPlugInType == SOCCER)
	{
		ossup::MicTestPlugIn<OSSteerVehicle>* plugIn =
				static_cast<ossup::MicTestPlugIn<OSSteerVehicle>*>(mPlugIn);
		plugIn->setSoccerField(ossup::LVecBase3fToOpenSteerVec3(min),
				ossup::LVecBase3fToOpenSteerVec3(max), goalFraction);
	}
}

/**
 * Returns the goal fraction, with respect to the field's y-dimension, of a
 * playing field, or a negative value on error.
 * \note SOCCER OSSteerPlugIn only.
 */
float OSSteerPlugIn::get_goal_fraction() const
{
	float fraction = OS_ERROR;
	if (mPlugInType == SOCCER)
	{
		ossup::MicTestPlugIn<OSSteerVehicle>* plugIn =
				static_cast<ossup::MicTestPlugIn<OSSteerVehicle>*>(mPlugIn);
		float fieldYdim = abs(
				plugIn->m_bbox->getMax().z - plugIn->m_bbox->getMin().z);
		float goalYdim = abs(
				plugIn->m_TeamAGoal->getMax().z
						- plugIn->m_TeamAGoal->getMin().z);
		fraction = goalYdim / fieldYdim;
	}
	return fraction;
}

/**
 * Sets steering speed (>=0).
 * \note LOW_SPEED_TURN OSSteerPlugIn only.
 */
void OSSteerPlugIn::set_steering_speed(float steeringSpeed)
{
	if (mPlugInType == LOW_SPEED_TURN)
	{
		static_cast<ossup::LowSpeedTurnPlugIn<OSSteerVehicle>*>(mPlugIn)->steeringSpeed =
				(steeringSpeed >= 0 ? steeringSpeed : -steeringSpeed);
	}
}

/**
 * Returns steering speed (>=0), a negative value on error.
 * \note LOW_SPEED_TURN OSSteerPlugIn only.
 */
float OSSteerPlugIn::get_steering_speed() const
{
	return (mPlugInType == LOW_SPEED_TURN) ?
			static_cast<ossup::LowSpeedTurnPlugIn<OSSteerVehicle>*>(mPlugIn)->steeringSpeed :
			OS_ERROR;
}

/**
 * Writes a sensible description of the OSSteerPlugIn to the indicated output
 * stream.
 */
void OSSteerPlugIn::output(ostream &out) const
{
	out << get_type() << " " << get_name();
}

/**
 * Enables the debug drawing.
 * A camera node path should be passed as argument.
 */
void OSSteerPlugIn::enable_debug_drawing(NodePath debugCamera)
{
#ifdef OS_DEBUG
	CONTINUE_IF_ELSE_V(mDebugCamera.is_empty())

	if ((!debugCamera.is_empty()) &&
			(!debugCamera.find(string("**/+Camera")).is_empty()))
	{
		//set the debug camera
		mDebugCamera = debugCamera;
		//set the debug node as child of mReferenceDebugNP node
		mDrawer3dNP = mReferenceDebugNP.attach_new_node(
				string("OpenSteerDebugNodePath_") + get_name());
		mDrawer3dStaticNP = mReferenceDebugNP.attach_new_node(
				string("OpenSteerDebugNodePathStatic_") + get_name());
		//set the 2D debug node as child of mReferenceDebug2DNP node
		mDrawer2dNP = mReferenceDebug2DNP.attach_new_node(
				string("OpenSteerDebugNodePath2D_") + get_name());
		//set debug node paths
		mDrawer3dNP.set_bin("fixed", 10);
		mDrawer3dStaticNP.set_bin("fixed", 10);
		mDrawer2dNP.set_bin("fixed", 10);
		//by default Debug NodePaths are hidden
		mDrawer3dNP.hide();
		mDrawer3dStaticNP.hide();
		mDrawer2dNP.hide();
		//no collide mask for all Debug NodePaths' children
		mDrawer3dNP.set_collide_mask(BitMask32::all_off());
		mDrawer3dStaticNP.set_collide_mask(BitMask32::all_off());
		mDrawer2dNP.set_collide_mask(BitMask32::all_off());
		//create new Debug Drawers
		NodePath meshDrawerCamera = mDebugCamera;
		if (!mDebugCamera.node()->is_of_type(Camera::get_class_type()))
		{
			meshDrawerCamera = mDebugCamera.find(string("**/+Camera"));
		}
		mDrawer3d = new ossup::DrawMeshDrawer(mDrawer3dNP, meshDrawerCamera,
				100, 0.04);
		mDrawer3dStatic = new ossup::DrawMeshDrawer(mDrawer3dStaticNP,
				meshDrawerCamera, 100, 0.04);
		mDrawer3dStatic->setSize(3.0);
		mDrawer2d = new ossup::DrawMeshDrawer(mDrawer2dNP, meshDrawerCamera, 50,
				0.04);
	}
#endif //OS_DEBUG
}

/**
 * Disables the debug drawing.
 */
void OSSteerPlugIn::disable_debug_drawing()
{
#ifdef OS_DEBUG
	if (! mDebugCamera.is_empty())
	{
		//set the opensteer debug camera to empty node path
		mDebugCamera = NodePath();
		//remove the opensteer debug node paths
		mDrawer3dNP.remove_node();
		mDrawer3dStaticNP.remove_node();
		mDrawer2dNP.remove_node();
		//reset the DebugDrawers
		if (mDrawer3d)
		{
			delete mDrawer3d;
			mDrawer3d = NULL;
		}
		if (mDrawer3dStatic)
		{
			delete mDrawer3dStatic;
			mDrawer3dStatic = NULL;
		}
		if (mDrawer2d)
		{
			delete mDrawer2d;
			mDrawer2d = NULL;
		}
	}
#endif //OS_DEBUG
}

/**
 * Enables/disables debugging.
 * Returns a negative value on error.
 */
int OSSteerPlugIn::toggle_debug_drawing(bool enable)
{
#ifdef OS_DEBUG
	//continue if mDebugCamera, mDrawer3dNP, mDrawer3dStaticNP and mDrawer2dNP are not empty
	CONTINUE_IF_ELSE_R(
			(!mDebugCamera.is_empty())
					&& ((!mDrawer3dNP.is_empty())
							&& (!mDrawer3dStaticNP.is_empty())
							&& (!mDrawer2dNP.is_empty())), OS_ERROR)

	if (enable)
	{
		if (mDrawer3dNP.is_hidden())
		{
			//clear drawer
			mDrawer3d->clear();
			mDrawer3dNP.show();
			//set Debug Draw Update
			mEnableDebugDrawUpdate = true;
			//set drawer
			gDrawer3d = mDrawer3d;
		}
		if (mDrawer3dStaticNP.is_hidden())
		{
			//clear drawer
			mDrawer3dStatic->clear();
			mDrawer3dStaticNP.show();
			//draw static geometry
			do_debug_draw_static_geometry();
		}
		if (mDrawer2dNP.is_hidden())
		{
			//clear drawer
			mDrawer2d->clear();
			mDrawer2dNP.show();
			//set Debug Draw Update
			mEnableDebugDrawUpdate = true;
			//set drawer
			gDrawer2d = mDrawer2d;
		}
	}
	else
	{
		if (!mDrawer3dNP.is_hidden())
		{
			//clear drawer
			mDrawer3d->clear();
			mDrawer3dNP.hide();
			//set Debug Draw Update
			mEnableDebugDrawUpdate = false;
			//set drawer
			gDrawer3d = NULL;
		}
		if (!mDrawer3dStaticNP.is_hidden())
		{
			//clear drawer
			mDrawer3dStatic->clear();
			mDrawer3dStaticNP.hide();
			//set drawer
			gDrawer3d = NULL;
		}
		if (!mDrawer2dNP.is_hidden())
		{
			//clear drawer
			mDrawer2d->clear();
			mDrawer2dNP.hide();
			//set Debug Draw Update
			mEnableDebugDrawUpdate = false;
			//set drawer
			gDrawer2d = NULL;
		}
	}
	//
#endif //OS_DEBUG
	return OS_SUCCESS;
}

#ifdef OS_DEBUG
/**
 * Draws static geometry.
 * \note Internal use only.
 */
void OSSteerPlugIn::do_debug_draw_static_geometry()
{
	//continue if mDrawer3dStaticNP is not empty
	CONTINUE_IF_ELSE_V(
			(!mDebugCamera.is_empty()) && (!mDrawer3dStaticNP.is_empty()))

	//set drawer
	ossup::DrawMeshDrawer * currentDrawer = gDrawer3d;
	gDrawer3d = mDrawer3dStatic;

	//drawers' initializations
	mDrawer3dStatic->initialize();

	//draw static geometry
	//ctf: render home base and obstacles
	if (mPlugInType == CAPTURE_THE_FLAG)
	{
		ossup::CtfPlugIn<OSSteerVehicle>* plugIn = static_cast<ossup::CtfPlugIn<
				OSSteerVehicle>*>(mPlugIn);
		plugIn->drawHomeBase();
		plugIn->drawObstacles();
	}
	//map drive: render path and map
	if (mPlugInType == MAP_DRIVE)
	{
		ossup::MapDrivePlugIn<OSSteerVehicle>* plugIn =
				static_cast<ossup::MapDrivePlugIn<OSSteerVehicle>*>(mPlugIn);
		plugIn->drawMap();
		plugIn->drawPath();
	}
	//pedestrian: render path and obstacles
	if (mPlugInType == PEDESTRIAN)
	{
		ossup::PedestrianPlugIn<OSSteerVehicle>* plugIn =
				static_cast<ossup::PedestrianPlugIn<OSSteerVehicle>*>(mPlugIn);
		plugIn->drawPath();
		plugIn->drawObstacles();
	}
	//soccer: render path
	if (mPlugInType == SOCCER)
	{
		static_cast<ossup::MicTestPlugIn<OSSteerVehicle>*>(mPlugIn)->drawSoccerField();
	}
	//boid: render obstacles
	if (mPlugInType == BOID)
	{
		static_cast<ossup::BoidsPlugIn<OSSteerVehicle>*>(mPlugIn)->drawObstacles();
	}

	//drawer finalizations
	mDrawer3dStatic->finalize();

	//(re)set drawer
	gDrawer3d = currentDrawer;
}
#endif //OS_DEBUG

//TypedWritable API
/**
 * Tells the BamReader how to create objects of type OSSteerPlugIn.
 */
void OSSteerPlugIn::register_with_read_factory()
{
	BamReader::get_factory()->register_factory(get_class_type(), make_from_bam);
}

/**
 * Writes the contents of this object to the datagram for shipping out to a
 * Bam file.
 */
void OSSteerPlugIn::write_datagram(BamWriter *manager, Datagram &dg)
{
	PandaNode::write_datagram(manager, dg);

	///Name of this OSSteerPlugIn.
	dg.add_string(get_name());

	///The type of this OSSteerPlugIn.
	dg.add_uint8((uint8_t) mPlugInType);

	///Pathway stuff.
	///@{
	dg.add_int32(mPathwayPoints.size());
	for (int i = 0; i < mPathwayPoints.size(); ++i)
	{
		mPathwayPoints[i].write_datagram(dg);
	}
	dg.add_int32(mPathwayRadii.size());
	for (int i = 0; i < mPathwayRadii.size(); ++i)
	{
		dg.add_stdfloat(mPathwayRadii[i]);
	}
	dg.add_bool(mPathwaySingleRadius);
	dg.add_bool(mPathwayClosedCycle);
	///@}

	/// Pointers
	///The reference node path.
	manager->write_pointer(dg, mReferenceNP.node());

	///Steer vehicles.
	dg.add_uint32(mSteerVehicles.size());
	{
		pvector<PT(OSSteerVehicle)>::iterator iter;
		for (iter = mSteerVehicles.begin(); iter != mSteerVehicles.end(); ++iter)
		{
			manager->write_pointer(dg, (*iter));
		}
	}

	///The "local" obstacles handled by this OSSteerPlugIn.
	dg.add_uint32(mLocalObstacles.first().size());
	{
		pvector<OSSteerManager::ObstacleAttributes>::iterator iter;
		for (iter = mLocalObstacles.second().begin();
				iter != mLocalObstacles.second().end(); ++iter)
		{
			(*iter).first().write_datagram(dg);
			manager->write_pointer(dg, (*iter).second().node());
		}
	}

	///SPECIFICS
	if(mPlugInType == ONE_TURNING)
	{
		/*do nothing*/;
	}
	if(mPlugInType == PEDESTRIAN)
	{
		dg.add_uint8((uint8_t) get_proximity_database());
	}
	if(mPlugInType == BOID)
	{
		dg.add_uint8((uint8_t) get_proximity_database());
		get_world_center().write_datagram(dg);
		dg.add_stdfloat(get_world_radius());
	}
	if(mPlugInType == MULTIPLE_PURSUIT)
	{
		/*do nothing*/;
	}
	if(mPlugInType == SOCCER)
	{
		ValueList<LPoint3f> points = get_playing_field();
		points[0].write_datagram(dg);
		points[1].write_datagram(dg);
		dg.add_stdfloat(get_goal_fraction());
	}
	if(mPlugInType == CAPTURE_THE_FLAG)
	{
		;
	}
	if(mPlugInType == LOW_SPEED_TURN)
	{
		dg.add_stdfloat(get_steering_speed());
	}
	if(mPlugInType == MAP_DRIVE)
	{
		;
	}
}

/**
 * Receives an array of pointers, one for each time manager->read_pointer()
 * was called in fillin(). Returns the number of pointers processed.
 */
int OSSteerPlugIn::complete_pointers(TypedWritable **p_list, BamReader *manager)
{
	int pi = PandaNode::complete_pointers(p_list, manager);

	/// Pointers
	///The reference node path.
	PT(PandaNode)referenceNPPandaNode = DCAST(PandaNode, p_list[pi++]);
	mReferenceNP = NodePath::any_path(referenceNPPandaNode);

	///Steer vehicles.
	{
		pvector<PT(OSSteerVehicle)>::iterator iter;
		for (iter = mSteerVehicles.begin(); iter != mSteerVehicles.end(); ++iter)
		{
			(*iter) = DCAST(OSSteerVehicle, p_list[pi++]);
		}
	}

	///The "local" obstacles handled by this OSSteerPlugIn.
	{
		pvector<OSSteerManager::ObstacleAttributes>::iterator iter;
		for (iter = mLocalObstacles.second().begin();
				iter != mLocalObstacles.second().end(); ++iter)
		{
			PT(PandaNode)realPandaNode = DCAST(PandaNode, p_list[pi++]);
			(*iter).second() = NodePath::any_path(realPandaNode);
		}
	}
	return pi;
}

/**
 * Called by the BamReader to perform any final actions needed for setting up
 * the object after all objects have been read and all pointers have been
 * completed.
 */
void OSSteerPlugIn::finalize(BamReader *manager)
{
	//1: (re)set OpenSteer plug-in type
	//create the new OpenSteer plug-in
	do_create_plug_in(mPlugInType);
	//set pathway for the new OpenSteer plug-in
	set_pathway(mPathwayPoints, mPathwayRadii, mPathwaySingleRadius,
			mPathwayClosedCycle);
	//set the new OpenSteer plug-in's local obstacles reference
	static_cast<ossup::PlugIn*>(mPlugIn)->localObstacles =
			&mLocalObstacles.first();
	//set the new OpenSteer plug-in's global obstacles reference
	static_cast<ossup::PlugIn*>(mPlugIn)->obstacles =
			&OSSteerManager::get_global_ptr()->get_global_obstacles().first();
	//open the new OpenSteer plug-in
	mPlugIn->open();
	//2: add OpenSteer vehicles to the new OpenSteer plug-in's real update list
	{
		pvector<PT(OSSteerVehicle)>::iterator iter;
		for (iter = mSteerVehicles.begin(); iter != mSteerVehicles.end(); ++iter)
		{
			//XXX
			//first check vehicle's compatibility, because it might
			//not have gained its final type
			if (check_steer_vehicle_compatibility(NodePath::any_path((*iter))))
			{
				//do add to real update list
				static_cast<ossup::PlugIn*>(mPlugIn)->addVehicle(
						&(*iter)->get_abstract_vehicle());
			}
		}
	}
	//3: (re)add obstacles
	//temporarily remove all ObstacleAttributes (if any)
	pvector<OSSteerManager::ObstacleAttributes> currentObstacleAttrs =
	mLocalObstacles.second();
	mLocalObstacles.first().clear();
	mLocalObstacles.second().clear();
	{
		pvector<OSSteerManager::ObstacleAttributes>::iterator iter;
		for (iter = currentObstacleAttrs.begin(); iter != currentObstacleAttrs.end(); ++iter)
		{
			do_add_obstacle((*iter).second(),
					(*iter).first().get_type(),
					(*iter).first().get_seenFromState(),
					(*iter).first().get_width(),
					(*iter).first().get_height(),
					(*iter).first().get_depth(),
					(*iter).first().get_radius(),
					(*iter).first().get_side(),
					(*iter).first().get_up(),
					(*iter).first().get_forward(),
					(*iter).first().get_position());
		}
	}

	///SPECIFICS
	if(mPlugInType == ONE_TURNING)
	{
		/*do nothing*/;
	}
	if(mPlugInType == PEDESTRIAN)
	{
		set_proximity_database(mPD_ser);
	}
	if(mPlugInType == BOID)
	{
		set_proximity_database(mPD_ser);
		set_world_center(mWorldCenter_ser);
		set_world_radius(mWorldRadius_ser);
	}
	if(mPlugInType == MULTIPLE_PURSUIT)
	{
		/*do nothing*/;
	}
	if(mPlugInType == SOCCER)
	{
		set_playing_field(mFieldMinPoint_ser, mFieldMaxPoint_ser,
				mGoalFraction_ser);
		pvector<PT(OSSteerVehicle)>::iterator iter;
		for (iter = mSteerVehicles.begin(); iter != mSteerVehicles.end(); ++iter)
		{
			// check if vehicle has gained its final type (i.e. finalized)
			if (dynamic_cast<ossup::Player<OSSteerVehicle>*>(
					&(*iter)->get_abstract_vehicle()))
			{
				add_player_to_team((*iter), (*iter)->mPlayingTeam_ser);
			}
		}
	}
	if(mPlugInType == CAPTURE_THE_FLAG)
	{
		;
	}
	if(mPlugInType == LOW_SPEED_TURN)
	{
		set_steering_speed(mSteeringSpeed_ser);
	}
	if(mPlugInType == MAP_DRIVE)
	{
		;
	}
}

/**
 * Some objects require all of their nested pointers to have been completed
 * before the objects themselves can be completed.  If this is the case,
 * override this method to return true, and be careful with circular
 * references (which would make the object unreadable from a bam file).
 */
bool OSSteerPlugIn::require_fully_complete() const
{
	return true;
}

/**
 * This function is called by the BamReader's factory when a new object of
 * type OSSteerPlugIn is encountered in the Bam file.  It should create the
 * OSSteerPlugIn and extract its information from the file.
 */
TypedWritable *OSSteerPlugIn::make_from_bam(const FactoryParams &params)
{
	// continue only if OSSteerManager exists
	CONTINUE_IF_ELSE_R(OSSteerManager::get_global_ptr(), NULL)

	// create a OSSteerPlugIn with default parameters' values: they'll be restored later
	OSSteerManager::get_global_ptr()->set_parameters_defaults(
			OSSteerManager::STEERPLUGIN);
	OSSteerPlugIn *node = DCAST(OSSteerPlugIn,
			OSSteerManager::get_global_ptr()->create_steer_plug_in().node());

	DatagramIterator scan;
	BamReader *manager;

	parse_params(params, scan, manager);
	node->fillin(scan, manager);
	manager->register_finalize(node);

	return node;
}

/**
 * This internal function is called by make_from_bam to read in all of the
 * relevant data from the BamFile for the new OSSteerPlugIn.
 */
void OSSteerPlugIn::fillin(DatagramIterator &scan, BamReader *manager)
{
	PandaNode::fillin(scan, manager);

	///Name of this OSSteerPlugIn.
	set_name(scan.get_string());

	///The type of this OSSteerPlugIn.
	mPlugInType = (OSSteerPlugInType) scan.get_uint8();

	///Pathway stuff.
	///@{
	// clear default pathway
	mPathwayPoints.clear();
	mPathwayRadii.clear();
	int sizei = scan.get_int32();
	for (int i = 0; i < sizei; ++i)
	{
		LPoint3f point;
		point.read_datagram(scan);
		mPathwayPoints.add_value(point);
	}
	sizei = scan.get_int32();
	for (int i = 0; i < sizei; ++i)
	{
		mPathwayRadii.add_value(scan.get_stdfloat());
	}
	mPathwaySingleRadius = scan.get_bool();
	mPathwayClosedCycle = scan.get_bool();
	///@}

	/// Pointers
	///The reference node path.
	manager->read_pointer(scan);

	///Steer vehicles.
	mSteerVehicles.clear();
	unsigned int size = scan.get_uint32();
	mSteerVehicles.resize(size);
	for (unsigned int i = 0; i < mSteerVehicles.size(); ++i)
	{
		manager->read_pointer(scan);
	}

	///The "local" obstacles handled by this OSSteerPlugIn.
	//resize mLocalObstacles: will be restored in complete_pointers()
	mLocalObstacles.first().clear();
	mLocalObstacles.second().clear();
	size = scan.get_uint32();
	mLocalObstacles.first().resize(size);
	mLocalObstacles.second().resize(size);
	for (unsigned int i = 0; i < mLocalObstacles.first().size(); ++i)
	{
		mLocalObstacles.first()[i] = NULL;
		mLocalObstacles.second()[i].first().read_datagram(scan);
		manager->read_pointer(scan);
	}

	///SPECIFICS
	if(mPlugInType == ONE_TURNING)
	{
		/*do nothing*/;
	}
	if(mPlugInType == PEDESTRIAN)
	{
		mPD_ser = (OSProximityDatabase) scan.get_uint8();
	}
	if(mPlugInType == BOID)
	{
		mPD_ser = (OSProximityDatabase) scan.get_uint8();
		mWorldCenter_ser.read_datagram(scan);
		mWorldRadius_ser = scan.get_stdfloat();
	}
	if(mPlugInType == MULTIPLE_PURSUIT)
	{
		/*do nothing*/;
	}
	if(mPlugInType == SOCCER)
	{
		mFieldMinPoint_ser.read_datagram(scan);
		mFieldMaxPoint_ser.read_datagram(scan);
		mGoalFraction_ser = scan.get_stdfloat();
	}
	if(mPlugInType == CAPTURE_THE_FLAG)
	{
		;
	}
	if(mPlugInType == LOW_SPEED_TURN)
	{
		mSteeringSpeed_ser = scan.get_stdfloat();
	}
	if(mPlugInType == MAP_DRIVE)
	{
		;
	}
}

//TypedObject semantics: hardcoded
TypeHandle OSSteerPlugIn::_type_handle;
