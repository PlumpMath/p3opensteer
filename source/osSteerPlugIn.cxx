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
			string("type"));
	//pathway (will be used on setup())
	string mPathwayParam = mTmpl->get_parameter_value(OSSteerManager::STEERPLUGIN,
			string("pathway"));
	//obstacles (will be used on setup())
	plist<string> mObstacleListParam = mTmpl->get_parameter_values(OSSteerManager::STEERPLUGIN,
			string("obstacles"));
	//
	//create the plug in
	if (mPlugInTypeParam == string("pedestrian"))
	{
		mPlugIn = new ossup::PedestrianPlugIn<OSSteerVehicle>;
		mPlugInType = PEDESTRIAN;
	}
	else if (mPlugInTypeParam == string("boid"))
	{
		mPlugIn = new ossup::BoidsPlugIn<OSSteerVehicle>;
		mPlugInType = BOID;
	}
	else if (mPlugInTypeParam == string("multiple_pursuit"))
	{
		mPlugIn = new ossup::MpPlugIn<OSSteerVehicle>;
		mPlugInType = MULTIPLE_PURSUIT;
	}
	else if (mPlugInTypeParam == string("soccer"))
	{
		mPlugIn = new ossup::MicTestPlugIn<OSSteerVehicle>;
		mPlugInType = SOCCER;
	}
	else if (mPlugInTypeParam == string("capture_the_flag"))
	{
		mPlugIn = new ossup::CtfPlugIn<OSSteerVehicle>;
		mPlugInType = CAPTURE_THE_FLAG;
	}
	else if (mPlugInTypeParam == string("low_speed_turn"))
	{
		mPlugIn = new ossup::LowSpeedTurnPlugIn<OSSteerVehicle>;
		mPlugInType = LOW_SPEED_TURN;
	}
	else if (mPlugInTypeParam == string("map_drive"))
	{
		mPlugIn = new ossup::MapDrivePlugIn<OSSteerVehicle>;
		mPlugInType = MAP_DRIVE;
	}
	else
	{
		//default: "one_turning"
		mPlugIn = new ossup::OneTurningPlugIn<OSSteerVehicle>;
		mPlugInType = ONE_TURNING;
	}
	//build pathway
	do_build_pathway(mPathwayParam);
	//set the plugin local obstacles reference
	static_cast<ossup::PlugIn*>(mPlugIn)->localObstacles = &mLocalObstacles;
	//set the plugin global obstacles reference
	static_cast<ossup::PlugIn*>(mPlugIn)->obstacles =
			&mTmpl->get_global_obstacles().first();
	//add its own obstacles
	if (! mReferenceNP.is_empty())
	{
		do_add_obstacles(mObstacleListParam);
	}
	//open the plug in
	mPlugIn->open();
}

/**
 * Builds the pathway.
 * \note Internal use only.
 */
void OSSteerPlugIn::do_build_pathway(const string& pathwayParam)
{
	//
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
		static_cast<ossup::PlugIn*>(mPlugIn)->setPathway(numPoints, points, true,
				&radius, closedCycle);
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
		static_cast<ossup::PlugIn*>(mPlugIn)->setPathway(numPoints, points, false,
				radii, closedCycle);
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
	//
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
	for (iterLocal = mLocalObstacles.begin();
			iterLocal != mLocalObstacles.end(); ++iterLocal)
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
	//remove all handled SteerVehicles (if any) from update
	PTA(PT(OSSteerVehicle))::const_iterator iter;
	for (iter = mSteerVehicles.begin(); iter != mSteerVehicles.end(); ++iter)
	{
		//set steerVehicle reference to null
		(*iter)->mSteerPlugIn.clear();
		//do remove from real update list
		static_cast<ossup::PlugIn*>(mPlugIn)->removeVehicle(
				&(*iter)->get_abstract_vehicle());
	}
	//close the plug in
	mPlugIn->close();
	//delete the plug in
	delete mPlugIn;
	do_reset();
}

typedef ossup::VehicleAddOnMixin<ossup::SimpleVehicle, OSSteerVehicle> VehicleAddOn;

/**
 * Adds a OSSteerVehicle to this OSSteerPlugIn (ie to the underlying OpenSteer
 * management mechanism).
 * Returns a negative number on error.
 */
int OSSteerPlugIn::add_steer_vehicle(NodePath steerVehicleNP)
{
	CONTINUE_IF_ELSE_R(
			(!steerVehicleNP.is_empty())
					&& (steerVehicleNP.node()->is_of_type(
							OSSteerVehicle::get_class_type())), OS_ERROR)

	PT(OSSteerVehicle)steerVehicle = DCAST(OSSteerVehicle, steerVehicleNP.node());

	// continue if steerVehicle doesn't belong to any plug-in
	CONTINUE_IF_ELSE_R(!steerVehicle->mSteerPlugIn, OS_ERROR)

	bool result = false;
	//add to update list
	PTA(PT(OSSteerVehicle))::iterator iter;
	//check if OSSteerVehicle has not been already added
	iter = find(mSteerVehicles.begin(), mSteerVehicles.end(), steerVehicle);
	if (iter == mSteerVehicles.end())
	{
		//OSSteerVehicle needs to be added
		//get the actual pos
		LPoint3f pos = steerVehicleNP.get_pos();
		LVector3f newForward = mReferenceNP.get_relative_vector(steerVehicleNP,
				LVector3f::forward());
		LVector3f newUp = mReferenceNP.get_relative_vector(steerVehicleNP,
				LVector3f::up());
		steerVehicleNP.heads_up(pos + newForward, newUp);
		//OSSteerPlugIn object updates SteerVehicle's pos/dir wrt reference node path
		ossup::VehicleSettings settings =
				static_cast<VehicleAddOn*>(steerVehicle->mVehicle)->getSettings();
		settings.m_forward =
				ossup::LVecBase3fToOpenSteerVec3(newForward).normalize();
		settings.m_up = ossup::LVecBase3fToOpenSteerVec3(newUp).normalize();
		settings.m_position = ossup::LVecBase3fToOpenSteerVec3(pos);

		//get steerVehicle dimensions
		LVecBase3f modelDims;
		LVector3f modelDeltaCenter;
		float modelRadius;
//	if (!buildFromBam) XXX
//	{
		//compute new agent dimensions
		modelRadius = OSSteerManager::get_global_ptr()->get_bounding_dimensions(
				steerVehicleNP, modelDims, modelDeltaCenter);
//	} XXX
//	else
//	{
//		modelRadius = crowdAgent->mAgentParams.get_radius();
//		modelDims.set_x(0.0);
//		modelDims.set_y(0.0);
//		modelDims.set_z(crowdAgent->mAgentParams.get_height());
//	}
		//update radius
		settings.m_radius = modelRadius;
		static_cast<VehicleAddOn*>(steerVehicle->mVehicle)->setSettings(
				settings);

		//set height correction
		steerVehicle->mHeigthCorrection = LVector3f(0.0, 0.0,
				modelDims.get_z());
		//add to the list of SteerVehicles
		mSteerVehicles.push_back(steerVehicle);
		//do add to real update list
		static_cast<ossup::PlugIn*>(mPlugIn)->addVehicle(
				&steerVehicle->get_abstract_vehicle());
		//set steerVehicle reference to this plugin
		steerVehicle->mSteerPlugIn = this;
		//
		result = true;
	}
	//
	return (result ? OS_SUCCESS : OS_ERROR);
}

/**
 * Removes a SteerVehicle from this OSSteerPlugIn (ie from the OpenSteer
 * handling mechanism).
 * Returns a negative number on error.
 */
int OSSteerPlugIn::remove_steer_vehicle(NodePath steerVehicleNP)
{
	CONTINUE_IF_ELSE_R(
			(!steerVehicleNP.is_empty())
					&& (steerVehicleNP.node()->is_of_type(
							OSSteerVehicle::get_class_type())), OS_ERROR)

	PT(OSSteerVehicle)steerVehicle = DCAST(OSSteerVehicle, steerVehicleNP.node());

	// continue if steerVehicle belongs to this plug-in
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
 * Sets the pathway of this OSSteerPlugin.
 */
void OSSteerPlugIn::set_pathway(int numOfPoints, LPoint3f const points[],
		bool singleRadius, float const radii[], bool closedCycle)
{
	//convert to OpenSteer points
	OpenSteer::Vec3* osPoints = new OpenSteer::Vec3[numOfPoints];
	for (int idx = 0; idx < numOfPoints; ++idx)
	{
		osPoints[idx] = ossup::LVecBase3fToOpenSteerVec3(points[idx]);
	}
	//set the actual path
	static_cast<ossup::PlugIn*>(mPlugIn)->setPathway(numOfPoints, osPoints,
			singleRadius, radii, closedCycle);
	//
	delete[] osPoints;
}

/**
 * Adds an obstacle.
 * A non empty NodePath (objectNP) will corresponds to the  underlying OpenSteer
 * obstacle, and is directly reparented to the reference node. In this case
 * parameters are extracted from the NodePath.
 * Otherwise (empty NodePath) it is only seen by the OpenSteer library, and all
 * parameters should be specified.\n
 * objectNP, type and seenFromState parameters must be always specified.\n
 * Returns the obstacle's unique reference (>0), or a negative number on error.
 */
int OSSteerPlugIn::add_obstacle(NodePath& objectNP,
		const string& type, const string& seenFromState,
		float width, float height,	float depth,
		float radius, const LVector3f& side, const LVector3f& up,
		const LVector3f& forward, const LPoint3f& position)
{
	LPoint3f newPos = position;
	LVector3f newSide = side, newUp = up, newForw = forward;
	if (!objectNP.is_empty())
	{
		//get obstacle dimensions
		LVecBase3f modelDims;
		LVector3f modelDeltaCenter;
		float modelRadius;
//		if (!buildFromBam) XXX
//		{
		//compute new obstacle dimensions
		modelRadius = OSSteerManager::get_global_ptr()->get_bounding_dimensions(
				objectNP, modelDims, modelDeltaCenter);
//		} XXX
//		else
//		{
//			modelRadius = mObstacles[index].first().get_radius();
//			modelDims = mObstacles[index].first().get_dims();
//		}

		//the obstacle is reparented to the OSSteerPlugIn's reference node path
		objectNP.wrt_reparent_to(mReferenceNP);
		//correct obstacle's parameters
		newPos = objectNP.get_pos();
		newForw = mReferenceNP.get_relative_vector(objectNP,
				LVector3f::forward());
		newUp = mReferenceNP.get_relative_vector(objectNP, LVector3f::up());
		newSide = mReferenceNP.get_relative_vector(objectNP,
				LVector3f::right());
		width = modelDims.get_x();
		height = modelDims.get_z();
		depth = modelDims.get_y();
		radius = modelRadius;
	}
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
		box->setSide(ossup::LVecBase3fToOpenSteerVec3(newSide).normalize());
		box->setUp(ossup::LVecBase3fToOpenSteerVec3(newUp).normalize());
		box->setForward(ossup::LVecBase3fToOpenSteerVec3(newForw).normalize());
		box->setPosition(ossup::LVecBase3fToOpenSteerVec3(newPos));
		obstacle->setSeenFrom(seenFS);
	}
	if (type == string("plane"))
	{
		obstacle = new ossup::PlaneObstacle(
				ossup::LVecBase3fToOpenSteerVec3(newSide).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(newUp).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(newForw).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(newPos));
		obstacle->setSeenFrom(seenFS);
	}
	if (type == string("rectangle"))
	{
		obstacle = new ossup::RectangleObstacle(width, height,
				ossup::LVecBase3fToOpenSteerVec3(newSide).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(newUp).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(newForw).normalize(),
				ossup::LVecBase3fToOpenSteerVec3(newPos), seenFS);
	}
	if (type == string("sphere"))
	{
		obstacle = new ossup::SphereObstacle(radius,
				ossup::LVecBase3fToOpenSteerVec3(newPos));
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
		settings.set_position(newPos);
		settings.set_forward(newForw);
		settings.set_up(newUp);
		settings.set_side(newSide);
		settings.set_width(width);
		settings.set_height(height);
		settings.set_depth(depth);
		settings.set_radius(radius);
		ref = unique_ref();
		settings.set_ref(ref);
		settings.set_obstacle(obstacle);
		//2: add OpenSteer obstacle's pointer to global list
		globalObstacles.first().push_back(obstacle);
		//3: add obstacle's attributes to global list
		globalObstacles.second().push_back(
				OSSteerManager::ObstacleAttributes(settings, objectNP));
		//4: add OpenSteer obstacle's pointer to local list
		mLocalObstacles.push_back(obstacle);
	}
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
	OpenSteer::ObstacleGroup::iterator iterLocal = find(mLocalObstacles.begin(),
			mLocalObstacles.end(), (*iterA).first().get_obstacle());
	if (iterLocal != mLocalObstacles.end())
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
		delete *iterLocal;
		//4: remove the OpenSteer obstacle's pointer from the local list
		mLocalObstacles.erase(iterLocal);
	}
	//
	return resultNP;
}

/**
 * Updates the OpenSteer underlying plug-in.
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
			//unset enableAnnotation
			ossup::enableAnnotation = true;

			//set drawers
			mDrawer3d->initialize();
			mDrawer2d->initialize();
			gDrawer3d = mDrawer3d;
			gDrawer2d = mDrawer2d;

			// invoke PlugIn's Update method
			mPlugIn->update(mCurrentTime, dt);

			// invoke selected PlugIn's Redraw method
			mPlugIn->redraw(mCurrentTime, dt);
			// draw any annotation queued up during selected PlugIn's Update method
			OpenSteer::drawAllDeferredLines();
			OpenSteer::drawAllDeferredCirclesOrDisks();
			mDrawer3d->finalize();
			mDrawer2d->finalize();
		}
		else
		{
			//unset enableAnnotation
			ossup::enableAnnotation = false;
#endif //OS_DEBUG

			// invoke PlugIn's Update method
			mPlugIn->update(mCurrentTime, dt);

#ifdef OS_DEBUG
		}
	}
#endif //OS_DEBUG
}

/**
 * Sets steering speed (LOW SPEED TURN).
 */
void OSSteerPlugIn::set_steering_speed(float steeringSpeed)
{
	if (mPlugInType == LOW_SPEED_TURN)
	{
		static_cast<ossup::LowSpeedTurnPlugIn<OSSteerVehicle>*>(mPlugIn)->steeringSpeed =
				steeringSpeed;
	}
}

/**
 * Returns steering speed (LOW SPEED TURN).
 */
float OSSteerPlugIn::get_steering_speed()
{
	return mPlugInType == LOW_SPEED_TURN ?
			static_cast<ossup::LowSpeedTurnPlugIn<OSSteerVehicle>*>(mPlugIn)->steeringSpeed :
			0.0;
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
 * Enables the debug drawing, only if nav mesh has been already setup.
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
		//set the 2D debug node as child of mReferenceDebug2DNP node
		mDrawer2dNP = mReferenceDebug2DNP.attach_new_node(
				string("OpenSteerDebugNodePath2D_") + get_name());
		//set debug node paths
		mDrawer3dNP.set_bin("fixed", 10);
		mDrawer2dNP.set_bin("fixed", 10);
		//by default Debug NodePaths are hidden
		mDrawer3dNP.hide();
		mDrawer2dNP.hide();
		//no collide mask for all Debug NodePaths' children
		mDrawer3dNP.set_collide_mask(BitMask32::all_off());
		mDrawer2dNP.set_collide_mask(BitMask32::all_off());
		//create new Debug Drawers
		NodePath meshDrawerCamera = mDebugCamera;
		if (! mDebugCamera.node()->is_of_type(Camera::get_class_type()))
		{
			meshDrawerCamera = mDebugCamera.find(string("**/+Camera"));
		}
		mDrawer3d = new ossup::DrawMeshDrawer(mDrawer3dNP, meshDrawerCamera, 100,
				0.04);
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
		//set the recast debug camera to empty node path
		mDebugCamera = NodePath();
		//remove the recast debug node paths
		mDrawer3dNP.remove_node();
		mDrawer2dNP.remove_node();
		//reset the DebugDrawers
		if (mDrawer3d)
		{
			delete mDrawer3d;
			mDrawer3d = NULL;
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
 * Returns a negative number on error.
 */
int OSSteerPlugIn::toggle_debug_drawing(bool enable)
{
#ifdef OS_DEBUG
	//continue if mDrawer3dNP and mDrawer2dNP are not empty
	CONTINUE_IF_ELSE_R((!mDrawer3dNP.is_empty()) && (!mDrawer2dNP.is_empty()),
			OS_ERROR)

	if (enable)
	{
		if (mDrawer3dNP.is_hidden())
		{
			mDrawer3dNP.show();
		}
		if (mDrawer2dNP.is_hidden())
		{
			mDrawer2dNP.show();
		}
	}
	else
	{
		if (! mDrawer3dNP.is_hidden())
		{
			mDrawer3dNP.hide();
		}
		if (! mDrawer2dNP.is_hidden())
		{
			mDrawer2dNP.hide();
		}
	}
	//set Debug Draw Update
	mEnableDebugDrawUpdate = enable;
	//clear drawers
	mDrawer3d->clear();
	mDrawer2d->clear();
	//
#endif
	return OS_SUCCESS;
}

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

	//XXX


	///Current underlying AbstractPlugIn.
//	OpenSteer::AbstractPlugIn* mPlugIn;
	///The type of this OSSteerPlugIn.
	dg.add_uint8((uint8_t) mPlugInType);
	///The reference node path.
	NodePath mReferenceNP;
	///The reference node path for debug drawing.
//	NodePath mReferenceDebugNP, mReferenceDebug2DNP;
	///Current time.
//	float mCurrentTime;
	///The SteerVehicle components handled by this OSSteerPlugIn.
	pset<PT(OSSteerVehicle)> mSteerVehicles;
	///The "local" obstacles handled by this OSSteerPlugIn.
	OpenSteer::ObstacleGroup mLocalObstacles;
	///Unique ref.
	int mRef;

#ifdef OS_DEBUG
	///OpenSteer debug node paths.
	NodePath mDrawer3dNP, mDrawer2dNP;
	///OpenSteer debug camera.
	NodePath mDebugCamera;
	///OpenSteer DebugDrawers.
	ossup::DrawMeshDrawer *mDrawer3d, *mDrawer2d;
	///Enable Debug Draw update.
	bool mEnableDebugDrawUpdate;
#endif

}

/**
 * Receives an array of pointers, one for each time manager->read_pointer()
 * was called in fillin(). Returns the number of pointers processed.
 */
int OSSteerPlugIn::complete_pointers(TypedWritable **p_list, BamReader *manager)
{
	int pi = PandaNode::complete_pointers(p_list, manager);

	//XXX

	return pi;
}

/**
 * Called by the BamReader to perform any final actions needed for setting up
 * the object after all objects have been read and all pointers have been
 * completed.
 */
void OSSteerPlugIn::finalize(BamReader *manager)
{
	//XXX
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

	//XXX
}

//TypedObject semantics: hardcoded
TypeHandle OSSteerPlugIn::_type_handle;
