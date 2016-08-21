/**
 * \file common.h
 *
 * \date 2016-06-20
 * \author consultit
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <random>
#include <algorithm>
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
extern string vehicleFile[5];
extern string vehicleAnimFiles[5][2];
extern const float animRateFactor[2];
//obstacle model
extern string obstacleFile;
//bam file
extern string bamFileName;
//support
extern random_device rd;

///functions' declarations
void startFramework(int argc, char *argv[], const string&);
NodePath loadPlane(const string&, float widthX = 30.0, float widthY = 30.0);
NodePath loadTerrainLowPoly(const string&, float widthScale = 128,
		float heightScale = 64.0, const string& texture = "dry-grass.png");
NodePath loadTerrain(const string&, float widthScale = 0.5,
		float heightScale = 10.0);
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
			PT(OSSteerPlugIn)steerPlugIn,
			const LVecBase3f& scale = LVecBase3f(1.0, 1.0, 1.0)):
	addObstacle(addObstacle), sceneNP(sceneNP), steerPlugIn(steerPlugIn),
	scale(scale)
	{
	}
	//
	bool addObstacle;
	NodePath sceneNP;
	PT(OSSteerPlugIn)steerPlugIn;
	LVecBase3f scale;
};
void handleObstacles(const Event*, void*);
// data passed to vehicle's handling callback
struct HandleVehicleData
{
	HandleVehicleData(float meanScale, int vehicleFileIdx,
			const string& moveType, const NodePath& sceneNP,
			PT(OSSteerPlugIn)steerPlugIn,
			vector<PT(OSSteerVehicle)>&steerVehicles,
			vector<vector<PT(AnimControl)> >& vehicleAnimCtls,
			const LVector3f& deltaPos = LVector3f::zero()):
		meanScale(meanScale), vehicleFileIdx(vehicleFileIdx), moveType(moveType),
		sceneNP(sceneNP), steerPlugIn(steerPlugIn),
		steerVehicles(steerVehicles), vehicleAnimCtls(vehicleAnimCtls),
		deltaPos(deltaPos)
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
	LVector3f deltaPos;
};
void handleVehicles(const Event*, void*);

// Driver class
class Driver
{
public:
	Driver(PandaFramework* framework, const NodePath& ownerObject,
			int taskSort = 0);
	virtual ~Driver();

	AsyncTask::DoneStatus update(GenericAsyncTask* task);

	//enable/disable
	bool enable();
	bool disable();
	inline bool is_enabled() const;
	//enable/disable controls
	inline void enable_forward(bool enable);
	inline bool is_forward_enabled() const;
	inline void enable_backward(bool enable);
	inline bool is_backward_enabled() const;
	inline void enable_strafe_left(bool enable);
	inline bool is_strafe_left_enabled() const;
	inline void enable_strafe_right(bool enable);
	inline bool is_strafe_right_enabled() const;
	inline void enable_up(bool enable);
	inline bool is_up_enabled() const;
	inline void enable_down(bool enable);
	inline bool is_down_enabled() const;
	inline void enable_head_left(bool enable);
	inline bool is_head_left_enabled() const;
	inline void enable_head_right(bool enable);
	inline bool is_head_right_enabled() const;
	inline void enable_pitch_up(bool enable);
	inline bool is_pitch_up_enabled() const;
	inline void enable_pitch_down(bool enable);
	inline bool is_pitch_down_enabled() const;
	inline void enable_mouse_move(bool enable);
	inline bool is_mouse_move_enabled() const;
	//max values
	inline void set_head_limit(bool enabled = false, float hLimit = 0.0);
	inline void set_pitch_limit(bool enabled = false, float pLimit = 0.0);
	inline void set_max_linear_speed(const LVector3f& linearSpeed);
	inline void set_max_angular_speed(float angularSpeed);
	inline Pair<LVector3f, float> get_max_speeds() const;
	inline void set_linear_accel(const LVector3f& linearAccel);
	inline void set_angular_accel(float angularAccel);
	inline Pair<LVector3f, float> get_accels() const;
	inline void set_linear_friction(float linearFriction);
	inline void set_angular_friction(float angularFriction);
	inline ValueList<float> get_frictions() const;
	inline void set_sens(float sensX, float sensY);
	inline ValueList<float> get_sens() const;
	inline void set_fast_factor(float factor);
	inline float get_fast_factor() const;
	//speed current values
	inline Pair<LVector3f, ValueList<float> > get_current_speeds() const;

private:
	///Main parameters.
	GraphicsWindow* mWin;
	NodePath mOwnerObjectNP;
	///Enabling flag.
	bool mEnabled;
	///Key controls and effective keys.
	bool mForward, mBackward, mStrafeLeft, mStrafeRight, mUp, mDown, mHeadLeft,
			mHeadRight, mPitchUp, mPitchDown, mMouseMove;
	bool mForwardKey, mBackwardKey, mStrafeLeftKey, mStrafeRightKey, mUpKey,
			mDownKey, mHeadLeftKey, mHeadRightKey, mPitchUpKey, mPitchDownKey,
			mMouseMoveKey;
	std::string mSpeedKey;
	///Key control values.
	bool mMouseEnabledH, mMouseEnabledP;
	bool mHeadLimitEnabled, mPitchLimitEnabled;
	float mHLimit, mPLimit;
	int mSignOfTranslation, mSignOfMouse;
	/// Sensitivity settings.
	float mFastFactor;
	LVecBase3f mActualSpeedXYZ, mMaxSpeedXYZ, mMaxSpeedSquaredXYZ;
	float mActualSpeedH, mActualSpeedP, mMaxSpeedHP, mMaxSpeedSquaredHP;
	LVecBase3f mAccelXYZ;
	float mAccelHP;
	float mFrictionXYZ;
	float mFrictionHP;
	float mStopThreshold;
	float mSensX, mSensY;
	int mCentX, mCentY;
	///private member functions
	void do_reset();
	void do_initialize();
	void do_finalize();
	void do_enable();
	void do_disable();
	///A task data for step update.
	PT(TaskInterface<Driver>::TaskData)mUpdateData;
	PT(AsyncTask) mUpdateTask;
	int mTaskSort;
};

inline bool Driver::is_enabled() const
{
	return mEnabled;
}

inline void Driver::enable_forward(bool enable)
{
	if (mForwardKey)
	{
		mForward = enable;
	}
}

inline bool Driver::is_forward_enabled() const
{
	return mForward;
}

inline void Driver::enable_backward(bool enable)
{
	if (mBackwardKey)
	{
		mBackward = enable;
	}
}

inline bool Driver::is_backward_enabled() const
{
	return mBackward;
}

inline void Driver::enable_strafe_left(bool enable)
{
	if (mStrafeLeftKey)
	{
		mStrafeLeft = enable;
	}
}

inline bool Driver::is_strafe_left_enabled() const
{

	return mStrafeLeft;
}

inline void Driver::enable_strafe_right(bool enable)
{
	if (mStrafeRightKey)
	{
		mStrafeRight = enable;
	}
}

inline bool Driver::is_strafe_right_enabled() const
{
	return mStrafeRight;
}

inline void Driver::enable_up(bool enable)
{
	if (mUpKey)
	{
		mUp = enable;
	}
}

inline bool Driver::is_up_enabled() const
{
	return mUp;
}

inline void Driver::enable_down(bool enable)
{
	if (mDownKey)
	{
		mDown = enable;
	}
}

inline bool Driver::is_down_enabled() const
{
	return mDown;
}

inline void Driver::enable_head_left(bool enable)
{
	if (mHeadLeftKey)
	{
		mHeadLeft = enable;
	}
}

inline bool Driver::is_head_left_enabled() const
{
	return mHeadLeft;
}

inline void Driver::enable_head_right(bool enable)
{
	if (mHeadRightKey)
	{
		mHeadRight = enable;
	}
}

inline bool Driver::is_head_right_enabled() const
{
	return mHeadRight;
}

inline void Driver::enable_pitch_up(bool enable)
{
	if (mPitchUpKey)
	{
		mPitchUp = enable;
	}
}

inline bool Driver::is_pitch_up_enabled() const
{
	return mPitchUp;
}

inline void Driver::enable_pitch_down(bool enable)
{
	if (mPitchDownKey)
	{
		mPitchDown = enable;
	}
}

inline bool Driver::is_pitch_down_enabled() const
{
	return mPitchDown;
}

inline void Driver::enable_mouse_move(bool enable)
{
	if (mMouseMoveKey)
	{
		mMouseMove = enable;
	}
}

inline bool Driver::is_mouse_move_enabled() const
{
	return mMouseMove;
}

inline void Driver::set_head_limit(bool enabled, float hLimit)
{
	mHeadLimitEnabled = enabled;
	hLimit >= 0.0 ? mHLimit = hLimit : mHLimit = -hLimit;
}

inline void Driver::set_pitch_limit(bool enabled, float pLimit)
{
	mPitchLimitEnabled = enabled;
	pLimit >= 0.0 ? mPLimit = pLimit : mPLimit = -pLimit;
}

inline void Driver::set_max_linear_speed(const LVector3f& maxLinearSpeed)
{
	mMaxSpeedXYZ = LVector3f(
			abs(maxLinearSpeed.get_x()),
			abs(maxLinearSpeed.get_y()),
			abs(maxLinearSpeed.get_z()));
	mMaxSpeedSquaredXYZ = LVector3f(
			maxLinearSpeed.get_x() * maxLinearSpeed.get_x(),
			maxLinearSpeed.get_y() * maxLinearSpeed.get_y(),
			maxLinearSpeed.get_z() * maxLinearSpeed.get_z());
}

inline void Driver::set_max_angular_speed(float maxAngularSpeed)
{
	mMaxSpeedHP = abs(maxAngularSpeed);
	mMaxSpeedSquaredHP = maxAngularSpeed * maxAngularSpeed;
}

inline Pair<LVector3f, float> Driver::get_max_speeds() const
{
	return Pair<LVector3f, float>(mMaxSpeedXYZ, mMaxSpeedHP);
}

inline void Driver::set_linear_accel(const LVector3f& linearAccel)
{
	mAccelXYZ = LVector3f(
			abs(linearAccel.get_x()),
			abs(linearAccel.get_y()),
			abs(linearAccel.get_z()));
}

inline void Driver::set_angular_accel(float angularAccel)
{
	mAccelHP = abs(angularAccel);
}

inline Pair<LVector3f, float> Driver::get_accels() const
{
	return Pair<LVector3f, float>(mAccelXYZ, mAccelHP);
}

inline void Driver::set_linear_friction(float linearFriction)
{
	mFrictionXYZ = abs(linearFriction);
}

inline void Driver::set_angular_friction(float angularFriction)
{
	mFrictionHP = abs(angularFriction);
}

inline ValueList<float> Driver::get_frictions() const
{
	ValueList<float> frictions;
	frictions.add_value(mFrictionXYZ);
	frictions.add_value(mFrictionHP);
	return frictions;
}

inline void Driver::set_sens(float sensX, float sensY)
{
	mSensX = abs(sensX);
	mSensY = abs(sensY);
}

inline ValueList<float> Driver::get_sens() const
{
	ValueList<float> sens;
	sens.add_value(mSensX);
	sens.add_value(mSensY);
	return sens;
}

inline void Driver::set_fast_factor(float factor)
{
	mFastFactor = abs(factor);
}

inline float Driver::get_fast_factor() const
{
	return mFastFactor;
}

inline Pair<LVector3f, ValueList<float> > Driver::get_current_speeds() const
{
	ValueList<float> angularSpeeds;
	angularSpeeds.add_value(mActualSpeedH);
	angularSpeeds.add_value(mActualSpeedP);
	return Pair<LVector3f, ValueList<float> >(mActualSpeedXYZ, angularSpeeds);
}

#endif /* COMMON_H_ */
