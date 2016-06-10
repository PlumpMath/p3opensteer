/**
 * \file osSteerManager.h
 *
 * \date 2016-05-13
 * \author consultit
 */

#ifndef OSSTEERMANGER_H_
#define OSSTEERMANGER_H_

#include "osTools.h"
#include "opensteer_includes.h"
#include "collisionTraverser.h"
#include "collisionHandlerQueue.h"
#include "collisionRay.h"

class OSSteerPlugIn;
class OSSteerVehicle;

/**
 * OSSteerManager Singleton class.
 *
 * Used for handling OSSteerPlugIns and OSSteerVehicles.
 */
class EXPORT_CLASS OSSteerManager: public TypedReferenceCount,
		public Singleton<OSSteerManager>
{
PUBLISHED:
	OSSteerManager(const NodePath& root = NodePath(),
			const CollideMask& mask = GeomNode::get_default_collide_mask());
	virtual ~OSSteerManager();

	/**
	 * \name REFERENCE NODES
	 */
	///@{
	INLINE NodePath get_reference_node_path() const;
	INLINE void set_reference_node_path(const NodePath& reference);
	INLINE NodePath get_reference_node_path_debug() const;
	///@}

	/**
	 * \name OSSteerPlugIn
	 */
	///@{
	NodePath create_steer_plug_in();
	bool destroy_steer_plug_in(NodePath plugInNP);
	NodePath get_steer_plug_in(int index) const;
	INLINE int get_num_steer_plug_ins() const;
	MAKE_SEQ(get_steer_plug_ins, get_num_steer_plug_ins, get_steer_plug_in);
	///@}

	/**
	 * \name OSSteerVehicle
	 */
	///@{
	NodePath create_steer_vehicle(const string& name);
	bool destroy_steer_vehicle(NodePath steerVehicleNP);
	NodePath get_steer_vehicle(int index) const;
	INLINE int get_num_steer_vehicles() const;
	MAKE_SEQ(get_steer_vehicles, get_num_steer_vehicles, get_steer_vehicle);
	///@}

	/**
	 * The type of object for creation parameters.
	 */
	enum OSType
	{
		STEERPLUGIN = 0,
		STEERVEHICLE
	};

	/**
	 * \name TEXTUAL PARAMETERS
	 */
	///@{
	ValueList<string> get_parameter_name_list(OSType type) const;
	void set_parameter_values(OSType type, const string& paramName, const ValueList<string>& paramValues);
	ValueList<string> get_parameter_values(OSType type, const string& paramName) const;
	void set_parameter_value(OSType type, const string& paramName, const string& value);
	string get_parameter_value(OSType type, const string& paramName) const;
	void set_parameters_defaults(OSType type);
	///@}

	/**
	 * \name DEFAULT UPDATE
	 */
	///@{
	AsyncTask::DoneStatus update(GenericAsyncTask* task);
	void start_default_update();
	void stop_default_update();
	///@}

	/**
	 * \name SINGLETON
	 */
	///@{
	INLINE static OSSteerManager* get_global_ptr();
	///@}

	/**
	 * \name UTILITIES
	 */
	///@{
	float get_bounding_dimensions(NodePath modelNP, LVecBase3f& modelDims,
			LVector3f& modelDeltaCenter) const;
	Pair<bool,float> get_collision_height(const LPoint3f& origin,
			const NodePath& space = NodePath()) const;
	INLINE CollideMask get_collide_mask() const;
	INLINE NodePath get_collision_root() const;
	INLINE CollisionTraverser* get_collision_traverser() const;
	INLINE CollisionHandlerQueue* get_collision_handler() const;
	INLINE CollisionRay* get_collision_ray() const;
	///@}

	/**
	 * \name SERIALIZATION
	 */
	///@{
	bool write_to_bam_file(const string& fileName);
	bool read_from_bam_file(const string& fileName);
	///@}

	/**
	 * Equivalent to duDebugDrawPrimitives.
	 */
	enum OSDebugDrawPrimitives
	{
#ifndef CPPPARSER
		POINTS = ossup::DrawMeshDrawer::DRAW_POINTS,
		LINES = ossup:DrawMeshDrawer::DRAW_LINES,
		TRIS = ossup:DrawMeshDrawer::DRAW_TRIS,
		QUADS = ossup:DrawMeshDrawer::DRAW_QUADS,
#else
		POINTS,LINES,TRIS,QUADS
#endif //CPPPARSER
	};

	/**
	 * \name LOW LEVEL DEBUG DRAWING
	 */
	///@{
	void debug_draw_primitive(RNDebugDrawPrimitives primitive,
			const ValueList<LPoint3f>& points, const LVecBase4f color = LVecBase4f::zero(), float size =
					1.0f);
	void debug_draw_reset();
	///@}

private:
	///The reference node path.
	NodePath mReferenceNP;
	///List of OSSteerPlugIns handled by this manager.
	typedef pvector<PT(OSSteerPlugIn)> SteerPlugInList;
	SteerPlugInList mNavMeshes;
	///OSSteerPlugIns' parameter table.
	ParameterTable mNavMeshesParameterTable;

	///List of OSSteerVehicles handled by this template.
	typedef pvector<PT(OSSteerVehicle)> SteerVehicleList;
	SteerVehicleList mCrowdAgents;
	///OSSteerVehicles' parameter table.
	ParameterTable mCrowdAgentsParameterTable;

	///@{
	///A task data for step simulation update.
	PT(TaskInterface<OSSteerManager>::TaskData) mUpdateData;
	PT(AsyncTask) mUpdateTask;
	///@}

	///Utilities.
	NodePath mRoot;
	CollideMask mMask; //a.k.a. BitMask32
	CollisionTraverser* mCTrav;
	CollisionHandlerQueue* mCollisionHandler;
	CollisionRay* mPickerRay;

	///The reference node path for debug drawing.
	NodePath mReferenceDebugNP;
#ifdef OS_DEBUG
	class DebugDrawPrimitives: public ossup::DebugDrawPanda3d
	{
	public:
		DebugDrawPrimitives(NodePath render): ossup::DebugDrawPanda3d(render)
		{
		}
	};
	/// DebugDrawers.
	DebugDrawPrimitives* mDD;
#endif //OS_DEBUG

public:
	/**
	 * \name TypedObject API
	 */
	///@{
	static TypeHandle get_class_type()
	{
		return _type_handle;
	}
	static void init_type()
	{
		TypedReferenceCount::init_type();
		register_type(_type_handle, "OSSteerManager",
				TypedReferenceCount::get_class_type());
	}
	virtual TypeHandle get_type() const override
	{
		return get_class_type();
	}
	virtual TypeHandle force_init_type() override
	{
		init_type();
		return get_class_type();
	}
	///@}

private:
	static TypeHandle _type_handle;

};

///inline
#include "osSteerManager.I"

#endif /* OSSTEERMANGER_H_ */
