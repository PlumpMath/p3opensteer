/**
 * \file osTools.I
 *
 * \date 2016-05-13
 * \author consultit
 */

#ifndef OSTOOLS_I_
#define OSTOOLS_I_


///ValueList
/**
 *
 */
template<typename Type>
INLINE ValueList<Type>::ValueList(unsigned int size):_values(size)
{
}

/**
 *
 */
template<typename Type>
INLINE ValueList<Type>::ValueList(const ValueList &copy) :
		_values(copy._values)
{
}

/**
 *
 */
template<typename Type>
INLINE ValueList<Type>::~ValueList()
{
}

/**
 * Adds a new value to the collection.
 */
template<typename Type>
INLINE void ValueList<Type>::add_value(const Type& value)
{
	_values.push_back(value);
}

/**
 *
 */
template<typename Type>
INLINE void ValueList<Type>::operator =(const ValueList &copy)
{
	_values = copy._values;
}

/**
 *
 */
template<typename Type>
INLINE bool ValueList<Type>::operator== (const ValueList &other) const
{
	return _values == other._values;
}

/**
 * Removes all values from the collection.
 */
template<typename Type>
INLINE void ValueList<Type>::clear()
{
	_values.clear();
}

/**
 * Returns the number of values in the collection.
 */
template<typename Type>
INLINE int ValueList<Type>::get_num_values() const
{
	return _values.size();
}

/**
 * Returns the nth value in the collection.
 */
template<typename Type>
INLINE Type ValueList<Type>::get_value(int index) const
{
	nassertr_always((index >= 0) && (index < (int ) _values.size()), Type())

	return _values[index];
}

/**
 * Returns the nth value in the collection.  This is the same as
 * get_value(), but it may be a more convenient way to access it.
 */
template<typename Type>
INLINE Type ValueList<Type>::operator [](int index) const
{
	nassertr_always((index >= 0) && (index < (int ) _values.size()), Type())

	return _values[index];
}

/**
 * Returns the number of values in the collection.  This is the same thing as
 * get_num_values().
 */
template<typename Type>
INLINE int ValueList<Type>::size() const
{
	return _values.size();
}

/**
 * Appends the other list onto the end of this one.
 */
template<typename Type>
INLINE void ValueList<Type>::operator +=(const ValueList &other)
{
	add_values_from(other);
}

/**
 * Returns a ValueList representing the concatenation of the two
 * lists.
 */
template<typename Type>
INLINE ValueList<Type> ValueList<Type>::operator +(const ValueList &other) const
{
	ValueList a(*this);
	a += other;
	return a;
}

/**
 * Removes the indicated value from the collection.  Returns true if
 * the value was removed, false if it was not a member of the collection.
 */
template<typename Type>
bool ValueList<Type>::remove_value(const Type& value)
{
	typename pvector<Type>::const_iterator iter = find(_values.begin(),
			_values.end(), value);
	CONTINUE_IF_ELSE_R(iter != _values.end(), false)

	_values.erase(iter);
	return true;
}

/**
 * Adds all the values indicated in the other collection to this value.
 * The other values are simply appended to the end of the values in this list;
 * duplicates are not automatically removed.
 */
template<typename Type>
void ValueList<Type>::add_values_from(const ValueList &other)
{
	int other_num_values = other.get_num_values();
	for (int i = 0; i < other_num_values; i++)
	{
		add_value(other.get_value(i));
	}
}

/**
 * Removes from this collection all of the values listed in the other
 * collection.
 */
template<typename Type>
void ValueList<Type>::remove_values_from(const ValueList& other)
{
	pvector<Type> new_values;
	int num_values = get_num_values();
	for (int i = 0; i < num_values; i++)
	{
		const Type& value = get_value(i);
		if (!other.has_value(value))
		{
			new_values.push_back(value);
		}
	}
	_values = new_values;
}

/**
 * Returns true if the indicated value appears in this collection,
 * false otherwise.
 */
template<typename Type>
bool ValueList<Type>::has_value(const Type& value) const
{
	typename pvector<Type>::const_iterator iter = find(_values.begin(),
			_values.end(), value);

	return iter != _values.end() ? true : false;
}

#ifndef CPPPARSER
template<typename Type>
inline ValueList<Type>::operator plist<Type>() const
{
	plist<Type> valueList;
	for (typename pvector<Type>::size_type i = 0; i < _values.size(); ++i)
	{
		valueList.push_back(_values[i]);
	}
	return valueList;
}
template<typename Type>
inline ValueList<Type>::operator pvector<Type>() const
{
	return _values;
}
#endif //CPPPARSER

///Vehicle settings.
INLINE float OSVehicleSettings::get_mass() const
{
	return _vehicleSettings.m_mass;
}
INLINE void OSVehicleSettings::set_mass(float value)
{
	_vehicleSettings.m_mass = value;
}
INLINE float OSVehicleSettings::get_radius() const
{
	return _vehicleSettings.m_radius;
}
INLINE void OSVehicleSettings::set_radius(float value)
{
	_vehicleSettings.m_radius = value;
}
INLINE float OSVehicleSettings::get_speed() const
{
	return _vehicleSettings.m_speed;
}
INLINE void OSVehicleSettings::set_speed(float value)
{
	_vehicleSettings.m_speed = value;
}
INLINE float OSVehicleSettings::get_maxForce() const
{
	return _vehicleSettings.m_maxForce;
}
INLINE void OSVehicleSettings::set_maxForce(float value)
{
	_vehicleSettings.m_maxForce = value;
}
INLINE float OSVehicleSettings::get_maxSpeed() const
{
	return _vehicleSettings.m_maxSpeed;
}
INLINE void OSVehicleSettings::set_maxSpeed(float value)
{
	_vehicleSettings.m_maxSpeed = value;
}
INLINE LVector3f OSVehicleSettings::get_forward() const
{
	return ossup::OpenSteerVec3ToLVecBase3f(_vehicleSettings.m_forward);
}
INLINE void OSVehicleSettings::set_forward(const LVector3f& value)
{
	_vehicleSettings.m_forward = ossup::LVecBase3fToOpenSteerVec3(value);
}
INLINE LVector3f OSVehicleSettings::get_side() const
{
	return ossup::OpenSteerVec3ToLVecBase3f(_vehicleSettings.m_side);
}
INLINE void OSVehicleSettings::set_side(const LVector3f& value)
{
	_vehicleSettings.m_side = ossup::LVecBase3fToOpenSteerVec3(value);
}
INLINE LVector3f OSVehicleSettings::get_up() const
{
	return ossup::OpenSteerVec3ToLVecBase3f(_vehicleSettings.m_up);
}
INLINE void OSVehicleSettings::set_up(const LVector3f& value)
{
	_vehicleSettings.m_up = ossup::LVecBase3fToOpenSteerVec3(value);
}
INLINE LVector3f OSVehicleSettings::get_position() const
{
	return ossup::OpenSteerVec3ToLVecBase3f(_vehicleSettings.m_position);
}
INLINE void OSVehicleSettings::set_position(const LVector3f& value)
{
	_vehicleSettings.m_position = ossup::LVecBase3fToOpenSteerVec3(value);
}

///OSObstacleSettings.
INLINE bool OSObstacleSettings::operator==(
		const OSObstacleSettings &other) const
{
	return (_type == other._type) && (_seenFromState == other._seenFromState)
			&& (_position == other._position) && (_forward == other._forward)
			&& (_up == other._up) && (_side == other._side)
			&& (_width == other._width) && (_height == other._height)
			&& (_depth == other._depth) && (_radius == other._radius)
			&& (_ref == other._ref) && (_obstacle == other._obstacle);
}
INLINE string OSObstacleSettings::get_type() const
{
	return _type;
}
INLINE void OSObstacleSettings::set_type(const string& value)
{
	_type = value;
}
INLINE string OSObstacleSettings::get_seenFromState() const
{
	return _seenFromState;
}
INLINE void OSObstacleSettings::set_seenFromState(const string& value)
{
	_seenFromState = value;
}
INLINE LPoint3f OSObstacleSettings::get_position() const
{
	return _position;
}
INLINE void OSObstacleSettings::set_position(const LPoint3f& value)
{
	_position = value;
}
INLINE LVector3f OSObstacleSettings::get_forward() const
{
	return _forward;
}
INLINE void OSObstacleSettings::set_forward(const LVector3f& value)
{
	_forward = value;
}
INLINE LVector3f OSObstacleSettings::get_up() const
{
	return _up;
}
INLINE void OSObstacleSettings::set_up(const LVector3f& value)
{
	_up = value;
}
INLINE LVector3f OSObstacleSettings::get_side() const
{
	return _side;
}
INLINE void OSObstacleSettings::set_side(const LVector3f& value)
{
	_side = value;
}
INLINE float OSObstacleSettings::get_width() const
{
	return _width;
}
INLINE void OSObstacleSettings::set_width(float value)
{
	_width = value;
}
INLINE float OSObstacleSettings::get_height() const
{
	return _height;
}
INLINE void OSObstacleSettings::set_height(float value)
{
	_height = value;
}
INLINE float OSObstacleSettings::get_depth() const
{
	return _depth;
}
INLINE void OSObstacleSettings::set_depth(float value)
{
	_depth = value;
}
INLINE float OSObstacleSettings::get_radius() const
{
	return _radius;
}
INLINE void OSObstacleSettings::set_radius(float value)
{
	_radius = value;
}
INLINE int OSObstacleSettings::get_ref() const
{
	return _ref;
}
INLINE void OSObstacleSettings::set_ref(int value)
{
	_ref = value;
}
inline OpenSteer::AbstractObstacle* OSObstacleSettings::get_obstacle() const
{
	return _obstacle;
}
inline void OSObstacleSettings::set_obstacle(OpenSteer::AbstractObstacle* value)
{
	_obstacle = value;
}

#endif /* OSTOOLS_I_ */
