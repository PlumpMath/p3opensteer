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

#endif /* OSTOOLS_I_ */
