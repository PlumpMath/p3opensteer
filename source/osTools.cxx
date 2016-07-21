/**
 * \file osTools.cxx
 *
 * \date 2016-05-13
 * \author consultit
 */

#include "osTools.h"

pvector<string> parseCompoundString(
		const string& srcCompoundString, char separator)
{
	//erase blanks
	string compoundString = srcCompoundString;
	compoundString = eraseCharacter(compoundString, ' ');
	compoundString = eraseCharacter(compoundString, '\t');
	compoundString = eraseCharacter(compoundString, '\n');
	//parse
	pvector<string> substrings;
	int len = compoundString.size() + 1;
	char* dest = new char[len];
	strncpy(dest, compoundString.c_str(), len);
	//find
	char* pch;
	char* start = dest;
	bool stop = false;
	while (! stop)
	{
		string substring("");
		pch = strchr(start, separator);
		if (pch != NULL)
		{
			//insert the substring
			substring.append(start, pch - start);
			start = pch + 1;
			substrings.push_back(substring);
		}
		else
		{
			if (start < &dest[len - 1])
			{
				//insert the last not empty substring
				substring.append(start, &dest[len - 1] - start);
				substrings.push_back(substring);
			}
			else if (start == &dest[len - 1])
			{
				//insert the last empty substring
				substrings.push_back(substring);
			}
			stop = true;
		}
	}
	delete[] dest;
	//
	return substrings;
}

string eraseCharacter(const string& source, int character)
{
	int len = source.size() + 1;
	char* dest = new char[len];
	char* start = dest;
	strncpy(dest, source.c_str(), len);
	//erase
	char* pch;
	pch = strchr(dest, character);
	while (pch != NULL)
	{
		len -= pch - start;
		memmove(pch, pch + 1, len - 1);
		start = pch;
		//continue
		pch = strchr(pch, character);
	}
	string outStr(dest);
	delete[] dest;
	return outStr;
}

string replaceCharacter(const string& source, int character,
		int replacement)
{
	int len = source.size() + 1;
	char* dest = new char[len];
	strncpy(dest, source.c_str(), len);
	//replace hyphens
	char* pch;
	pch = strchr(dest, character);
	while (pch != NULL)
	{
		*pch = replacement;
		pch = strchr(pch + 1, character);
	}
	string outStr(dest);
	delete[] dest;
	return outStr;
}

///ThrowEventData
void ThrowEventData::write_datagram(Datagram &dg) const
{
	dg.add_bool(mEnable);
	dg.add_string(mEventName);
	dg.add_bool(mThrown);
	dg.add_stdfloat(mTimeElapsed);
	dg.add_uint32(mCount);
	dg.add_stdfloat(mFrequency);
	dg.add_stdfloat(mPeriod);
}

void ThrowEventData::read_datagram(DatagramIterator &scan)
{
	mEnable = scan.get_bool();
	mEventName = scan.get_string();
	mThrown = scan.get_bool();
	mTimeElapsed = scan.get_stdfloat();
	mCount = scan.get_uint32();
	mFrequency = scan.get_stdfloat();
	mPeriod = scan.get_stdfloat();
}

///Vehicle settings.
/**
 *
 */
OSVehicleSettings::OSVehicleSettings(): _vehicleSettings()
{
}
/**
 * Writes the Vehicle settings into a datagram.
 */
void OSVehicleSettings::write_datagram(Datagram &dg) const
{
	dg.add_stdfloat(get_mass());
	dg.add_stdfloat(get_radius());
	dg.add_stdfloat(get_speed());
	dg.add_stdfloat(get_maxForce());
	dg.add_stdfloat(get_maxSpeed());
	get_forward().write_datagram(dg);
	get_side().write_datagram(dg);
	get_up().write_datagram(dg);
	get_position().write_datagram(dg);
}

/**
 * Restores the Vehicle settings from the datagram.
 */
void OSVehicleSettings::read_datagram(DatagramIterator &scan)
{
	set_mass(scan.get_stdfloat());
	set_radius(scan.get_stdfloat());
	set_speed(scan.get_stdfloat());
	set_maxForce(scan.get_stdfloat());
	set_maxSpeed(scan.get_stdfloat());
	LVector3f value;
	value.read_datagram(scan);
	set_forward(value);
	value.read_datagram(scan);
	set_side(value);
	value.read_datagram(scan);
	set_up(value);
	value.read_datagram(scan);
	set_position(value);
}

///Flock settings.
/**
 *
 */
OSFlockSettings::OSFlockSettings():
		_separationRadius(0.0), _separationAngle(0.0), _separationWeight(0.0),
		_alignmentRadius(0.0), _alignmentAngle(0.0), _alignmentWeight(0.0),
		_cohesionRadius(0.0), _cohesionAngle(0.0), _cohesionWeight(0.0)
{
}/**
 *
 */
OSFlockSettings::OSFlockSettings(float sR, float sA, float sW, float aR,
		float aA, float aW, float cR, float cA, float cW) :
		_separationRadius(sR), _separationAngle(sA), _separationWeight(sW),
		_alignmentRadius(aR), _alignmentAngle(aA), _alignmentWeight(aW),
		_cohesionRadius(cR), _cohesionAngle(cA), _cohesionWeight(cW)
{
}
/**
 * Writes the flock settings into a datagram.
 */
void OSFlockSettings::write_datagram(Datagram &dg) const
{
	dg.add_stdfloat(get_separation_radius());
	dg.add_stdfloat(get_separation_angle());
	dg.add_stdfloat(get_separation_weight());
	dg.add_stdfloat(get_alignment_radius());
	dg.add_stdfloat(get_alignment_angle());
	dg.add_stdfloat(get_alignment_weight());
	dg.add_stdfloat(get_cohesion_radius());
	dg.add_stdfloat(get_cohesion_angle());
	dg.add_stdfloat(get_cohesion_weight());
}

/**
 * Restores the flock settings from the datagram.
 */
void OSFlockSettings::read_datagram(DatagramIterator &scan)
{
	set_separation_radius(scan.get_stdfloat());
	set_separation_angle(scan.get_stdfloat());
	set_separation_weight(scan.get_stdfloat());
	set_alignment_radius(scan.get_stdfloat());
	set_alignment_angle(scan.get_stdfloat());
	set_alignment_weight(scan.get_stdfloat());
	set_cohesion_radius(scan.get_stdfloat());
	set_cohesion_angle(scan.get_stdfloat());
	set_cohesion_weight(scan.get_stdfloat());
}

///OSObstacleSettings.
/**
 *
 */
OSObstacleSettings::OSObstacleSettings() :
		_width(0), _height(0), _depth(0), _radius(0), _ref(0), _obstacle(0)
{
}
/**
 * Writes the OSObstacleSettings into a datagram.
 */
void OSObstacleSettings::write_datagram(Datagram &dg) const
{
	dg.add_string(_type);
	dg.add_string(_seenFromState);
	_position.write_datagram(dg);
	_forward.write_datagram(dg);
	_up.write_datagram(dg);
	_side.write_datagram(dg);
	dg.add_stdfloat(get_width());
	dg.add_stdfloat(get_height());
	dg.add_stdfloat(get_depth());
	dg.add_stdfloat(get_radius());
	dg.add_int32(get_ref());
}
/**
 * Restores the OSObstacleSettings from the datagram.
 */
void OSObstacleSettings::read_datagram(DatagramIterator &scan)
{
	set_type(scan.get_string());
	set_seenFromState(scan.get_string());
	_position.read_datagram(scan);
	_forward.read_datagram(scan);
	_up.read_datagram(scan);
	_side.read_datagram(scan);
	set_width(scan.get_stdfloat());
	set_height(scan.get_stdfloat());
	set_depth(scan.get_stdfloat());
	set_radius(scan.get_stdfloat());
	set_ref(scan.get_int32());
}

///ValueList template
// Tell GCC that we'll take care of the instantiation explicitly here.
#ifdef __GNUC__
#pragma implementation
#endif

template class ValueList<string>;
template class ValueList<LPoint3f>;
template struct Pair<bool,float>;
