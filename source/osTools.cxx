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
	while (not stop)
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

///ValueList template
// Tell GCC that we'll take care of the instantiation explicitly here.
#ifdef __GNUC__
#pragma implementation
#endif

template class ValueList<string>;
template class ValueList<LPoint3f>;
template struct Pair<bool,float>;
