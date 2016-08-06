// ----------------------------------------------------------------------------
//
//
// OpenSteer -- Steering Behaviors for Autonomous Characters
//
// Copyright (c) 2002-2005, Sony Computer Entertainment America
// Original author: Craig Reynolds <craig_reynolds@playstation.sony.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
//
// ----------------------------------------------------------------------------
//
//
// Driving through map-based obstacles    (OpenSteerDemo PlugIn)
//
// This demonstration is inspired by the DARPA Grand Challenge cross country
// race for autonomous vehicles (http://www.darpa.mil/grandchallenge/).  A
// route is defined as a series of (GPS) waypoints and a width associated
// with each segment between waypoints.  This demo assumes sensors on-board
// the vehicle create a binary map classifying the surrounding terrain into
// drivable and not drivable.  The vehicle tries to follow the route while
// avoiding obstacles and maximizing speed.  When the vehicle finds itself
// in danger of collision, it "gives up" (turns yellow) and slows to a stop.
// If it collides with an obstacle it turns red.  In both cases the
// simulation is restarted.  (This plug-in includes two non-path-following
// demos of map-based obstacle avoidance.  Use F1 to select among them.)
//
// 06-01-05 bknafla: exchanged GCRoute with PolylineSegmentedPathwaySegmentRadii
// 08-16-04 cwr: merge back into OpenSteer code base
// 10-15-03 cwr: created 
//
//
// ----------------------------------------------------------------------------
/**
 * \file /Ely/include/Support/OpenSteerLocal/PlugIn_MapDrive.h
 *
 * \date 2016-05-13
 * \author Craig Reynolds (modified by consultit)
 */

#ifndef PLUGIN_MAPDRIVE_H_
#define PLUGIN_MAPDRIVE_H_

#include <iomanip>
#include <sstream>
#include <cassert>
#include <cmath>
#include <OpenSteer/SimpleVehicle.h>
#include <OpenSteer/Color.h>
#include <OpenSteer/UnusedParameter.h>
#include <OpenSteer/PlugIn.h>
// Include OpenSteer::PolylineSegmentedPathwaySegmentRadii
#include <OpenSteer/PolylineSegmentedPathwaySegmentRadii.h>
// Include OpenSteer::mapPointToPathway
#include <OpenSteer/QueryPathAlike.h>
// Include OpenSteer::DontExtractPathDistance, OpenSteer::HasSegmentRadii
#include <OpenSteer/QueryPathAlikeUtilities.h>
// Include OpenSteer::nextSegment, OpenSteer::previousSegment
#include <OpenSteer/SegmentedPathAlikeUtilities.h>
// Include OpenSteer::square, OpenSteer::clamp
#include <OpenSteer/Utilities.h>
// Include OpenSteer::size_t
#include <OpenSteer/StandardTypes.h>
#include "common.h"

// to use local version of the map class
#define OLDTERRAINMAP
#ifndef OLDTERRAINMAP
#include "OpenSteer/TerrainMap.h"
#endif

// ----------------------------------------------------------------------------

///extern float windowWidth;

namespace ossup
{

using namespace OpenSteer;

class PointToRadiusMapping: public OpenSteer::DontExtractPathDistance
{
public:
	PointToRadiusMapping() :
			radius(0.0f)
	{
	}

	void setPointOnPathCenterLine(OpenSteer::Vec3 const&)
	{
	}
	void setPointOnPathBoundary(OpenSteer::Vec3 const&)
	{
	}
	void setRadius(float r)
	{
		radius = r;
	}
	void setTangent(OpenSteer::Vec3 const&)
	{
	}
	void setSegmentIndex(OpenSteer::size_t)
	{
	}
	void setDistancePointToPath(float)
	{
	}
	void setDistancePointToPathCenterLine(float)
	{
	}
	void setDistanceOnPath(float)
	{
	}
	void setDistanceOnSegment(float)
	{
	}

	float radius;
};

class PointToTangentMapping: public OpenSteer::DontExtractPathDistance
{
public:
	PointToTangentMapping() :
			tangent(OpenSteer::Vec3(0.0f, 0.0f, 0.0f))
	{
	}

	void setPointOnPathCenterLine(OpenSteer::Vec3 const&)
	{
	}
	void setPointOnPathBoundary(OpenSteer::Vec3 const&)
	{
	}
	void setRadius(float)
	{
	}
	void setTangent(OpenSteer::Vec3 const& t)
	{
		tangent = t;
	}
	void setSegmentIndex(OpenSteer::size_t)
	{
	}
	void setDistancePointToPath(float)
	{
	}
	void setDistancePointToPathCenterLine(float)
	{
	}
	void setDistanceOnPath(float)
	{
	}
	void setDistanceOnSegment(float)
	{
	}

	OpenSteer::Vec3 tangent;
};

class PointToPointOnCenterLineAndOutsideMapping: public OpenSteer::DontExtractPathDistance
{
public:
	PointToPointOnCenterLineAndOutsideMapping() :
			pointOnPathCenterLine(OpenSteer::Vec3(0.0f, 0.0f, 0.0f)), distancePointToPathBoundary(
					0.0f)
	{
	}

	void setPointOnPathCenterLine(OpenSteer::Vec3 const& point)
	{
		pointOnPathCenterLine = point;
	}
	void setPointOnPathBoundary(OpenSteer::Vec3 const&)
	{
	}
	void setRadius(float)
	{
	}
	void setTangent(OpenSteer::Vec3 const&)
	{
	}
	void setSegmentIndex(OpenSteer::size_t)
	{
	}
	void setDistancePointToPath(float d)
	{
		distancePointToPathBoundary = d;
	}
	void setDistancePointToPathCenterLine(float)
	{
	}
	void setDistanceOnPath(float)
	{
	}
	void setDistanceOnSegment(float)
	{
	}

	OpenSteer::Vec3 pointOnPathCenterLine;
	float distancePointToPathBoundary;
};

class PointToOutsideMapping: public OpenSteer::DontExtractPathDistance
{
public:
	PointToOutsideMapping() :
			distancePointToPathBoundary(0.0f)
	{
	}

	void setPointOnPathCenterLine(OpenSteer::Vec3 const&)
	{
	}
	void setPointOnPathBoundary(OpenSteer::Vec3 const&)
	{
	}
	void setRadius(float)
	{
	}
	void setTangent(OpenSteer::Vec3 const&)
	{
	}
	void setSegmentIndex(OpenSteer::size_t)
	{
	}
	void setDistancePointToPath(float d)
	{
		distancePointToPathBoundary = d;
	}
	void setDistancePointToPathCenterLine(float)
	{
	}
	void setDistanceOnPath(float)
	{
	}
	void setDistanceOnSegment(float)
	{
	}

	float distancePointToPathBoundary;
};

class PointToSegmentIndexMapping: public OpenSteer::DontExtractPathDistance
{
public:
	PointToSegmentIndexMapping() :
			segmentIndex(0)
	{
	}

	void setPointOnPathCenterLine(OpenSteer::Vec3 const&)
	{
	}
	void setPointOnPathBoundary(OpenSteer::Vec3 const&)
	{
	}
	void setRadius(float)
	{
	}
	void setTangent(OpenSteer::Vec3 const&)
	{
	}
	void setSegmentIndex(OpenSteer::size_t i)
	{
		segmentIndex = i;
	}
	void setDistancePointToPath(float)
	{
	}
	void setDistancePointToPathCenterLine(float)
	{
	}
	void setDistanceOnPath(float)
	{
	}
	void setDistanceOnSegment(float)
	{
	}

	OpenSteer::size_t segmentIndex;
};

/**
 * Maps @a point to @a pathway and extracts the radius at the mapping point.
 */
inline float mapPointToRadius(
		OpenSteer::PolylineSegmentedPathwaySegmentRadii const& pathway,
		OpenSteer::Vec3 const& point)
{
	PointToRadiusMapping mapping;
	OpenSteer::mapPointToPathAlike(pathway, point, mapping);
	return mapping.radius;
}

/**
 * Maps @a point to @a pathway and extracts the tangent at the mapping
 * point.
 */
inline OpenSteer::Vec3 mapPointToTangent(
		OpenSteer::PolylineSegmentedPathwaySegmentRadii const& pathway,
		OpenSteer::Vec3 const& point)
{
	PointToTangentMapping mapping;
	OpenSteer::mapPointToPathAlike(pathway, point, mapping);
	return mapping.tangent;
}

/**
 * Returns @c true if @a point is inside @a pathway segment @a segmentIndex.
 *
 * On point on the boundary isn't inside the pathway.
 */
inline bool isInsidePathSegment(
		OpenSteer::PolylineSegmentedPathwaySegmentRadii const& pathway,
		OpenSteer::PolylineSegmentedPathwaySegmentRadii::size_type segmentIndex,
		OpenSteer::Vec3 const& point)
{
#ifdef OS_DEBUG
	assert(pathway.isValid() && "pathway isn't valid.");
	assert(
			segmentIndex < pathway.segmentCount()
					&& "segmentIndex out of range.");
#endif

	float const segmentDistance = pathway.mapPointToSegmentDistance(
			segmentIndex, point);
	OpenSteer::Vec3 const pointOnSegmentCenterLine =
			pathway.mapSegmentDistanceToPoint(segmentIndex, segmentDistance);
	float const segmentRadiusAtPoint = pathway.mapSegmentDistanceToRadius(
			segmentIndex, segmentDistance);

	float const distancePointToPointOnSegmentCenterLine = (point
			- pointOnSegmentCenterLine).length();

	return distancePointToPointOnSegmentCenterLine < segmentRadiusAtPoint;
}

/**
 * Maps the @a point to @a pathway and extracts the tangent at the mapping
 * point or of the next path segment as indicated by @a direction if the
 * mapping point is near a path defining point (waypoint).
 *
 * @param pathway Pathway to inspect.
 * @param point Point to map to @a pathway.
 * @param direction Follow the path in path direction (@c 1) or in reverse
 *                  direction ( @c -1 ).
 */
inline OpenSteer::Vec3 mapPointAndDirectionToTangent(
		OpenSteer::PolylineSegmentedPathwaySegmentRadii const& pathway,
		OpenSteer::Vec3 const& point, int direction)
{
#ifdef OS_DEBUG
	assert(
			((1 == direction) || (-1 == direction))
					&& "direction must be 1 or -1.");
#endif
	typedef OpenSteer::PolylineSegmentedPathwaySegmentRadii::size_type size_type;

	PointToSegmentIndexMapping mapping;
	OpenSteer::mapPointToPathAlike(pathway, point, mapping);
	size_type segmentIndex = mapping.segmentIndex;
	size_type nextSegmentIndex = segmentIndex;
	if (0 < direction)
	{
		nextSegmentIndex = OpenSteer::nextSegment(pathway, segmentIndex);
	}
	else
	{
		nextSegmentIndex = OpenSteer::previousSegment(pathway, segmentIndex);
	}

	if (isInsidePathSegment(pathway, nextSegmentIndex, point))
	{
		segmentIndex = nextSegmentIndex;
	}

	// To save calculations to gather the tangent in a sound way the fact is
	// used that a polyline segmented pathway has the same tangent for a
	// whole segment.
	return pathway.mapSegmentDistanceToTangent(segmentIndex, 0.0f)
			* static_cast<float>(direction);

	/*
	 const int segmentIndex = indexOfNearestSegment (point);
	 const int nextIndex = segmentIndex + pathFollowDirection;
	 const bool insideNextSegment = isInsidePathSegment (point, nextIndex);
	 const int i = (segmentIndex +
	 (insideNextSegment ? pathFollowDirection : 0));
	 return normals [i] * (float)pathFollowDirection;
	 */
}

/**
 * Returns @c true if @a point is near a waypoint of @a pathway.
 *
 * It is near if its distance to a waypoint of the path is lesser than the
 * radius of one of the segments that the waypoint belongs to.
 *
 * On point on the boundary isn't inside the pathway.
 */
inline bool isNearWaypoint(
		OpenSteer::PolylineSegmentedPathwaySegmentRadii const& pathway,
		OpenSteer::Vec3 const& point)
{
#ifdef OS_DEBUG
	assert(pathway.isValid() && "pathway must be valid.");
#endif

	typedef OpenSteer::PolylineSegmentedPathwaySegmentRadii::size_type size_type;

	size_type pointIndex = 0;

	// Test first waypoint.
	OpenSteer::Vec3 pointPathwayPointVector = point - pathway.point(pointIndex);
	float pointPathwayPointDistance = pointPathwayPointVector.dot(
			pointPathwayPointVector);
	if (pointPathwayPointDistance
			< OpenSteer::square(pathway.segmentRadius(pointIndex)))
	{
		return true;
	}

	// Test other waypoints.
	size_type const maxInnerPointIndex = pathway.pointCount() - 2;
	for (pointIndex = 1; pointIndex <= maxInnerPointIndex; ++pointIndex)
	{
		pointPathwayPointVector = point - pathway.point(pointIndex);
		pointPathwayPointDistance = pointPathwayPointVector.dot(
				pointPathwayPointVector);
		if ((pointPathwayPointDistance
				< OpenSteer::square(pathway.segmentRadius(pointIndex)))
				|| (pointPathwayPointDistance
						< OpenSteer::square(
								pathway.segmentRadius(pointIndex - 1))))
		{
			return true;
		}
	}

	// Test last waypoint.
	pointPathwayPointVector = point - pathway.point(pointIndex);
	pointPathwayPointDistance = pointPathwayPointVector.dot(
			pointPathwayPointVector);
	if (pointPathwayPointDistance
			< OpenSteer::square(pathway.segmentRadius(pointIndex - 1)))
	{
		return true;
	}

	return false;

	/*
	 // loop over all waypoints
	 for (int i = 1; i < pointCount; i++)
	 {
	 // return true if near enough to this waypoint
	 const float r = maxXXX (radii[i], radii[i+1]);
	 const float d = (point - points[i]).length ();
	 if (d < r) return true;
	 }
	 return false;
	 */
}

/**
 * Maps @a point to @a pathway and returns the mapping point on the pathway
 * boundary and how far outside @a point is from the mapping point.
 */
inline OpenSteer::Vec3 mapPointToPointOnCenterLineAndOutside(
		OpenSteer::PolylineSegmentedPathwaySegmentRadii const& pathway,
		OpenSteer::Vec3 const& point, float& outside)
{
	PointToPointOnCenterLineAndOutsideMapping mapping;
	OpenSteer::mapPointToPathAlike(pathway, point, mapping);
	outside = mapping.distancePointToPathBoundary;
	return mapping.pointOnPathCenterLine;
}

/**
 * Maps @a point to @a pathway and returns how far outside @a point is from
 * the mapping point on the path boundary.
 */
inline float mapPointToOutside(
		OpenSteer::PolylineSegmentedPathwaySegmentRadii const& pathway,
		OpenSteer::Vec3 const& point)
{
	PointToOutsideMapping mapping;
	OpenSteer::mapPointToPathAlike(pathway, point, mapping);
	return mapping.distancePointToPathBoundary;
}

/**
 * Returns @c true if @a point is inside @a pathway, @c false otherwise.
 * A point on the boundary isn't inside the pathway.
 */
inline bool isInsidePathway(
		OpenSteer::PolylineSegmentedPathwaySegmentRadii const& pathway,
		OpenSteer::Vec3 const& point)
{
	return 0.0f > mapPointToOutside(pathway, point);
}

inline OpenSteer::PolylineSegmentedPathwaySegmentRadii::size_type mapPointToSegmentIndex(
		OpenSteer::PolylineSegmentedPathwaySegmentRadii const& pathway,
		OpenSteer::Vec3 const& point)
{
	PointToSegmentIndexMapping mapping;
	OpenSteer::mapPointToPathAlike(pathway, point, mapping);
	return mapping.segmentIndex;
}

// random utility, worth moving to Utilities.h?
inline int irandom2(int min, int max)
{
	return (int) frandom2((float) min, (float) max);
}

///// define map size (and compute its half diagonal)
///float MapDriver::worldSize = 200;
///float MapDriver::worldDiag = sqrtXXX(square(worldSize) / 2);
///// 0 = obstacle avoidance and speed control
///// 1 = wander, obstacle avoidance and speed control
///// 2 = path following, obstacle avoidance and speed control
///// int MapDriver::demoSelect = 0;
///int MapDriver::demoSelect = 2;
///float MapDriver::savedNearestWR = 0;
///float MapDriver::savedNearestR = 0;
///float MapDriver::savedNearestL = 0;
///float MapDriver::savedNearestWL = 0;

#ifdef OLDTERRAINMAP
// class BinaryTerrainMap : public TerrainMap
class TerrainMap
{
public:

	// constructor
	TerrainMap(const Vec3& c, float x, float z, int r) :
			center(c), xSize(x), zSize(z), resolution(r), outsideValue(false), map(
					resolution * resolution)
	{
		map.reserve(resolution * resolution);
		if (xSize < 0)
		{
			xSize = -xSize;
		}
		if (zSize < 0)
		{
			zSize = -zSize;
		}
	}

	// destructor
	~TerrainMap()
	{
	}

	// clear the map (to false)
	void clear(void)
	{
		for (int i = 0; i < resolution; i++)
			for (int j = 0; j < resolution; j++)
				setMapBit(i, j, 0);
	}

	// get and set a bit based on 2d integer map index
	bool getMapBit(int i, int j) const
	{
		return map[mapAddress(i, j)];
	}

	bool setMapBit(int i, int j, bool value)
	{
		return map[mapAddress(i, j)] = value;
	}

	// get a value based on a position in 3d world space
	bool getMapValue(const Vec3& point) const
	{
		const Vec3 local = point - center;
		const Vec3 localXZ = local.setYtoZero();

		const float hxs = xSize / 2;
		const float hzs = zSize / 2;

		const float x = localXZ.x;
		const float z = localXZ.z;

		const bool out = (x > +hxs) || (x < -hxs) || (z > +hzs) || (z < -hzs);

		if (out)
		{
			return outsideValue;
		}
		else
		{
			const float r = (float) resolution; // prevent VC7.1 warning
			const int i = (int) remapInterval(x, -hxs, hxs, 0.0f, r);
			const int j = (int) remapInterval(z, -hzs, hzs, 0.0f, r);
			return getMapBit(i, j);
		}
	}

	bool isInside(const Vec3& point) const
	{
		const Vec3 local = point - center;
		const Vec3 localXZ = local.setYtoZero();

		const float hxs = xSize / 2;
		const float hzs = zSize / 2;

		const float x = localXZ.x;
		const float z = localXZ.z;

		return (x < +hxs) && (x > -hxs) && (z < +hzs) && (z > -hzs);
	}

	void getCoords(const Vec3& point, int& i, int& j)
	{
		const Vec3 local = point - center;
		const Vec3 localXZ = local.setYtoZero();

		const float hxs = xSize / 2;
		const float hzs = zSize / 2;

		const float x = localXZ.x;
		const float z = localXZ.z;

		i = (int) remapInterval(x, -hxs, hxs, 0.0f, resolution);
		j = (int) remapInterval(z, -hzs, hzs, 0.0f, resolution);
	}

#ifdef OS_DEBUG
	void xxxDrawMap(void)
	{
		const float xs = xSize / (float) resolution;
		const float zs = zSize / (float) resolution;
		const Vec3 alongRow(xs, 0, 0);
		const Vec3 nextRow(-xSize, 0, zs);
		Vec3 g((xSize - xs) / -2, 0, (zSize - zs) / -2);
		g += center;
		for (int j = 0; j < resolution; j++)
		{
			for (int i = 0; i < resolution; i++)
			{
				if (getMapBit(i, j))
				{
					// spikes
					// const Vec3 spikeTop (0, 5.0f, 0);
					// drawLine (g, g+spikeTop, gWhite);

					// squares
					const float rockHeight = 0;
					const Vec3 v1(+xs / 2, rockHeight, +zs / 2);
					const Vec3 v2(+xs / 2, rockHeight, -zs / 2);
					const Vec3 v3(-xs / 2, rockHeight, -zs / 2);
					const Vec3 v4(-xs / 2, rockHeight, +zs / 2);
					// const Vec3 redRockColor (0.6f, 0.1f, 0.0f);
					const Color orangeRockColor(0.5f, 0.2f, 0.0f);
					drawQuadrangle(g + v1, g + v2, g + v3, g + v4,
							orangeRockColor);

					// pyramids
					// const Vec3 top (0, xs/2, 0);
					// const Vec3 redRockColor (0.6f, 0.1f, 0.0f);
					// const Vec3 orangeRockColor (0.5f, 0.2f, 0.0f);
					// drawTriangle (g+v1, g+v2, g+top, redRockColor);
					// drawTriangle (g+v2, g+v3, g+top, orangeRockColor);
					// drawTriangle (g+v3, g+v4, g+top, redRockColor);
					// drawTriangle (g+v4, g+v1, g+top, orangeRockColor);
				}
				g += alongRow;
			}
			g += nextRow;
		}
	}
#endif

	float minSpacing(void) const
	{
		return minXXX(xSize, zSize) / (float) resolution;
	}

	// used to detect if vehicle body is on any obstacles
	bool scanLocalXZRectangle(const AbstractLocalSpace& localSpace, float xMin,
			float xMax, float zMin, float zMax) const
	{
		const float spacing = minSpacing() / 2;

		for (float x = xMin; x < xMax; x += spacing)
		{
			for (float z = zMin; z < zMax; z += spacing)
			{
				const Vec3 sample(x, 0, z);
				const Vec3 global = localSpace.globalizePosition(sample);
				if (getMapValue(global))
					return true;
			}
		}
		return false;
	}

	// Scans along a ray (directed line segment) on the XZ plane, sampling
	// the map for a "true" cell.  Returns the index of the first sample
	// that gets a "hit", or zero if no hits found.
	int scanXZray(const Vec3& origin, const Vec3& sampleSpacing,
			const int sampleCount) const
	{
		Vec3 samplePoint(origin);

		for (int i = 1; i <= sampleCount; i++)
		{
			samplePoint += sampleSpacing;
			if (getMapValue(samplePoint))
				return i;
		}

		return 0;
	}

	int cellwidth(void) const
	{
		return resolution;
	}  // xxx cwr
	int cellheight(void) const
	{
		return resolution;
	}  // xxx cwr
	bool isPassable(const Vec3& point) const
	{
		return !getMapValue(point);
	}

	Vec3 center;
	float xSize;
	float zSize;
	int resolution;

	bool outsideValue;

private:

	int mapAddress(int i, int j) const
	{
		return i + (j * resolution);
	}

	std::vector<bool> map;
};
#endif

typedef PolylineSegmentedPathwaySegmentRadii GCRoute;
//typedef SegmentedPathway GCRoute;


/*
 * Use PolylineSegmentedPathwaySegmentRadii instead!


 // ----------------------------------------------------------------------------
 // A variation on PolylinePathway (whose path tube radius is constant)
 // GCRoute (Grand Challenge Route) has an array of radii-per-segment
 //
 // XXX The OpenSteer path classes are long overdue for a rewrite.  When
 // XXX that happens, support should be provided for constant-radius,
 // XXX radius-per-segment (as in GCRoute), and radius-per-vertex.


 class GCRoute : public PolylinePathway
 {
 public:

 // construct a GCRoute given the number of points (vertices), an
 // array of points, an array of per-segment path radii, and a flag
 // indiating if the path is connected at the end.
 GCRoute (const int _pointCount,
 const Vec3 _points[],
 const float _radii[],
 const bool _cyclic)
 {
 initialize (_pointCount, _points, _radii[0], _cyclic);

 radii = new float [pointCount];

 // loop over all points
 for (int i = 0; i < pointCount; i++)
 {
 // copy in point locations, closing cycle when appropriate
 const bool closeCycle = cyclic && (i == pointCount-1);
 const int j = closeCycle ? 0 : i;
 points[i] = _points[j];
 radii[i] = _radii[i];
 }
 }

 // override the PolylinePathway method to allow for GCRoute-style
 // per-leg radii

 // Given an arbitrary point ("A"), returns the nearest point ("P") on
 // this path.  Also returns, via output arguments, the path tangent at
 // P and a measure of how far A is outside the Pathway's "tube".  Note
 // that a negative distance indicates A is inside the Pathway.

 Vec3 mapPointToPath (const Vec3& point, Vec3& tangent, float& outside)
 {
 Vec3 onPath;
 outside = FLT_MAX;

 // loop over all segments, find the one nearest to the given point
 for (int i = 1; i < pointCount; i++)
 {
 // QQQ note bizarre calling sequence of pointToSegmentDistance
 segmentLength = lengths[i];
 segmentNormal = normals[i];
 const float d =pointToSegmentDistance(point,points[i-1],points[i]);

 // measure how far original point is outside the Pathway's "tube"
 // (negative values (from 0 to -radius) measure "insideness")
 const float o = d - radii[i];

 // when this is the smallest "outsideness" seen so far, take
 // note and save the corresponding point-on-path and tangent
 if (o < outside)
 {
 outside = o;
 onPath = chosen;
 tangent = segmentNormal;
 }
 }

 // return point on path
 return onPath;
 }

 // ignore that "tangent" output argument which is never used
 // XXX eventually move this to Pathway class
 Vec3 mapPointToPath (const Vec3& point, float& outside)
 {
 Vec3 tangent;
 return mapPointToPath (point, tangent, outside);
 }

 // get the index number of the path segment nearest the given point
 // XXX consider moving this to path class
 int indexOfNearestSegment (const Vec3& point)
 {
 int index = 0;
 float minDistance = FLT_MAX;

 // loop over all segments, find the one nearest the given point
 for (int i = 1; i < pointCount; i++)
 {
 segmentLength = lengths[i];
 segmentNormal = normals[i];
 float d = pointToSegmentDistance (point, points[i-1], points[i]);
 if (d < minDistance)
 {
 minDistance = d;
 index = i;
 }
 }
 return index;
 }

 // returns the dot product of the tangents of two path segments,
 // used to measure the "angle" at a path vertex: how sharp is the turn?
 float dotSegmentUnitTangents (int segmentIndex0, int segmentIndex1)
 {
 return normals[segmentIndex0].dot (normals[segmentIndex1]);
 }

 // return path tangent at given point (its projection on path)
 Vec3 tangentAt (const Vec3& point)
 {
 return normals [indexOfNearestSegment (point)];
 }

 // return path tangent at given point (its projection on path),
 // multiplied by the given pathfollowing direction (+1/-1 =
 // upstream/downstream).  Near path vertices (waypoints) use the
 // tangent of the "next segment" in the given direction
 Vec3 tangentAt (const Vec3& point, const int pathFollowDirection)
 {
 const int segmentIndex = indexOfNearestSegment (point);
 const int nextIndex = segmentIndex + pathFollowDirection;
 const bool insideNextSegment = isInsidePathSegment (point, nextIndex);
 const int i = (segmentIndex +
 (insideNextSegment ? pathFollowDirection : 0));
 return normals [i] * (float)pathFollowDirection;
 }

 // is the given point "near" a waypoint of this path?  ("near" == closer
 // to the waypoint than the max of radii of two adjacent segments)
 bool nearWaypoint (const Vec3& point)
 {
 // loop over all waypoints
 for (int i = 1; i < pointCount; i++)
 {
 // return true if near enough to this waypoint
 const float r = maxXXX (radii[i], radii[i+1]);
 const float d = (point - points[i]).length ();
 if (d < r) return true;
 }
 return false;
 }

 // is the given point inside the path tube of the given segment
 // number?  (currently not used. this seemed like a useful utility,
 // but wasn't right for the problem I was trying to solve)
 bool isInsidePathSegment (const Vec3& point, const int segmentIndex)
 {
 const int i = segmentIndex;

 // QQQ note bizarre calling sequence of pointToSegmentDistance
 segmentLength = lengths[i];
 segmentNormal = normals[i];
 const float d = pointToSegmentDistance(point, points[i-1], points[i]);

 // measure how far original point is outside the Pathway's "tube"
 // (negative values (from 0 to -radius) measure "insideness")
 const float o = d - radii[i];

 // return true if point is inside the tube
 return o < 0;
 }

 // per-segment radius (width) array
 float* radii;
 };

 */

/**
 * \note: Public class members for tweaking:
 * - \var pathFollowDirection: follow the path "upstream or downstream" (+1/-1)
 * (int)
 * - \var baseLookAheadTime: master look ahead (prediction) time (float)
 * - \var curvedSteering: use curved prediction (bool)
 * - \var incrementalSteering: use incremental steering (bool)
 */
// ----------------------------------------------------------------------------
template<typename Entity>
class MapDriver: public VehicleAddOnMixin<SimpleVehicle, Entity>
{
public:

	// type for a flock: an STL vector of MapDriver pointers
	typedef typename std::vector<MapDriver<Entity>*> groupType;

	// constructor
	MapDriver()
///	: map(makeMap()), path(makePath())
	{
		reset();
		map = 0;
		path = 0;
		demoSelect = 2;
		worldSize = 1.0;
		worldDiag = sqrtXXX(square(worldSize) / 2);

#ifdef OS_DEBUG
		// to compute mean time between collisions
		sumOfCollisionFreeTimes = 0;
		countOfCollisionFreeTimes = 0;
		// keep track for reliability statistics
		collisionLastTime = false;
		timeOfLastCollision = ClockObject::get_global_clock()->get_real_time();
		// keep track of average speed
		totalDistance = 0;
		totalTime = 0;
		// innitialize counters for various performance data
		stuckCount = 0;
		stuckCycleCount = 0;
		stuckOffPathCount = 0;
		lapsStarted = 0;
		lapsFinished = 0;
		hintGivenCount = 0;
		hintTakenCount = 0;
#endif

		// keep track of path following failure rate
		pathFollowTime = 0;
		pathFollowOffTime = 0;

		// follow the path "upstream or downstream" (+1/-1)
		pathFollowDirection = 1;

		// use curved prediction and incremental steering:
		curvedSteering = true;
		incrementalSteering = true;

#ifdef OS_DEBUG
		// 10 seconds with 200 points along the trail
		this->setTrailParameters(10, 200);
#endif
	}

	// destructor
	~MapDriver()
	{
///		delete (map);
///		delete (path);
	}

	// reset state
	void reset(void)
	{
		// reset the underlying vehicle class
		SimpleVehicle::reset();
		VehicleAddOnMixin<SimpleVehicle, Entity>::reset();

		// initially stopped
///		setSpeed(0);

		// Assume top speed is 20 meters per second (44.7 miles per hour).
		// This value will eventually be supplied by a higher level module.
///		setMaxSpeed(20);

		// steering force is clipped to this magnitude
///		setMaxForce(maxSpeed() * 0.4f);

		// vehicle is 2 meters wide and 3 meters long
		halfWidth = this->radius() * 1.0f;
		halfLength = this->radius() * 1.5f;

		// init dynamically controlled radius
		adjustVehicleRadiusForSpeed();

#ifdef OS_DEBUG
		// not previously avoiding
		annotateAvoid = Vec3::zero;
		// prevent long streaks due to teleportation
		this->clearTrailHistory();
#endif

		// first pass at detecting "stuck" state
		stuck = false;

		// QQQ need to clean up this hack
		qqqLastNearestObstacle = Vec3::zero;

		// master look ahead (prediction) time
		baseLookAheadTime = 3;

		if (demoSelect == 2)
		{
#ifdef OS_DEBUG
			lapsStarted++;
#endif
///			const float s = worldSize;
			const float d = (float) pathFollowDirection;
///			this->setPosition(Vec3(s * d * 0.6f, 0, s * -0.4f));
			this->regenerateOrthonormalBasisUF(Vec3::side * d);
		}

		// reset bookeeping to detect stuck cycles
		resetStuckCycleDetection();

		// assume no previous steering
		currentSteering = Vec3::zero;

#ifdef OS_DEBUG
		// assume normal running state
		dtZero = false;
#endif

		// QQQ temporary global QQQoaJustScraping
		QQQoaJustScraping = false;

		// state saved for speedometer
		//      annoteMaxRelSpeed = annoteMaxRelSpeedCurve = annoteMaxRelSpeedPath = 0;
		//      annoteMaxRelSpeed = annoteMaxRelSpeedCurve = annoteMaxRelSpeedPath = 1;
	}

	// reset state along path and inside map
	void resetForPathOrMap(void)
	{
		// init dynamically controlled radius
		adjustVehicleRadiusForSpeed();
#ifdef OS_DEBUG
		// not previously avoiding
		annotateAvoid = Vec3::zero;
		// prevent long streaks due to teleportation
		this->clearTrailHistory();
#endif
		// first pass at detecting "stuck" state
		stuck = false;
		// QQQ need to clean up this hack
		qqqLastNearestObstacle = Vec3::zero;
		if (demoSelect == 2)
		{
#ifdef OS_DEBUG
			lapsStarted++;
#endif
			const float d = (float) pathFollowDirection;
			this->regenerateOrthonormalBasisUF(Vec3::side * d);
		}
		// reset bookeeping to detect stuck cycles
		resetStuckCycleDetection();
		// assume no previous steering
		currentSteering = Vec3::zero;
#ifdef OS_DEBUG
		// assume normal running state
		dtZero = false;
#endif
		// QQQ temporary global QQQoaJustScraping
		QQQoaJustScraping = false;
	}

	void resetToPath()
	{
///		reset();
		resetForPathOrMap();
		if ((*path)[0] && (*path)[0]->isValid())
		{
			Vec3 forward;
			Vec3 pos;
			float distance;
			//get a point on the pathway
			pos = (*path)[0]->mapPointToPath(this->position(), forward,
					distance);
			//get segment index
			GCRoute::size_type segIdx = mapPointToSegmentIndex(
					*static_cast<GCRoute*>((*path)[0]), pos);
			//get forward direction
			forward = mapPointAndDirectionToTangent(
					*static_cast<GCRoute*>((*path)[0]), pos,
					pathFollowDirection);
			//randomize position on the segment
			Vec3 start = static_cast<GCRoute*>((*path)[0])->segmentStart(
					segIdx);
			pos =
					start
							+ forward
									* frandom2(0.0,
											static_cast<GCRoute*>((*path)[0])->segmentLength(
													segIdx));
			//set pos and forward
			this->setPosition(pos);
			this->setForward(forward);
		}
	}

	void resetToMap()
	{
		if((*path)[0] && (*path)[0]->isValid())
		{
			resetToPath();
		}
		else
		{
///			reset();
			resetForPathOrMap();
			float x = frandom2(map->center.x - map->xSize / 2.0 * 0.9,
					map->center.x + map->xSize / 2.0 * 0.9);
			float z = frandom2(map->center.z - map->zSize / 2.0 * 0.9,
					map->center.z + map->zSize / 2.0 * 0.9);
			this->setPosition(Vec3(x, map->center.y, z));
		}
	}

	// per frame simulation update
	void update(const float currentTime, const float elapsedTime)
	{

		handleExitFromMap();

		// QQQ first pass at detecting "stuck" state
		if (stuck && (this->relativeSpeed() < 0.001f))
		{
#ifdef OS_DEBUG
			stuckCount++;
#endif
			if (demoSelect == 2)
			{
				resetToPath();
			}
			else
			{
				resetToMap();
			}
		}

#ifdef OS_DEBUG
		// take note when current dt is zero (as in paused) for stat counters
		dtZero = (elapsedTime == 0);
#endif

		// pretend we are bigger when going fast
		adjustVehicleRadiusForSpeed();

#ifdef OS_DEBUG
		// state saved for speedometer
		//      annoteMaxRelSpeed = annoteMaxRelSpeedCurve = annoteMaxRelSpeedPath = 0;
		annoteMaxRelSpeed = annoteMaxRelSpeedCurve = annoteMaxRelSpeedPath = 1;
#endif

		// determine combined steering
		Vec3 steering;
		const bool offPath = !bodyInsidePath();
		if (stuck || offPath || detectImminentCollision())
		{
			// bring vehicle to a stop if we are stuck (newly or previously
			// stuck, because off path or collision seemed imminent)
			// (QQQ combine with stuckCycleCount code at end of this function?)
			//          this->applyBrakingForce (curvedSteering ? 3 : 2, elapsedTime); // QQQ
			this->applyBrakingForce((curvedSteering ? 3.0f : 2.0f),
					elapsedTime); // QQQ
#ifdef OS_DEBUG
			// count "off path" events
			if (offPath && !stuck && (demoSelect == 2))
				stuckOffPathCount++;
#endif
			stuck = true;

			// QQQ trying to prevent "creep" during emergency stops
			this->resetSmoothedAcceleration();
			currentSteering = Vec3::zero;
		}
		else
		{
			// determine steering for obstacle avoidance (save for annotation)
			const Vec3 avoid =
#ifdef OS_DEBUG
					annotateAvoid =
#endif
					steerToAvoidObstaclesOnMap(lookAheadTimeOA(), *map,
							hintForObstacleAvoidance());
			const bool needToAvoid = avoid != Vec3::zero;

			// any obstacles to avoid?
			if (needToAvoid)
			{
				// slow down and turn to avoid the obstacles
				const float targetSpeed = (
						(curvedSteering && QQQoaJustScraping) ?
								maxSpeedForCurvature() : 0);
#ifdef OS_DEBUG
				annoteMaxRelSpeed = targetSpeed / this->maxSpeed();
#endif
				const float avoidWeight = 3 + (3 * this->relativeSpeed()); // ad hoc
				steering = avoid * avoidWeight;
				steering += this->steerForTargetSpeed(targetSpeed);
			}
			else
			{
				// otherwise speed up and...
				steering = this->steerForTargetSpeed(maxSpeedForCurvature());

				// wander for demo 1
				if (demoSelect == 1)
				{
					const Vec3 wander = this->steerForWander(elapsedTime);
					const Vec3 flat = wander.setYtoZero();
					const Vec3 weighted = flat.truncateLength(this->maxForce())
							* 6;
#ifdef OS_DEBUG
					const Vec3 a = this->position() + Vec3(0, 0.2f, 0);
					this->annotationLine(a, a + (weighted * 0.3f), gWhite);
#endif
					steering += weighted;
				}

				// follow the path in demo 2
				if (demoSelect == 2)
				{
					const Vec3 pf = steerToFollowPath(pathFollowDirection,
							lookAheadTimePF(),
							*static_cast<GCRoute*>((*path)[0]));
					if (pf != Vec3::zero)
					{
						// steer to remain on path
						if (pf.dot(this->forward()) < 0)
							steering = pf;
						else
							steering = pf + steering;
					}
					else
					{
						// path aligment: when neither obstacle avoidance nor
						// path following is required, align with path segment
						const Vec3 pathHeading = mapPointAndDirectionToTangent(
								*static_cast<GCRoute*>((*path)[0]),
								this->position(), pathFollowDirection); // path->tangentAt (position (), pathFollowDirection);
						{
							const Vec3 b = (this->position()
									+ (this->up() * 0.2f)
									+ (this->forward() * halfLength * 1.4f));
#ifdef OS_DEBUG
							const float l = 2;
							this->annotationLine(b, b + (this->forward() * l),
									gCyan);
							this->annotationLine(b, b + (pathHeading * l),
									gCyan);
#endif
						}
						steering +=
								(steerTowardHeading(pathHeading)
										* (isNearWaypoint(
												*static_cast<GCRoute*>((*path)[0]),
												this->position()) /* path->nearWaypoint (position () ) */?
												0.5f : 0.1f));
					}
				}
			}
		}

		if (!stuck)
		{
			// convert from absolute to incremental steering signal
			if (incrementalSteering)
				steering = convertAbsoluteToIncrementalSteering(steering,
						elapsedTime);
			// enforce minimum turning radius
			steering = adjustSteeringForMinimumTurningRadius(steering);
		}

		// apply selected steering force to vehicle, record data
		this->applySteeringForce(steering, elapsedTime);

		///call the entity update
		this->entityUpdate(currentTime, elapsedTime);

#ifdef OS_DEBUG
		collectReliabilityStatistics(currentTime, elapsedTime);
#endif

		// detect getting stuck in cycles -- we are moving but not
		// making progress down the route (annotate smoothedPosition)
		if (demoSelect == 2)
		{
			const bool circles = weAreGoingInCircles();
#ifdef OS_DEBUG
			if (circles && !stuck)
				stuckCycleCount++;
			this->annotationCircleOrDisk(0.5, this->up(),
					this->smoothedPosition(), gWhite, 12, circles, false);
#endif
			if (circles)
				stuck = true;
		}

#ifdef OS_DEBUG
		// annotation
		this->perFrameAnnotation();
		this->recordTrailVertex(currentTime, this->position());
#endif
	}

	//  // QQQ 5-8-04 random experiment, currently unused
	//  //
	//  // reduce lateral steering at low speeds
	//  //
	//  Vec3 reduceTurningAtLowSpeeds (const Vec3& rawSteering)
	//  {
	//      const Vec3 thrust = rawSteering.parallelComponent (forward ());
	//      const Vec3 lateral = rawSteering.perpendicularComponent (forward ());
	//      // const float adjust = relativeSpeed ();
	//      // const float adjust = square (relativeSpeed ());
	//      const float adjust = square (square (relativeSpeed ()));
	//      return thrust + (lateral * adjust);
	//  }

	void adjustVehicleRadiusForSpeed(void)
	{
		const float minRadius = sqrtXXX(square(halfWidth) + square(halfLength));
		const float safetyMargin = (
				curvedSteering ?
						interpolate(this->relativeSpeed(), 0.0f, 1.5f) : 0.0f);
		this->setRadius(minRadius + safetyMargin);
	}

#ifdef OS_DEBUG
	void collectReliabilityStatistics(const float currentTime,
			const float elapsedTime)
	{
		// detect obstacle avoidance failure and keep statistics
		collisionDetected = map->scanLocalXZRectangle(*this, -halfWidth,
				halfWidth, -halfLength, halfLength);

		// record stats to compute mean time between collisions
		const float timeSinceLastCollision = currentTime - timeOfLastCollision;
		if (collisionDetected && !collisionLastTime
				&& (timeSinceLastCollision > 1))
		{
			std::ostringstream message;
			message << "collision after " << timeSinceLastCollision
					<< " seconds";
			std::cout << message.str() << std::ends;
			sumOfCollisionFreeTimes += timeSinceLastCollision;
			countOfCollisionFreeTimes++;
			timeOfLastCollision = currentTime;
		}
		collisionLastTime = collisionDetected;

		// keep track of average speed
		totalDistance += this->speed() * elapsedTime;
		totalTime += elapsedTime;

		// keep track of path following failure rate
		// QQQ for now, duplicating this code from the draw method:
		// if we are following a path but are off the path,
		// draw a red line to where we should be
		if (demoSelect == 2)
		{
			pathFollowTime += elapsedTime;
			if (!bodyInsidePath())
				pathFollowOffTime += elapsedTime;
		}
	}
#endif

	Vec3 hintForObstacleAvoidance(void)
	{
		// used only when path following, return zero ("no hint") otherwise
		if (demoSelect != 2)
			return Vec3::zero;

		// are we heading roughly parallel to the current path segment?
		const Vec3 p = this->position();
		const Vec3 pathHeading = mapPointAndDirectionToTangent(
				*static_cast<GCRoute*>((*path)[0]), p, pathFollowDirection); // path->tangentAt (p, pathFollowDirection);
		if (pathHeading.dot(this->forward()) < 0.8f)
		{
			// if not, the "hint" is to turn to align with path heading
#ifdef OS_DEBUG
			const Vec3 s = this->side() * halfWidth;
			const float f = halfLength * 2;
			this->annotationLine(p + s, p + s + (this->forward() * f), gBlack);
			this->annotationLine(p - s, p - s + (this->forward() * f), gBlack);
			this->annotationLine(p, p + (pathHeading * 5), gMagenta);
#endif
			return pathHeading;
		}
		else
		{
			// when there is a valid nearest obstacle position
			const Vec3 obstacle = qqqLastNearestObstacle;
			const Vec3 o = obstacle + (this->up() * 0.1f);
			if (obstacle != Vec3::zero)
			{
				// get offset, distance from obstacle to its image on path
				float outside;
				const Vec3 onPath = mapPointToPointOnCenterLineAndOutside(
						*static_cast<GCRoute*>((*path)[0]), obstacle, outside); // path->mapPointToPath (obstacle, outside);
				const Vec3 offset = onPath - obstacle;
				const float offsetDistance = offset.length();

				// when the obstacle is inside the path tube
				if (outside < 0)
				{
					// when near the outer edge of a sufficiently wide tube
					// const int segmentIndex = path->indexOfNearestSegment (onPath);
					// const float segmentRadius = path->segmentRadius( segmentIndex );
					float const segmentRadius = mapPointToRadius(
							*static_cast<GCRoute*>((*path)[0]), onPath);
					const float w = halfWidth * 6;
					const bool nearEdge = offsetDistance > w;
					const bool wideEnough = segmentRadius > (w * 2);
					if (nearEdge && wideEnough)
					{
						const float obstacleDistance = (obstacle - p).length();
						const float range = this->speed() * lookAheadTimeOA();
						const float farThreshold = range * 0.8f;
						const bool usableHint = obstacleDistance > farThreshold;
						if (usableHint)
						{
#ifdef OS_DEBUG
							const Vec3 q = p + (offset.normalize() * 5);
							this->annotationLine(p, q, gMagenta);
							this->annotationCircleOrDisk(0.4f, this->up(), o,
									gWhite, 12, false, false);
#endif
							return offset;
						}
					}
				}
#ifdef OS_DEBUG
				this->annotationCircleOrDisk(0.4f, this->up(), o, gBlack, 12,
						false, false);
#endif
			}
		}
		// otherwise, no hint
		return Vec3::zero;
	}

	// like steerToAvoidObstacles, but based on a BinaryTerrainMap indicating
	// the possitions of impassible regions
	//
	Vec3 steerToAvoidObstaclesOnMap(const float minTimeToCollision,
			const TerrainMap& map)
	{
		return steerToAvoidObstaclesOnMap(minTimeToCollision, map, Vec3::zero); // no steer hint
	}

	// given a map of obstacles (currently a global, binary map) steer so as
	// to avoid collisions within the next minTimeToCollision seconds.
	//
	Vec3 steerToAvoidObstaclesOnMap(const float minTimeToCollision,
			const TerrainMap& map, const Vec3& steerHint)
	{
		const float spacing = map.minSpacing() / 2;
		const float maxSide = this->radius();
		const float maxForward = minTimeToCollision * this->speed();
		const int maxSamples = (int) (maxForward / spacing);
		const Vec3 step = this->forward() * spacing;
		const Vec3 fOffset = this->position();
		Vec3 sOffset;
		float s = spacing / 2;

		const int infinity = 9999; // qqq
		int nearestL = infinity;
		int nearestR = infinity;
		int nearestWL = infinity;
		int nearestWR = infinity;
		Vec3 nearestO;

#ifdef OS_DEBUG
		wingDrawFlagL = false;
		wingDrawFlagR = false;
#endif

		const bool hintGiven = steerHint != Vec3::zero;

#ifdef OS_DEBUG
		if (hintGiven && !dtZero)
			hintGivenCount++;
		if (hintGiven)
			this->annotationCircleOrDisk(halfWidth * 0.9f, this->up(),
					this->position() + (this->up() * 0.2f), gWhite, 12, false,
					false);
#endif

		// QQQ temporary global QQQoaJustScraping
		QQQoaJustScraping = true;

		const float signedRadius = 1 / nonZeroCurvatureQQQ();
		const Vec3 localCenterOfCurvature = this->side() * signedRadius;
		const Vec3 center = this->position() + localCenterOfCurvature;
		const float sign = signedRadius < 0 ? 1.0f : -1.0f;
		const float arcRadius = signedRadius * -sign;
		const float twoPi = 2 * OPENSTEER_M_PI;
		const float circumference = twoPi * arcRadius;
		const float rawLength = this->speed() * minTimeToCollision * sign;
		const float fracLimit = 1.0f / 6.0f;
		const float distLimit = circumference * fracLimit;
		const float arcLength = arcLengthLimit(rawLength, distLimit);
		const float arcAngle = twoPi * arcLength / circumference;

#ifdef OS_DEBUG
		// XXX temp annotation to show limit on arc angle
		if (curvedSteering)
		{
			if ((this->speed() * minTimeToCollision)
					> (circumference * fracLimit))
			{
				const float q = twoPi * fracLimit;
				const Vec3 fooz = this->position() - center;
				const Vec3 booz = fooz.rotateAboutGlobalY(sign * q);
				this->annotationLine(center, center + fooz, gRed);
				this->annotationLine(center, center + booz, gRed);
			}
		}

		// assert loops will terminate
		assert(spacing > 0);
#endif

		// scan corridor straight ahead of vehicle,
		// keep track of nearest obstacle on left and right sides
		while (s < maxSide)
		{
			sOffset = this->side() * s;
			s += spacing;
			const Vec3 lOffset = fOffset + sOffset;
			const Vec3 rOffset = fOffset - sOffset;

			Vec3 lObsPos, rObsPos;

			const int L = (
					curvedSteering ?
							(int) (scanObstacleMap(lOffset, center, arcAngle,
									maxSamples, 0,
#ifdef OS_DEBUG
									gYellow, gRed,
#endif
									lObsPos) / spacing) :
							map.scanXZray(lOffset, step, maxSamples));
			const int R = (
					curvedSteering ?
							(int) (scanObstacleMap(rOffset, center, arcAngle,
									maxSamples, 0,
#ifdef OS_DEBUG
									gYellow, gRed,
#endif
									rObsPos) / spacing) :
							map.scanXZray(rOffset, step, maxSamples));

			if ((L > 0) && (L < nearestL))
			{
				nearestL = L;
				if (L < nearestR)
					nearestO = (
							(curvedSteering) ?
									lObsPos : lOffset + ((float) L * step));
			}
			if ((R > 0) && (R < nearestR))
			{
				nearestR = R;
				if (R < nearestL)
					nearestO = (
							(curvedSteering) ?
									rObsPos : rOffset + ((float) R * step));
			}

			if (!curvedSteering)
			{
				annotateAvoidObstaclesOnMap(lOffset, L, step);
				annotateAvoidObstaclesOnMap(rOffset, R, step);
			}

			if (curvedSteering)
			{
				// QQQ temporary global QQQoaJustScraping
				const bool outermost = s >= maxSide;
				const bool eitherSide = (L > 0) || (R > 0);
				if (!outermost && eitherSide)
					QQQoaJustScraping = false;
			}
		}
		qqqLastNearestObstacle = nearestO;

		// scan "wings"
		{
			const int wingScans = 4;
			// see duplicated code at: QQQ draw sensing "wings"
			// QQQ should be a parameter of this method
			const Vec3 wingWidth = this->side() * wingSlope() * maxForward;

#ifdef OS_DEBUG
			const Color beforeColor(0.75f, 0.9f, 0.0f);  // for annotation
			const Color afterColor(0.9f, 0.5f, 0.0f);  // for annotation
#endif

			for (int i = 1; i <= wingScans; i++)
			{
				const float fraction = (float) i / (float) wingScans;
				const Vec3 endside = sOffset + (wingWidth * fraction);
				const Vec3 corridorFront = this->forward() * maxForward;

				// "loop" from -1 to 1
				for (int j = -1; j < 2; j += 2)
				{
					float k = (float) j; // prevent VC7.1 warning
					const Vec3 start = fOffset + (sOffset * k);
					const Vec3 end = fOffset + corridorFront + (endside * k);
					const Vec3 ray = end - start;
					const float rayLength = ray.length();
					const Vec3 step = ray * spacing / rayLength;
					const int raySamples = (int) (rayLength / spacing);
					const float endRadius = wingSlope() * maxForward * fraction
							* (signedRadius < 0 ? 1 : -1) * (j == 1 ? 1 : -1);
					Vec3 ignore;
					const int scan = (
							curvedSteering ?
									(int) (scanObstacleMap(start, center,
											arcAngle, raySamples, endRadius,
#ifdef OS_DEBUG
											beforeColor, afterColor,
#endif
											ignore) / spacing) :
									map.scanXZray(start, step, raySamples));

					if (!curvedSteering)
						annotateAvoidObstaclesOnMap(start, scan, step);

#ifdef OS_DEBUG
					if (j == 1)
					{
						if ((scan > 0) && (scan < nearestWL))
							nearestWL = scan;
					}
					else
					{
						if ((scan > 0) && (scan < nearestWR))
							nearestWR = scan;
					}
#endif
				}
			}
#ifdef OS_DEBUG
			wingDrawFlagL = nearestWL != infinity;
			wingDrawFlagR = nearestWR != infinity;
#endif
		}

#ifdef OS_DEBUG
		// for annotation
		savedNearestWR = (float) nearestWR;
		savedNearestR = (float) nearestR;
		savedNearestL = (float) nearestL;
		savedNearestWL = (float) nearestWL;
#endif

		// flags for compound conditions, used below
		const bool obstacleFreeC = nearestL == infinity && nearestR == infinity;
		const bool obstacleFreeL = nearestL == infinity
				&& nearestWL == infinity;
		const bool obstacleFreeR = nearestR == infinity
				&& nearestWR == infinity;
		const bool obstacleFreeWL = nearestWL == infinity;
		const bool obstacleFreeWR = nearestWR == infinity;
		const bool obstacleFreeW = obstacleFreeWL && obstacleFreeWR;

		// when doing curved steering and we have already detected "just
		// scarping" but neither wing is free, recind the "just scarping"
		// QQQ temporary global QQQoaJustScraping
		const bool JS = curvedSteering && QQQoaJustScraping;
		const bool cancelJS = !obstacleFreeWL && !obstacleFreeWR;
		if (JS && cancelJS)
			QQQoaJustScraping = false;

		// ----------------------------------------------------------
		// now we have measured everything, decide which way to steer
		// ----------------------------------------------------------

		// no obstacles found on path, return zero steering
		if (obstacleFreeC)
		{
			qqqLastNearestObstacle = Vec3::zero;
#ifdef OS_DEBUG
			annotationNoteOAClauseName("obstacleFreeC");
#endif
			// qqq  this may be in the wrong place (what would be the right
			// qqq  place?!) but I'm trying to say "even if the path is
			// qqq  clear, don't go too fast when driving between obstacles
			if (obstacleFreeWL || obstacleFreeWR
					|| this->relativeSpeed() < 0.7f)
				return Vec3::zero;
			else
				return -this->forward();
		}

		// if the nearest obstacle is way out there, take hint if any
		//      if (hintGiven && (minXXX (nearestL, nearestR) > (maxSamples * 0.8f)))
		if (hintGiven
				&& (minXXX((float) nearestL, (float) nearestR)
						> (maxSamples * 0.8f)))
		{
#ifdef OS_DEBUG
			annotationNoteOAClauseName("nearest obstacle is way out there");
			annotationHintWasTaken();
#endif
			if (steerHint.dot(this->side()) > 0)
				return this->side();
			else
				return -this->side();
		}

		// QQQ experiment 3-9-04
		//
		// since there are obstacles ahead, if we are already near
		// maximum curvature, we MUST turn in opposite direction
		//
		// are we turning more sharply than the minimum turning radius?
		// (code from adjustSteeringForMinimumTurningRadius)
		const float maxCurvature = 1 / (minimumTurningRadius() * 1.2f);
		if (absXXX(this->curvature()) > maxCurvature)
		{
#ifdef OS_DEBUG
			annotationNoteOAClauseName("min turn radius");
			this->annotationCircleOrDisk(minimumTurningRadius() * 1.2f,
					this->up(), center, gBlue * 0.8f, 40, false, false);
#endif
			return this->side() * sign;
		}

		// if either side is obstacle-free, turn in that direction
#ifdef OS_DEBUG
		if (obstacleFreeL || obstacleFreeR)
			annotationNoteOAClauseName("obstacle-free side");
#endif

		if (obstacleFreeL)
			return this->side();
		if (obstacleFreeR)
			return -this->side();

		// if wings are clear, turn away from nearest obstacle straight ahead
		if (obstacleFreeW)
		{
#ifdef OS_DEBUG
			annotationNoteOAClauseName("obstacleFreeW");
#endif
			// distance to obs on L and R side of corridor roughtly the same
			const bool same = absXXX(nearestL - nearestR) < 5; // within 5
			// if they are about the same and a hint is given, use hint
			if (same && hintGiven)
			{
#ifdef OS_DEBUG
				annotationHintWasTaken();
#endif
				if (steerHint.dot(this->side()) > 0)
					return this->side();
				else
					return -this->side();
			}
			else
			{
				// otherwise steer toward the less cluttered side
				if (nearestL > nearestR)
					return this->side();
				else
					return -this->side();
			}
		}

		// if the two wings are about equally clear and a steering hint is
		// provided, use it
		const bool equallyClear = absXXX(nearestWL - nearestWR) < 2; // within 2
		if (equallyClear && hintGiven)
		{
#ifdef OS_DEBUG
			annotationNoteOAClauseName("equallyClear");
			annotationHintWasTaken();
#endif
			if (steerHint.dot(this->side()) > 0)
				return this->side();
			else
				return -this->side();
		}

		// turn towards the side whose "wing" region is less cluttered
		// (the wing whose nearest obstacle is furthest away)
#ifdef OS_DEBUG
		annotationNoteOAClauseName("wing less cluttered");
#endif
		if (nearestWL > nearestWR)
			return this->side();
		else
			return -this->side();
	}

	// QQQ reconsider calling sequence
	// called when steerToAvoidObstaclesOnMap decides steering is required
	// (default action is to do nothing, layered classes can overload it)
	// virtual void annotateAvoidObstaclesOnMap (const Vec3& scanOrigin,
	//                                           int scanIndex,
	//                                           const Vec3& scanStep)
	// {
	// }
	void annotateAvoidObstaclesOnMap(const Vec3& scanOrigin, int scanIndex,
			const Vec3& scanStep)
	{
#ifdef OS_DEBUG
		if (scanIndex > 0)
		{
			const Vec3 hit = scanOrigin + (scanStep * (float) scanIndex);
			this->annotationLine(scanOrigin, hit, Color(0.7f, 0.3f, 0.3f));
		}
#endif

		///call parent::annotateAvoidObstacle
		VehicleAddOnMixin<SimpleVehicle, Entity>::annotateAvoidObstacle(0.0);
	}

#ifdef OS_DEBUG
	void annotationNoteOAClauseName(const char* clauseName)
	{
		OPENSTEER_UNUSED_PARAMETER(clauseName);

		// does noting now, idea was that it might draw 2d text near vehicle
		// with this state information
		//

		// print version:
		//
		// if (!dtZero) std::cout << clauseName << std::endl;

		// was had been in caller:
		//
		//if (!dtZero)
		//{
		//    const int WR = nearestWR; debugPrint (WR);
		//    const int R  = nearestR;  debugPrint (R);
		//    const int L  = nearestL;  debugPrint (L);
		//    const int WL = nearestWL; debugPrint (WL);
		//}
	}

	void annotationHintWasTaken(void)
	{
		if (!dtZero)
			hintTakenCount++;

		const float r = halfWidth * 0.9f;
		const Vec3 ff = this->forward() * r;
		const Vec3 ss = this->side() * r;
		const Vec3 pp = this->position() + (this->up() * 0.2f);
		this->annotationLine(pp + ff + ss, pp - ff + ss, gWhite);
		this->annotationLine(pp - ff - ss, pp - ff + ss, gWhite);
		this->annotationLine(pp - ff - ss, pp + ff - ss, gWhite);
		this->annotationLine(pp + ff + ss, pp + ff - ss, gWhite);
	}
#endif

	// scan across the obstacle map along a given arc
	// (possibly with radius adjustment ramp)
	// returns approximate distance to first obstacle found
	//
	// QQQ 1: this calling sequence does not allow for zero curvature case
	// QQQ 2: in library version of this, "map" should be a parameter
	// QQQ 3: instead of passing in colors, call virtual annotation function?
	// QQQ 4: need flag saying to continue after a hit, for annotation
	// QQQ 5: I needed to return both distance-to and position-of the first
	//        obstacle. I added returnObstaclePosition but maybe this should
	//        return a "scan results object" with a flag for obstacle found,
	//        plus distant and position if so.
	//
	float scanObstacleMap(const Vec3& start, const Vec3& center,
			const float arcAngle, const int segments,
			const float endRadiusChange,
#ifdef OS_DEBUG
			const Color& beforeColor, const Color& afterColor,
#endif
			Vec3& returnObstaclePosition)
	{
		// "spoke" is initially the vector from center to start,
		// which is then rotated step by step around center
		Vec3 spoke = start - center;
		// determine the angular step per segment
		const float step = arcAngle / segments;
		// store distance to, and position of first obstacle
		float obstacleDistance = 0;
		returnObstaclePosition = Vec3::zero;
		// for spiral "ramps" of changing radius
		const float startRadius = (endRadiusChange == 0) ? 0 : spoke.length();

		// traverse each segment along arc
		float sin = 0, cos = 0;
		Vec3 oldPoint = start;
		bool obstacleFound = false;
		for (int i = 0; i < segments; i++)
		{
			// rotate "spoke" to next step around circle
			// (sin and cos values get filled in on first call)
			spoke = spoke.rotateAboutGlobalY(step, sin, cos);

			// for spiral "ramps" of changing radius
			const float adjust = (
					(endRadiusChange == 0) ?
							1.0f :
							interpolate((float) (i + 1) / (float) segments,
									1.0f,
									(maxXXX(0, (startRadius + endRadiusChange))
											/ startRadius)));

			// construct new scan point: center point, offset by rotated
			// spoke (possibly adjusting the radius if endRadiusChange!=0)
			const Vec3 newPoint = center + (spoke * adjust);

			// once an obstacle if found "our work here is done" -- continue
			// to loop only for the sake of annotation (make that optional?)
			if (obstacleFound)
			{
#ifdef OS_DEBUG
				this->annotationLine(oldPoint, newPoint, afterColor);
#endif
			}
			else
			{
				// no obstacle found on this scan so far,
				// scan map along current segment (a chord of the arc)
				const Vec3 offset = newPoint - oldPoint;
				const float d2 = offset.length() * 2;

				// when obstacle found: set flag, save distance and position
				if (!map->isPassable(newPoint))
				{
					obstacleFound = true;
					obstacleDistance = d2 * 0.5f * (i + 1);
					returnObstaclePosition = newPoint;
				}
#ifdef OS_DEBUG
				this->annotationLine(oldPoint, newPoint, beforeColor);
#endif
			}
			// save new point for next time around loop
			oldPoint = newPoint;
		}
		// return distance to first obstacle (or zero if none found)
		return obstacleDistance;
	}

	bool detectImminentCollision(void)
	{
		// QQQ  this should be integrated into steerToAvoidObstaclesOnMap
		// QQQ  since it shares so much infrastructure
		// QQQ  less so after changes on 3-16-04
		bool returnFlag = false;
		const float spacing = map->minSpacing() / 2;
		const float maxSide = halfWidth + spacing;
		const float minDistance = curvedSteering ? 2.0f : 2.5f; // meters
		const float predictTime = curvedSteering ? .75f : 1.3f; // seconds
		const float maxForward = this->speed()
				* combinedLookAheadTime(predictTime, minDistance);
		const Vec3 step = this->forward() * spacing;
		float s = curvedSteering ? (spacing / 4) : (spacing / 2);

		const float signedRadius = 1 / nonZeroCurvatureQQQ();
		const Vec3 localCenterOfCurvature = this->side() * signedRadius;
		const Vec3 center = this->position() + localCenterOfCurvature;
		const float sign = signedRadius < 0 ? 1.0f : -1.0f;
		const float arcRadius = signedRadius * -sign;
		const float twoPi = 2 * OPENSTEER_M_PI;
		const float circumference = twoPi * arcRadius;
		const Vec3 qqqLift(0, 0.2f, 0);
		Vec3 ignore;

		// scan region ahead of vehicle
		while (s < maxSide)
		{
			const Vec3 sOffset = this->side() * s;
			const Vec3 lOffset = this->position() + sOffset;
			const Vec3 rOffset = this->position() - sOffset;
			const float bevel = 0.3f;
			const float fraction = s / maxSide;
			const float scanDist = (halfLength
					+ interpolate(fraction, maxForward, maxForward * bevel));
			const float angle = (scanDist * twoPi * sign) / circumference;
			const int samples = (int) (scanDist / spacing);
			const int L = (
					curvedSteering ?
							(int) (scanObstacleMap(lOffset + qqqLift, center,
									angle, samples, 0,
#ifdef OS_DEBUG
									gMagenta, gCyan,
#endif
									ignore) / spacing) :
							map->scanXZray(lOffset, step, samples));
			const int R = (
					curvedSteering ?
							(int) (scanObstacleMap(rOffset + qqqLift, center,
									angle, samples, 0,
#ifdef OS_DEBUG
									gMagenta, gCyan,
#endif
									ignore) / spacing) :
							map->scanXZray(rOffset, step, samples));

			returnFlag = returnFlag || (L > 0);
			returnFlag = returnFlag || (R > 0);

#ifdef OS_DEBUG
			// annotation
			if (!curvedSteering)
			{
				const Vec3 d(step * (float) samples);
				this->annotationLine(lOffset, lOffset + d, gWhite);
				this->annotationLine(rOffset, rOffset + d, gWhite);
			}
#endif

			// increment sideways displacement of scan line
			s += spacing;
		}
		return returnFlag;
	}

	// see comments at SimpleVehicle::predictFuturePosition, in this instance
	// I just need the future position (not a LocalSpace), so I'll keep the
	// calling sequence and just conditionalize its body
	//
	// this should be const, but easier for now to ignore that

	Vec3 predictFuturePosition(const float predictionTime) const
	{
		if (curvedSteering)
		{
			// QQQ this chunk of code is repeated in far too many places,
			// QQQ it has to be moved inside some utility
			// QQQ
			// QQQ and now, worse, I rearranged it to try the "limit arc
			// QQQ angle" trick
			const float signedRadius = 1 / nonZeroCurvatureQQQ();
			const Vec3 localCenterOfCurvature = this->side() * signedRadius;
			const Vec3 center = this->position() + localCenterOfCurvature;
			const float sign = signedRadius < 0 ? 1.0f : -1.0f;
			const float arcRadius = signedRadius * -sign;
			const float twoPi = 2 * OPENSTEER_M_PI;
			const float circumference = twoPi * arcRadius;
			const float rawLength = this->speed() * predictionTime * sign;
			const float arcLength = arcLengthLimit(rawLength,
					circumference * 0.25f);
			const float arcAngle = twoPi * arcLength / circumference;

			const Vec3 spoke = this->position() - center;
			const Vec3 newSpoke = spoke.rotateAboutGlobalY(arcAngle);
			const Vec3 prediction = newSpoke + center;

#ifdef OS_DEBUG
			// QQQ unify with annotatePathFollowing
			const Color futurePositionColor(0.5f, 0.5f, 0.6f);
			annotationXZArc(this->position(), center, arcLength, 20,
					futurePositionColor);
#endif
			return prediction;
		}
		else
		{
			return this->position() + (this->velocity() * predictionTime);
		}
	}

	// QQQ experimental fix for arcLength limit in predictFuturePosition
	// QQQ and steerToAvoidObstaclesOnMap.
	//
	// args are the intended arc length (signed!), and the limit which is
	// a given (positive!) fraction of the arc's (circle's) circumference
	//

	float arcLengthLimit(const float length, const float limit) const
	{
		if (length > 0)
			return minXXX(length, limit);
		else
			return -minXXX(-length, limit);
	}

	// this is a version of the one in SteerLibrary.h modified for "slow when
	// heading off path".  I put it here because the changes were not
	// compatible with Pedestrians.cpp.  It needs to be merged back after
	// things settle down.
	//
	// its been modified in other ways too (such as "reduce the offset if
	// facing in the wrong direction" and "increase the target offset to
	// compensate the fold back") plus I changed the type of "path" from
	// Pathway to GCRoute to use methods like indexOfNearestSegment and
	// dotSegmentUnitTangents
	//
	// and now its been modified again for curvature-based prediction
	//
	Vec3 steerToFollowPath(const int direction, const float predictionTime,
			GCRoute& path)
	{
		if (curvedSteering)
			return steerToFollowPathCurve(direction, predictionTime, path);
		else
			return steerToFollowPathLinear(direction, predictionTime, path);
	}

	Vec3 steerToFollowPathLinear(const int direction,
			const float predictionTime, GCRoute& path)
	{
		// our goal will be offset from our path distance by this amount
		const float pathDistanceOffset = direction * predictionTime
				* this->speed();

		// predict our future position
		const Vec3 futurePosition = predictFuturePosition(predictionTime);

		// measure distance along path of our current and predicted positions
		const float nowPathDistance = path.mapPointToPathDistance(
				this->position());

		// are we facing in the correction direction?
		const Vec3 pathHeading = mapPointToTangent(path, this->position())
				* static_cast<float>(direction); // path.tangentAt(position()) * (float)direction;
		const bool correctDirection = pathHeading.dot(this->forward()) > 0;

		// find the point on the path nearest the predicted future position
		// XXX need to improve calling sequence, maybe change to return a
		// XXX special path-defined object which includes two Vec3s and a
		// XXX bool (onPath,tangent (ignored), withinPath)
		float futureOutside;
		const Vec3 onPath = mapPointToPointOnCenterLineAndOutside(path,
				futurePosition, futureOutside); // path.mapPointToPath (futurePosition,futureOutside);

		// determine if we are currently inside the path tube
		float nowOutside;
		const Vec3 nowOnPath = mapPointToPointOnCenterLineAndOutside(path,
				this->position(), nowOutside); // path.mapPointToPath (position (), nowOutside);

		// no steering is required if our present and future positions are
		// inside the path tube and we are facing in the correct direction
		const float m = -this->radius();
		const bool whollyInside = (futureOutside < m) && (nowOutside < m);
		if (whollyInside && correctDirection)
		{
			// all is well, return zero steering
			return Vec3::zero;
		}
		else
		{
			// otherwise we need to steer towards a target point obtained
			// by adding pathDistanceOffset to our current path position
			// (reduce the offset if facing in the wrong direction)
			const float targetPathDistance = (nowPathDistance
					+ (pathDistanceOffset * (correctDirection ? 1 : 0.1f)));
			Vec3 target = path.mapPathDistanceToPoint(targetPathDistance);

			// if we are on one segment and target is on the next segment and
			// the dot of the tangents of the two segments is negative --
			// increase the target offset to compensate the fold back
			const int ip = static_cast<int>(mapPointToSegmentIndex(path,
					this->position())); // path.indexOfNearestSegment (position ());
			const int it =
					static_cast<int>(mapPointToSegmentIndex(path, target)); // path.indexOfNearestSegment (target);
			// Because polyline paths have a constant tangent along a segment
			// just set the distance along the segment to @c 0.0f.
			Vec3 const ipTangent = path.mapSegmentDistanceToTangent(ip, 0.0f);
			// Because polyline paths have a constant tangent along a segment
			// just set the distance along the segment to @c 0.0f.
			Vec3 const itTangent = path.mapSegmentDistanceToTangent(it, 0.0f);
			if (((ip + direction) == it)
					&& ( /* path.dotSegmentUnitTangents (it, ip) */itTangent.dot(
							ipTangent) < -0.1f))
			{
				const float newTargetPathDistance = nowPathDistance
						+ (pathDistanceOffset * 2);
				target = path.mapPathDistanceToPoint(newTargetPathDistance);
			}

			this->annotatePathFollowing(futurePosition, onPath, target,
					futureOutside);

			// if we are currently outside head directly in
			// (QQQ new, experimental, makes it turn in more sharply)
			if (nowOutside > 0)
				return this->steerForSeek(nowOnPath);

			// steering to seek target on path
			const Vec3 seek = this->steerForSeek(target).truncateLength(
					this->maxForce());

			// return that seek steering -- except when we are heading off
			// the path (currently on path and future position is off path)
			// in which case we put on the brakes.
			if ((nowOutside < 0) && (futureOutside > 0))
				return (seek.perpendicularComponent(this->forward())
						- (this->forward() * this->maxForce()));
			else
				return seek;
		}
	}

	// Path following case for curved prediction and incremental steering
	// (called from steerToFollowPath for the curvedSteering case)
	//
	// QQQ this does not handle the case when we AND futurePosition
	// QQQ are outside, say when approach the path from far away
	//
	Vec3 steerToFollowPathCurve(const int direction, const float predictionTime,
			GCRoute& path)
	{
		// predict our future position (based on current curvature and speed)
		const Vec3 futurePosition = predictFuturePosition(predictionTime);
		// find the point on the path nearest the predicted future position
		float futureOutside;
		const Vec3 onPath = mapPointToPointOnCenterLineAndOutside(path,
				futurePosition, futureOutside); // path.mapPointToPath (futurePosition,futureOutside);
		const Vec3 pathHeading = mapPointAndDirectionToTangent(path, onPath,
				direction); // path.tangentAt (onPath, direction);
		const Vec3 rawBraking = this->forward() * this->maxForce() * -1;
		const Vec3 braking = ((futureOutside < 0) ? Vec3::zero : rawBraking);
		//qqq experimental wrong-way-fixer
		float nowOutside;
		Vec3 nowTangent;
		const Vec3 p = this->position();
		const Vec3 nowOnPath = path.mapPointToPath(p, nowTangent, nowOutside);
		nowTangent *= (float) direction;
		const float alignedness = nowTangent.dot(this->forward());

		// facing the wrong way?
		if (alignedness < 0)
		{
#ifdef OS_DEBUG
			this->annotationLine(p, p + (nowTangent * 10), gCyan);
#endif
			// if nearly anti-parallel
			if (alignedness < -0.707f)
			{
				const Vec3 towardCenter = nowOnPath - p;
				const Vec3 turn = (
						towardCenter.dot(this->side()) > 0 ?
								this->side() * this->maxForce() :
								this->side() * this->maxForce() * -1);
				return (turn + rawBraking);
			}
			else
			{
				return (steerTowardHeading(pathHeading).perpendicularComponent(
						this->forward()) + braking);
			}
		}

		// is the predicted future position(+radius+margin) inside the path?
		if (futureOutside < -(this->radius() + 1.0f)) //QQQ
		{
			// then no steering is required
			return Vec3::zero;
		}
		else
		{
			// otherwise determine corrective steering (including braking)
#ifdef OS_DEBUG
			this->annotationLine(futurePosition, futurePosition + pathHeading,
					gRed);
#endif
			annotatePathFollowing(futurePosition, onPath, this->position(),
					futureOutside);
			// two cases, if entering a turn (a waypoint between path segments)
			if ( /* path.nearWaypoint (onPath) */isNearWaypoint(path, onPath)
					&& (futureOutside > 0))
			{
				// steer to align with next path segment
#ifdef OS_DEBUG
				this->annotationCircleOrDisk(0.5f, this->up(), futurePosition,
						gRed, 8, false, false);
#endif
				return steerTowardHeading(pathHeading) + braking;
			}
			else
			{
				// otherwise steer away from the side of the path we
				// are heading for
				const Vec3 pathSide = this->localRotateForwardToSide(
						pathHeading);
				const Vec3 towardFP = futurePosition - onPath;
				const float whichSide =
						(pathSide.dot(towardFP) < 0) ? 1.0f : -1.0f;
				return (this->side() * this->maxForce() * whichSide) + braking;
			}
		}
	}

#ifdef OS_DEBUG
	void perFrameAnnotation(void)
	{
		const Vec3 p = this->position();

		// draw the circular collision boundary
		this->annotationCircleOrDisk(this->radius(), this->up(), p, gBlack, 32,
				false, false);

		// draw forward sensing corridor and wings ( for non-curved case)
		if (!curvedSteering)
		{
			const float corLength = this->speed() * lookAheadTimeOA();
			if (corLength > halfLength)
			{
				const Vec3 corFront = this->forward() * corLength;
				const Vec3 corBack = Vec3::zero; // (was bbFront)
				const Vec3 corSide = this->side() * this->radius();
				const Vec3 c1 = p + corSide + corBack;
				const Vec3 c2 = p + corSide + corFront;
				const Vec3 c3 = p - corSide + corFront;
				const Vec3 c4 = p - corSide + corBack;
				const Color color = (
						(annotateAvoid != Vec3::zero) ? gRed : gYellow);
				this->annotationLine(c1, c2, color);
				this->annotationLine(c2, c3, color);
				this->annotationLine(c3, c4, color);

				// draw sensing "wings"
				const Vec3 wingWidth = this->side() * wingSlope() * corLength;
				const Vec3 wingTipL = c2 + wingWidth;
				const Vec3 wingTipR = c3 - wingWidth;
				const Color wingColor(gOrange);
				if (wingDrawFlagL)
					this->annotationLine(c2, wingTipL, wingColor);
				if (wingDrawFlagL)
					this->annotationLine(c1, wingTipL, wingColor);
				if (wingDrawFlagR)
					this->annotationLine(c3, wingTipR, wingColor);
				if (wingDrawFlagR)
					this->annotationLine(c4, wingTipR, wingColor);
			}
		}

		// annotate steering acceleration
		const Vec3 above = this->position() + Vec3(0, 0.2f, 0);
		const Vec3 accel = this->smoothedAcceleration() * 5 / this->maxForce();
		const Color aColor(0.4f, 0.4f, 0.8f);
		this->annotationLine(above, above + accel, aColor);
	}

	// draw vehicle's body and annotation
	void draw(void)
	{
		// for now: draw as a 2d bounding box on the ground
		Color bodyColor(gBlack);
		if (stuck)
			bodyColor = gYellow;
		if (!bodyInsidePath())
			bodyColor = gOrange;
		if (collisionDetected)
			bodyColor = gRed;

		// draw vehicle's bounding box on gound plane (its "shadow")
		const Vec3 p = this->position();
		const Vec3 bbSide = this->side() * halfWidth;
		const Vec3 bbFront = this->forward() * halfLength;
		const Vec3 bbHeight(0, 0.1f, 0);
		drawQuadrangle(p - bbFront + bbSide + bbHeight,
				p + bbFront + bbSide + bbHeight,
				p + bbFront - bbSide + bbHeight,
				p - bbFront - bbSide + bbHeight, bodyColor);

		// annotate trail
		const Color darkGreen(0, 0.6f, 0);
		this->drawTrail(darkGreen, gBlack);
	}
#endif

	// called when steerToFollowPath decides steering is required
	void annotatePathFollowing(const Vec3& future, const Vec3& onPath,
			const Vec3& target, const float outside)
	{
#ifdef OS_DEBUG
		const Color toTargetColor(gGreen * 0.6f);
		const Color insidePathColor(gCyan * 0.6f);
		const Color outsidePathColor(gBlue * 0.6f);
		const Color futurePositionColor(0.5f, 0.5f, 0.6f);

		// draw line from our position to our predicted future position
		if (!curvedSteering)
			this->annotationLine(this->position(), future, futurePositionColor);

		// draw line from our position to our steering target on the path
		this->annotationLine(this->position(), target, toTargetColor);

		// draw a two-toned line between the future test point and its
		// projection onto the path, the change from dark to light color
		// indicates the boundary of the tube.

		const float o = outside + this->radius()
				+ (curvedSteering ? 1.0f : 0.0f);
		const Vec3 boundaryOffset = ((onPath - future).normalize() * o);

		const Vec3 onPathBoundary = future + boundaryOffset;
		this->annotationLine(onPath, onPathBoundary, insidePathColor);
		this->annotationLine(onPathBoundary, future, outsidePathColor);
#endif

		///call parent::annotatePathFollowing
		VehicleAddOnMixin<SimpleVehicle, Entity>::annotatePathFollowing(future,
				onPath, target, outside);
	}

///	GCRoute* makePath(void)
///	{
///		// a few constants based on world size
///		const float m = worldSize * 0.4f; // main diamond size
///		const float n = worldSize / 8;    // notch size
///		const float o = worldSize * 2;    // outside of the sand
///		// construction vectors
///		const Vec3 p(0, 0, m);
///		const Vec3 q(0, 0, m - n);
///		const Vec3 r(-m, 0, 0);
///		const Vec3 s(2 * n, 0, 0);
///		const Vec3 t(o, 0, 0);
///		const Vec3 u(-o, 0, 0);
///		const Vec3 v(n, 0, 0);
///		const Vec3 w(0, 0, 0);
///		// path vertices
///		const Vec3 a(t - p);
///		const Vec3 b(s + v - p);
///		const Vec3 c(s - q);
///		const Vec3 d(s + q);
///		const Vec3 e(s - v + p);
///		const Vec3 f(p - w);
///		const Vec3 g(r - w);
///		const Vec3 h(-p - w);
///		const Vec3 i(u - p);
///		// return Path object
///		const int pathPointCount = 9;
///		const Vec3 pathPoints[pathPointCount] =
///		{ a, b, c, d, e, f, g, h, i };
///		const float k = 10.0f;
///		const float pathRadii[pathPointCount] =
///		{ k, k, k, k, k, k, k, k, k };
///		return new GCRoute(pathPointCount, pathPoints, pathRadii, false);
///	}

	/*bool*/ void handleExitFromMap(void)
	{
		if(! map->isInside(this->position()))
		{
			if (demoSelect == 2)
			{
				// for path following, do wrap-around (teleport) and make new map
///				const float px = this->position().x;
///				const float fx = this->forward().x;
///				const float ws = worldSize * 0.51f; // slightly past edge
///				if (((fx > 0) && (px > ws)) || ((fx < 0) && (px < -ws)))
///				{
#ifdef OS_DEBUG
					// bump counters
					lapsStarted++;
					lapsFinished++;
#endif
					// set position on other side of the map (set new X coordinate)
///				this->setPosition(
///						(((px < 0) ? 1 : -1)
///								* ((worldSize * 0.5f)
///										+ (this->speed() * lookAheadTimePF()))),
///						this->position().y, this->position().z);
					resetToPath();

					// reset bookkeeping to detect stuck cycles
					resetStuckCycleDetection();

#ifdef OS_DEBUG
					// prevent long streaks due to teleportation
					this->clearTrailHistory();
#endif
///					return true;
///				}
			}
			else
			{
				// for the non-path-following demos:
				// reset simulation if the vehicle drives through the fence
///				if (this->position().length() > worldDiag)
///					this->reset();
				resetToMap();
			}
		}
		return /*false*/;
	}

	float wingSlope(void)
	{
		return interpolate(this->relativeSpeed(),
				(curvedSteering ? 0.3f : 0.35f), 0.06f);
	}

	void resetStuckCycleDetection(void)
	{
		this->resetSmoothedPosition(this->position() + (this->forward() * -80)); // qqq
	}

	// QQQ just a stop gap, not quite right
	// (say for example we were going around a circle with radius > 10)
	bool weAreGoingInCircles(void)
	{
		const Vec3 offset = this->smoothedPosition() - this->position();
		return offset.length() < 10;
	}

	float lookAheadTimeOA(void) const
	{
		const float minTime = (baseLookAheadTime
				* (curvedSteering ?
						interpolate(this->relativeSpeed(), 0.4f, 0.7f) : 0.66f));
		return combinedLookAheadTime(minTime, 3);
	}

	float lookAheadTimePF(void) const
	{
		return combinedLookAheadTime(baseLookAheadTime, 3);
	}

	// QQQ maybe move to SimpleVehicle ?
	// compute a "look ahead time" with two components, one based on
	// minimum time to (say) a collision and one based on minimum distance
	// arg 1 is "seconds into the future", arg 2 is "meters ahead"
	float combinedLookAheadTime(float minTime, float minDistance) const
	{
		if (this->speed() == 0)
			return 0;
		return maxXXX(minTime, minDistance / this->speed());
	}

	// is vehicle body inside the path?
	// (actually tests if all four corners of the bounding box are inside)
	//
	bool bodyInsidePath(void)
	{
		if (demoSelect == 2)
		{
			const Vec3 bbSide = this->side() * halfWidth;
			const Vec3 bbFront = this->forward() * halfLength;
			return ( /* path->isInsidePath (position () - bbFront + bbSide) */isInsidePathway(
					*static_cast<GCRoute*>((*path)[0]),
					this->position() - bbFront + bbSide)
					&&
					/* path->isInsidePath (position () + bbFront + bbSide) */isInsidePathway(
							*static_cast<GCRoute*>((*path)[0]),
							this->position() + bbFront + bbSide)
					&&
					/* path->isInsidePath (position () + bbFront - bbSide) */isInsidePathway(
							*static_cast<GCRoute*>((*path)[0]),
							this->position() + bbFront - bbSide)
					&&
					/* path->isInsidePath (position () - bbFront - bbSide) */isInsidePathway(
							*static_cast<GCRoute*>((*path)[0]),
							this->position() - bbFront - bbSide));
		}
		return true;
	}

	Vec3 convertAbsoluteToIncrementalSteering(const Vec3& absolute,
			const float elapsedTime)
	{
		const Vec3 curved = convertLinearToCurvedSpaceGlobal(absolute);
		blendIntoAccumulator(elapsedTime * 8.0f, curved, currentSteering);
#ifdef OS_DEBUG
		{
			// annotation
			const Vec3 u(0, 0.5, 0);
			const Vec3 p = this->position();
			this->annotationLine(p + u, p + u + absolute, gRed);
			this->annotationLine(p + u, p + u + curved, gYellow);
			this->annotationLine(p + u * 2, p + u * 2 + currentSteering,
					gGreen);
		}
#endif
		return currentSteering;
	}

	// QQQ new utility 2-25-04 -- may replace inline code elsewhere
	//
	// Given a location in this vehicle's linear local space, convert it into
	// the curved space defined by the vehicle's current path curvature.  For
	// example, forward() gets mapped on a point 1 unit along the circle
	// centered on the current center of curvature and passing through the
	// vehicle's position().
	//
	Vec3 convertLinearToCurvedSpaceGlobal(const Vec3& linear)
	{
		const Vec3 trimmedLinear = linear.truncateLength(this->maxForce());

		// ---------- this block imported from steerToAvoidObstaclesOnMap
		const float signedRadius = 1 / (nonZeroCurvatureQQQ() /*QQQ*/* 1);
		const Vec3 localCenterOfCurvature = this->side() * signedRadius;
		const Vec3 center = this->position() + localCenterOfCurvature;
		const float sign = signedRadius < 0 ? 1.0f : -1.0f;
		const float arcLength = trimmedLinear.dot(this->forward());
		//
		const float arcRadius = signedRadius * -sign;
		const float twoPi = 2 * OPENSTEER_M_PI;
		const float circumference = twoPi * arcRadius;
		const float arcAngle = twoPi * arcLength / circumference;
		// ---------- this block imported from steerToAvoidObstaclesOnMap

		// ---------- this block imported from scanObstacleMap
		// vector from center of curvature to position of vehicle
		const Vec3 initialSpoke = this->position() - center;
		// rotate by signed arc angle
		const Vec3 spoke = initialSpoke.rotateAboutGlobalY(arcAngle * sign);
		// ---------- this block imported from scanObstacleMap

		const Vec3 fromCenter = -localCenterOfCurvature.normalize();
		const float dRadius = trimmedLinear.dot(fromCenter);
		const float radiusChangeFactor = (dRadius + arcRadius) / arcRadius;
		const Vec3 resultLocation = center + (spoke * radiusChangeFactor);
#ifdef OS_DEBUG
		{
			const Vec3 center = this->position() + localCenterOfCurvature;
			annotationXZArc(this->position(), center, this->speed() * sign * -3,
					20, gWhite);
		}
#endif
		// return the vector from vehicle position to the coimputed location
		// of the curved image of the original linear offset
		return resultLocation - this->position();
	}

	// approximate value for the Polaris Ranger 6x6: 16 feet, 5 meters
	float minimumTurningRadius() const
	{
		return 5.0f;
	}

	Vec3 adjustSteeringForMinimumTurningRadius(const Vec3& steering)
	{
		const float maxCurvature = 1 / (minimumTurningRadius() * 1.1f);

		// are we turning more sharply than the minimum turning radius?
		if (absXXX(this->curvature()) > maxCurvature)
		{
			// remove the tangential (non-thrust) component of the steering
			// force, replace it with a force pointing away from the center
			// of curvature, causing us to "widen out" easing off from the
			// minimum turning radius
			const float signedRadius = 1 / nonZeroCurvatureQQQ();
			const float sign = signedRadius < 0 ? 1.0f : -1.0f;
			const Vec3 thrust = steering.parallelComponent(this->forward());
			const Vec3 trimmed = thrust.truncateLength(this->maxForce());
			const Vec3 widenOut = this->side() * this->maxForce() * sign;
#ifdef OS_DEBUG
			{
				// annotation
				const Vec3 localCenterOfCurvature = this->side() * signedRadius;
				const Vec3 center = this->position() + localCenterOfCurvature;
				this->annotationCircleOrDisk(minimumTurningRadius(), this->up(),
						center, gBlue, 40, false, false);
			}
#endif
			return trimmed + widenOut;
		}

		// otherwise just return unmodified input
		return steering;
	}

	// QQQ This is to work around the bug that scanObstacleMap's current
	// QQQ arguments preclude the driving straight [curvature()==0] case.
	// QQQ This routine returns the current vehicle path curvature, unless it
	// QQQ is *very* close to zero, in which case a small positive number is
	// QQQ returned (corresponding to a radius of 100,000 meters).
	// QQQ
	// QQQ Presumably it would be better to get rid of this routine and
	// QQQ redesign the arguments of scanObstacleMap
	//
	float nonZeroCurvatureQQQ(void) const
	{
		const float c = this->curvature();
		const float minCurvature = 1.0f / 100000.0f; // 100,000 meter radius
		const bool tooSmall = (c < minCurvature) && (c > -minCurvature);
		return (tooSmall ? minCurvature : c);
	}

	// QQQ ad hoc speed limitation based on path orientation...
	// QQQ should be renamed since it is based on more than curvature
	//
	float maxSpeedForCurvature()
	{
		float maxRelativeSpeed = 1;

		if (curvedSteering)
		{
			// compute an ad hoc "relative curvature"
			const float absC = absXXX(this->curvature());
			const float maxC = 1 / minimumTurningRadius();
			const float relativeCurvature = sqrtXXX(clip(absC / maxC, 0, 1));

			// map from full throttle when straight to 10% at max curvature
			const float curveSpeed = interpolate(relativeCurvature, 1.0f, 0.1f);
#ifdef OS_DEBUG
			annoteMaxRelSpeedCurve = curveSpeed;
#endif
			if (demoSelect != 2)
			{
				maxRelativeSpeed = curveSpeed;
			}
			else
			{
				// heading (unit tangent) of the path segment of interest
				const Vec3 pathHeading = mapPointAndDirectionToTangent(*static_cast<GCRoute*>((*path)[0]),
						this->position(), pathFollowDirection); // path->tangentAt (position (), pathFollowDirection);
				// measure how parallel we are to the path
				const float parallelness = pathHeading.dot(this->forward());

				// determine relative speed for this heading
				const float mw = 0.2f;
				const float headingSpeed = (
						(parallelness < 0) ?
								mw : interpolate(parallelness, mw, 1.0f));
				maxRelativeSpeed = minXXX(curveSpeed, headingSpeed);
#ifdef OS_DEBUG
				annoteMaxRelSpeedPath = headingSpeed;
#endif
			}
		}
#ifdef OS_DEBUG
		annoteMaxRelSpeed = maxRelativeSpeed;
#endif
		return this->maxSpeed() * maxRelativeSpeed;
	}

	// xxx library candidate
	// xxx assumes (but does not check or enforce) heading is unit length
	//
	Vec3 steerTowardHeading(const Vec3& desiredGlobalHeading)
	{
		const Vec3 headingError = desiredGlobalHeading - this->forward();
		return headingError.normalize() * this->maxForce();
	}

#ifdef OS_DEBUG
	// XXX this should eventually be in a library, make it a first
	// XXX class annotation queue, tie in with drawXZArc
	void annotationXZArc(const Vec3& start, const Vec3& center,
			const float arcLength, const int segments, const Color& color) const
	{
		// "spoke" is initially the vector from center to start,
		// it is then rotated around its tail
		Vec3 spoke = start - center;

		// determine the angular step per segment
		const float radius = spoke.length();
		const float twoPi = 2 * OPENSTEER_M_PI;
		const float circumference = twoPi * radius;
		const float arcAngle = twoPi * arcLength / circumference;
		const float step = arcAngle / segments;

		// draw each segment along arc
		float sin = 0, cos = 0;
		for (int i = 0; i < segments; i++)
		{
			const Vec3 old = spoke + center;

			// rotate point to next step around circle
			spoke = spoke.rotateAboutGlobalY(step, sin, cos);

			this->annotationLine(spoke + center, old, color);
		}
	}
#endif

	// map of obstacles
	TerrainMap* map;

	// route for path following (waypoints and legs)
//	GCRoute* path;
	PathwayGroup* path;

	// follow the path "upstream or downstream" (+1/-1)
	int pathFollowDirection;

	// master look ahead (prediction) time
	float baseLookAheadTime;

	// vehicle dimensions in meters
	float halfWidth;
	float halfLength;

	// keep track of path following failure rate
	// (these are probably obsolete now, replaced by stuckOffPathCount)
	float pathFollowTime;
	float pathFollowOffTime;

	// QQQ first pass at detecting "stuck" state
	bool stuck;

	Vec3 qqqLastNearestObstacle;

	// QQQ temporary global QQQoaJustScraping
	// QQQ replace this global flag with a cleaner mechanism
	bool QQQoaJustScraping;

	// for "curvature-based incremental steering" -- contains the current
	// steering into which new incremental steering is blended
	Vec3 currentSteering;

	// use curved prediction and incremental steering:
	bool curvedSteering;
	bool incrementalSteering;

	// which of the three demo modes is selected
	int demoSelect;

	// size of the world (the map actually)
	float worldSize;
	float worldDiag;

#ifdef OS_DEBUG
	// keep track of failure rate (when vehicle is on top of obstacle)
	bool collisionDetected;
	bool collisionLastTime;
	float timeOfLastCollision;
	float sumOfCollisionFreeTimes;
	int countOfCollisionFreeTimes;
	// keep track of average speed
	float totalDistance;
	float totalTime;
	// state saved for annotation
	Vec3 annotateAvoid;
	bool wingDrawFlagL, wingDrawFlagR;
	//stuck
	int stuckCount;
	int stuckCycleCount;
	int stuckOffPathCount;
	//
	int lapsStarted;
	int lapsFinished;
	// take note when current dt is zero (as in paused) for stat counters
	bool dtZero;
	int hintGivenCount;
	int hintTakenCount;
	// save obstacle avoidance stats for annotation
	// (nearest obstacle in each of the four zones)
	float savedNearestWR, savedNearestR, savedNearestL, savedNearestWL;
	float annoteMaxRelSpeed, annoteMaxRelSpeedCurve, annoteMaxRelSpeedPath;
	float windowWidth;
#endif
};

//MapDriver externally updated.
template<typename Entity>
class ExternalMapDriver: public MapDriver<Entity>
{
public:

	void update(const float currentTime, const float elapsedTime)
	{
		//call the entity update
		this->entityUpdate(currentTime, elapsedTime);

#ifdef OS_DEBUG
		// take note when current dt is zero (as in paused) for stat counters
		this->dtZero = (elapsedTime == 0);
		// pretend we are bigger when going fast
		this->adjustVehicleRadiusForSpeed();
		// state saved for speedometer
		//      annoteMaxRelSpeed = annoteMaxRelSpeedCurve = annoteMaxRelSpeedPath = 0;
		this->annoteMaxRelSpeed = this->annoteMaxRelSpeedCurve =
		this->annoteMaxRelSpeedPath = 1;
		// determine combined steering
		const bool offPath = !this->bodyInsidePath();
		if (this->stuck || offPath || this->detectImminentCollision())
		{
			// count "off path" events
			if (offPath && !this->stuck && (this->demoSelect == 2))
			this->stuckOffPathCount++;
			this->stuck = true;
		}
		else
		{
			// determine steering for obstacle avoidance (save for annotation)
			const Vec3 avoid = this->annotateAvoid =
			this->steerToAvoidObstaclesOnMap(this->lookAheadTimeOA(),
					*this->map, this->hintForObstacleAvoidance());
			const bool needToAvoid = avoid != Vec3::zero;

			// any obstacles to avoid?
			if (needToAvoid)
			{
				// slow down and turn to avoid the obstacles
				const float targetSpeed = (
						(this->curvedSteering && this->QQQoaJustScraping) ?
						this->maxSpeedForCurvature() : 0);
				this->annoteMaxRelSpeed = targetSpeed / this->maxSpeed();
			}
			else
			{
				// wander for demo 1
				if (this->demoSelect == 1)
				{
					const Vec3 wander = this->steerForWander(elapsedTime);
					const Vec3 flat = wander.setYtoZero();
					const Vec3 weighted = flat.truncateLength(this->maxForce())
					* 6;
					const Vec3 a = this->position() + Vec3(0, 0.2f, 0);
					this->annotationLine(a, a + (weighted * 0.3f), gWhite);
				}
				// follow the path in demo 2
				if (this->demoSelect == 2)
				{
					const Vec3 pf = this->steerToFollowPath(
							this->pathFollowDirection, this->lookAheadTimePF(),
							*static_cast<GCRoute*>((*this->path)[0]));
					if (pf == Vec3::zero)
					{
						//XXX
						// path alignment: when neither obstacle avoidance nor
						// path following is required, align with path segment
						const Vec3 pathHeading = mapPointAndDirectionToTangent(
								*static_cast<GCRoute*>((*this->path)[0]),
									this->position(), this->pathFollowDirection);// path->tangentAt (position (), pathFollowDirection);
						{
							const Vec3 b =
							(this->position() + (this->up() * 0.2f)
									+ (this->forward()
											* this->halfLength * 1.4f));
							const float l = 2;
							this->annotationLine(b, b + (this->forward() * l),
									gCyan);
							this->annotationLine(b, b + (pathHeading * l),
									gCyan);
						}
					}
				}
			}
		}
		this->collectReliabilityStatistics(currentTime, elapsedTime);
		// detect getting stuck in cycles -- we are moving but not
		// making progress down the route (annotate smoothedPosition)
		if (this->demoSelect == 2)
		{
			const bool circles = this->weAreGoingInCircles();
			if (circles && !this->stuck)
				this->stuckCycleCount++;
			this->annotationCircleOrDisk(0.5, this->up(),
					this->smoothedPosition(), gWhite, 12, circles, false);
			if (circles)
				this->stuck = true;
		}
		// annotation
		this->perFrameAnnotation();
		this->recordTrailVertex(currentTime, this->position());
#endif
	}
};

// ----------------------------------------------------------------------------
// PlugIn for OpenSteerDemo

/**
 * \note: After opening the plugin and before adding a
 * vehicle you should call makeMap.
 * \note: obstacles are "projected" along y axis over the map:
 * no height dimension (y) is taken into account.
 * \note: use setOptions when setting these options:
 * - demoSelect
 * - usePathFences
 */
template<typename Entity>
class MapDrivePlugIn: public PlugIn
{
public:

	const char* name(void)
	{
		return "Driving through map based obstacles";
	}

	float selectionOrderSortKey(void)
	{
		return 0.07f;
	}

	// be more "nice" to avoid a compiler warning
	virtual ~MapDrivePlugIn()
	{
	}

	void open(void)
	{
		// make new MapDriver
///		vehicle = new MapDriver();
///		vehicles.push_back(vehicle);
///		selectedVehicle = vehicle;

		demoSelect = 2;
		curvedSteering = true;
		//set a NULL map
		map = NULL;

#ifdef OS_DEBUG
		windowWidth = 1.0;
#endif

		// marks as obstacles map cells adjacent to the path
		usePathFences = true;
///		// scatter random rock clumps over map
///		useRandomRocks = true;

		worldSize = 1.0;
		worldDiag = sqrtXXX(square(worldSize) / 2);
		worldCenter = Vec3();
		worldResolution = 1;

///		// reset this plugin
///		reset();
	}

	void update(const float currentTime, const float elapsedTime)
	{
		// update simulation of test vehicle
		iterator iter;
		for (iter = vehicles.begin(); iter != vehicles.end(); ++iter)
		{
			(*iter)->update(currentTime, elapsedTime);

			// when vehicle drives outside the world
///			if ((*iter)->handleExitFromMap())
///				regenerateMap();
///
///			// QQQ first pass at detecting "stuck" state
///			if ((*iter)->stuck && ((*iter)->relativeSpeed() < 0.001f))
///			{
///#ifdef OS_DEBUG
///				(*iter)->stuckCount++;
///#endif
///				reset();
///			}
		}
	}

	void redraw(const float currentTime, const float elapsedTime)
	{
#ifdef OS_DEBUG
		// draw "ground plane"  (make it 4x map size)
///		const float s = worldSize * 2;
///		const float u = -0.2f;
///		drawQuadrangle(Vec3(+s, u, +s), Vec3(+s, u, -s), Vec3(-s, u, -s),
///				Vec3(-s, u, +s), Color(0.8f, 0.7f, 0.5f)); // "sand"

		///FIXME: delegated to external plugin initialization
		// draw map and path
///		drawMap();
///		drawPath();

		// draw test vehicle
		iterator iter;
		for (iter = vehicles.begin(); iter != vehicles.end(); ++iter)
		{
			(*iter)->draw();
		}

		// QQQ mark origin to help spot artifacts
		const float tick = 2;
		drawLine(Vec3(tick, 0, 0), Vec3(-tick, 0, 0), gGreen);
		drawLine(Vec3(0, 0, tick), Vec3(0, 0, -tick), gGreen);

		// compute conversion factor miles-per-hour to meters-per-second
		const float metersPerMile = 1609.344f;
		const float secondsPerHour = 3600;
		const float MPSperMPH = metersPerMile / secondsPerHour;

		// display status in the upper left corner of the window
		MapDriver<Entity>* vehicle =
				static_cast<MapDriver<Entity>*>(selectedVehicle);
		if (vehicle)
		{
			std::ostringstream status;
			status << "Speed: " << (int) vehicle->speed() << " mps ("
					<< (int) (vehicle->speed() / MPSperMPH) << " mph)"
					<< ", average: " << std::setprecision(1)
					<< std::setiosflags(std::ios::fixed)
					<< vehicle->totalDistance / vehicle->totalTime

					<< " mps\n\n";
			status << "collisions avoided for "
					<< (int) (ClockObject::get_global_clock()->get_real_time()
							- vehicle->timeOfLastCollision) << " seconds";
			if (vehicle->countOfCollisionFreeTimes > 0)
			{
				status << "\nmean time between collisions: "
						<< (int) (vehicle->sumOfCollisionFreeTimes
								/ vehicle->countOfCollisionFreeTimes) << " ("
						<< (int) vehicle->sumOfCollisionFreeTimes << "/"
						<< (int) vehicle->countOfCollisionFreeTimes << ")";
			}

			status << "\n\nStuck count: " << vehicle->stuckCount << " ("
					<< vehicle->stuckCycleCount << " cycles, "
					<< vehicle->stuckOffPathCount << " off path)";
			status << "\n\n[F1] ";
			if (1 == vehicle->demoSelect)
				status << "wander, ";
			if (2 == vehicle->demoSelect)
				status << "follow path, ";
			status << "avoid obstacle";

			if (2 == vehicle->demoSelect)
			{
				status << "\n[F2] path following direction: ";
				if (vehicle->pathFollowDirection > 0)
					status << "+1";
				else
					status << "-1";
				status << "\n[F3] path fence: ";
				if (usePathFences)
					status << "on";
				else
					status << "off";
			}

///		status << "\n[F4] rocks: ";
///		if (useRandomRocks)
///			status << "on";
///		else
///			status << "off";
			status << "\n[F5] prediction: ";
			if (vehicle->curvedSteering)
				status << "curved";
			else
				status << "linear";
			if (2 == vehicle->demoSelect)
			{
				status << "\n\nLap " << vehicle->lapsStarted << " (completed: "
						<< ((vehicle->lapsStarted < 2) ?
								0 :
								(int) (100
										* ((float) vehicle->lapsFinished
												/ (float) (vehicle->lapsStarted
														- 1)))) << "%)";

				status << "\nHints given: " << vehicle->hintGivenCount
						<< ", taken: " << vehicle->hintTakenCount;
			}
			status << "\n";
			qqqRange("WR ", vehicle->savedNearestWR, status);
			qqqRange("R  ", vehicle->savedNearestR, status);
			qqqRange("L  ", vehicle->savedNearestL, status);
			qqqRange("WL ", vehicle->savedNearestWL, status);
			status << std::ends;
/////		const float h = drawGetWindowHeight();
/////		const Vec3 screenLocation(10, h - 50, 0);
			const Vec3 screenLocation(-1.0, 0.9, 0);
			const Color color(0.15f, 0.15f, 0.5f);
/////		draw2dTextAt2dLocation(status, screenLocation, color,
/////				drawGetWindowWidth(), drawGetWindowHeight());
			draw2dTextAt2dLocation(status, screenLocation, color, 1.0, 1.0);

			{
				const float v = 5;
				const float m = 10;
/////			const float w = drawGetWindowWidth();
				const float w = windowWidth;
				const float f = w - (2 * m);
				const float s = vehicle->relativeSpeed();

				// limit tick mark
				const float l = vehicle->annoteMaxRelSpeed;
/////			draw2dLine(Vec3(m + (f * l), v - 3, 0), Vec3(m + (f * l), v + 3, 0),
/////					gBlack, drawGetWindowWidth(), drawGetWindowHeight());
				draw2dLine(Vec3(m + (f * l), v - 3, 0),
						Vec3(m + (f * l), v + 3, 0), gBlack, 0.0, 0.0);
				// two "inverse speedometers" showing limits due to curvature and
				// path alignment
				if (l != 0)
				{
					const float c = vehicle->annoteMaxRelSpeedCurve;
					const float p = vehicle->annoteMaxRelSpeedPath;
/////				draw2dLine(Vec3(m + (f * c), v + 1, 0), Vec3(w - m, v + 1, 0),
/////						gRed, drawGetWindowWidth(), drawGetWindowHeight());
/////				draw2dLine(Vec3(m + (f * p), v - 2, 0), Vec3(w - m, v - 1, 0),
/////						gGreen, drawGetWindowWidth(), drawGetWindowHeight());
					draw2dLine(Vec3(m + (f * c), v + 1, 0),
							Vec3(w - m, v + 1, 0), gRed, 0.0, 0.0);
					draw2dLine(Vec3(m + (f * p), v - 2, 0),
							Vec3(w - m, v - 1, 0), gGreen, 0.0, 0.0);
				}
				// speedometer: horizontal line with length proportional to speed
/////			draw2dLine(Vec3(m, v, 0), Vec3(m + (f * s), v, 0), gWhite,
/////					drawGetWindowWidth(), drawGetWindowHeight());
				draw2dLine(Vec3(m, v, 0), Vec3(m + (f * s), v, 0), gWhite, 0.0,
						0.0);
				// min and max tick marks
/////			draw2dLine(Vec3(m, v, 0), Vec3(m, v - 2, 0), gWhite,
/////					drawGetWindowWidth(), drawGetWindowHeight());
/////			draw2dLine(Vec3(w - m, v, 0), Vec3(w - m, v - 2, 0), gWhite,
/////					drawGetWindowWidth(), drawGetWindowHeight());
				draw2dLine(Vec3(m, v, 0), Vec3(m, v - 2, 0), gWhite, 0.0, 0.0);
				draw2dLine(Vec3(w - m, v, 0), Vec3(w - m, v - 2, 0), gWhite,
						0.0, 0.0);
			}
		}
#endif
	}

	void qqqRange(char const* string, float range, std::ostringstream& status)
	{
		status << "\n" << string;
		if (range == 9999.0f)
			status << "--";
		else
			status << (int) range;
	}

	void close(void)
	{
///		vehicles.clear();
///		delete (vehicle);
		delete map;
		map = NULL;
	}

	void reset(void)
	{
///		regenerateMap();

		// reset each vehicle
		iterator iter;
		for (iter = vehicles.begin(); iter != vehicles.end(); ++iter)
		{
			(*iter)->reset();
		}
	}

///	void handleFunctionKeys(int keyNumber)
///	{
///		switch (keyNumber)
///		{
///		case 1:
///			selectNextDemo();
///			break;
///		case 2:
///			reversePathFollowDirection();
///			break;
///		case 3:
///			togglePathFences();
///			break;
///		case 4:
///			toggleRandomRocks();
///			break;
///		case 5:
///			toggleCurvedSteering();
///			break;
///		case 6: // QQQ draw an enclosed "pen" of obstacles to test cycle-stuck
///		{
///			const float m = MapDriver::worldSize * 0.4f; // main diamond size
///			const float n = MapDriver::worldSize / 8;    // notch size
///			const Vec3 q(0, 0, m - n);
///			const Vec3 s(2 * n, 0, 0);
///			const Vec3 c(s - q);
///			const Vec3 d(s + q);
///			const int pathPointCount = 2;
///			const float pathRadii[pathPointCount] =
///			{ 10, 10 };
///			const Vec3 pathPoints[pathPointCount] =
///			{ c, d };
///			GCRoute r(pathPointCount, pathPoints, pathRadii, false);
///			drawPathFencesOnMap(*vehicle->map, r);
///			break;
///		}
///		}
///	}
///	void printMiniHelpForFunctionKeys(void)
///	{
///		std::ostringstream message;
///		message << "Function keys handled by ";
///		message << '"' << name() << '"' << ':' << std::ends;
/////		OpenSteerDemo::printMessage(message);
/////		OpenSteerDemo::printMessage("  F1     select next driving demo.");
/////		OpenSteerDemo::printMessage(
/////				"  F2     reverse path following direction.");
/////		OpenSteerDemo::printMessage("  F3     toggle path fences.");
/////		OpenSteerDemo::printMessage("  F4     toggle random rock clumps.");
/////		OpenSteerDemo::printMessage("  F5     toggle curved prediction.");
/////		OpenSteerDemo::printMessage("");
///	}

	virtual bool addVehicle(AbstractVehicle* vehicle)
	{
		if (! PlugInAddOnMixin<OpenSteer::PlugIn>::addVehicle(vehicle))
		{
			return false;
		}
		//pathway must be GCRoute (a.k.a. PolylineSegmentedPathwaySegmentRadii)
		if(!dynamic_cast<GCRoute*>(m_pathway[0]))
		{
			return false;
		}
		// try to allocate a token for this pedestrian in the proximity database
		MapDriver<Entity>* mapDriver = dynamic_cast<MapDriver<Entity>*>(vehicle);
		if (mapDriver && map)
		{
			//set map
			mapDriver->map = map;
			//set world size
			mapDriver->worldSize = worldSize;
			mapDriver->worldDiag = worldDiag;
			//set path
			mapDriver->path = &m_pathway;
			//set demo select
			mapDriver->demoSelect = demoSelect;
			//set curved steering
			mapDriver->curvedSteering = curvedSteering;
#ifdef OS_DEBUG
			mapDriver->windowWidth = windowWidth;
#endif
			// vehicle is 2 * radius meters wide and 3 * radius meters long
			mapDriver->halfWidth = mapDriver->radius() * 1.0f;
			mapDriver->halfLength = mapDriver->radius() * 1.5f;
			//set result
			return true;
		}
		//roll back addition
		PlugInAddOnMixin<OpenSteer::PlugIn>::removeVehicle(vehicle);
		//
		return false;
	}

	void reversePathFollowDirection(MapDriver<Entity>* vehicle)
	{
		int& pfd = vehicle->pathFollowDirection;
		pfd = (pfd > 0) ? -1 : +1;
	}

	void setOptions(int _demoSelect = 2, bool _usePathFences = true,
			bool _curvedSteering = true)
	{
		bool dirty = false;
		if (demoSelect != _demoSelect)
		{
			//update
			demoSelect = _demoSelect;
			dirty = true;
		}
		if (usePathFences != _usePathFences)
		{
			usePathFences = _usePathFences;
			dirty = true;
		}
		if (dirty)
		{
			regenerateMap();
		}
		curvedSteering = _curvedSteering;
	}

	void setDemoSelect(int _demoSelect = 2)
	{
		if (demoSelect != _demoSelect)
		{
			//update
			demoSelect = _demoSelect;
			regenerateMap();
		}
	}

	int getDemoSelect() const
	{
		return demoSelect;
	}

	void setUsePathFences(bool _usePathFences = true)
	{
		if (usePathFences != _usePathFences)
		{
			usePathFences = _usePathFences;
			regenerateMap();
		}
	}

	bool getUsePathFences() const
	{
		return usePathFences;
	}

	void setCurvedSteering(bool _curvedSteering = true)
	{
		curvedSteering = _curvedSteering;
		// update curvedSteering of each vehicles
		iterator iter;
		for (iter = vehicles.begin(); iter != vehicles.end(); ++iter)
		{
			(*iter)->curvedSteering = curvedSteering;
		}
	}

	bool getCurvedSteering() const
	{
		return curvedSteering;
	}

///	void togglePathFences(void)
///	{
///		usePathFences = !usePathFences;
///		reset();
///	}
///	void toggleRandomRocks(void)
///	{
///		useRandomRocks = !useRandomRocks;
///		reset();
///	}
///	void toggleCurvedSteering(void)
///	{
///		vehicle->curvedSteering = !vehicle->curvedSteering;
///		vehicle->incrementalSteering = !vehicle->incrementalSteering;
///		reset();
///	}
///	void selectNextDemo(void)
///	{
///		std::ostringstream message;
///		message << name() << ": ";
///		switch (++vehicle->demoSelect)
///		{
///		default:
///			vehicle->demoSelect = 0; // wrap-around, falls through to case 0:
///		case 0:
///			message << "obstacle avoidance and speed control";
///			reset();
///			break;
///		case 1:
///			message << "wander, obstacle avoidance and speed control";
///			reset();
///			break;
///		case 2:
///			message << "path following, obstacle avoidance and speed control";
///			reset();
///			break;
///		}
///		message << std::ends;
///	}

	void regenerateMap(void)
	{
		//if map not made return
		if (map == NULL)
		{
			return;
		}
		// regenerate map: clear and add random "rocks"
		map->clear();
		drawRandomClumpsOfRocksOnMap();
///		clearCenterOfMap();

		// draw fences for first two demo modes
		if (demoSelect < 2)
			drawBoundaryFencesOnMap(*map);
///		// randomize path widths
///		if (vehicle->demoSelect == 2)
///		{
///			const OpenSteer::size_t count = vehicle->path->segmentCount();
///			const bool upstream = vehicle->pathFollowDirection > 0;
///			const OpenSteer::size_t entryIndex = upstream ? 0 : count - 1;
///			const OpenSteer::size_t exitIndex = upstream ? count - 1 : 0;
///			const float lastExitRadius = vehicle->path->segmentRadius(
///					exitIndex);
///			for (OpenSteer::size_t i = 0; i < count; i++)
///			{
///				vehicle->path->setSegmentRadius(i, frandom2(4, 19));
///			}
///			vehicle->path->setSegmentRadius(entryIndex, lastExitRadius);
///		}
		// mark path-boundary map cells as obstacles
		// (when in path following demo and appropriate mode is set)
		if (usePathFences && (demoSelect == 2))
			drawPathFencesOnMap(*map, static_cast<GCRoute&>(*m_pathway[0]));

		// update references of all vehicles to the new map
		iterator iter;
		for (iter = vehicles.begin(); iter != vehicles.end(); ++iter)
		{
			//set map
			(*iter)->map = map;
			//set world size
			(*iter)->worldSize = worldSize;
			(*iter)->worldDiag = worldDiag;
		}
	}

#ifdef OS_DEBUG
	void drawMap(void)
	{
#ifdef OLDTERRAINMAP
		//if map not made return
		if (map == NULL)
		{
			return;
		}
		const float xs = map->xSize / (float) map->resolution;
		const float zs = map->zSize / (float) map->resolution;
		const Vec3 alongRow(xs, 0, 0);
		const Vec3 nextRow(-map->xSize, 0, zs);
		Vec3 g((map->xSize - xs) / -2, 0, (map->zSize - zs) / -2);
		g += map->center;
		for (int j = 0; j < map->resolution; j++)
		{
			for (int i = 0; i < map->resolution; i++)
			{
				if (map->getMapBit(i, j))
				{
					// spikes
					// const Vec3 spikeTop (0, 5.0f, 0);
					// drawLine (g, g+spikeTop, gWhite);

					// squares
					const float rockHeight = 0;
					const Vec3 v1(+xs / 2, rockHeight, +zs / 2);
					const Vec3 v2(+xs / 2, rockHeight, -zs / 2);
					const Vec3 v3(-xs / 2, rockHeight, -zs / 2);
					const Vec3 v4(-xs / 2, rockHeight, +zs / 2);
					// const Vec3 redRockColor (0.6f, 0.1f, 0.0f);
					const Color orangeRockColor(0.5f, 0.2f, 0.0f);
					drawQuadrangle(g + v1, g + v2, g + v3, g + v4,
							orangeRockColor);

					// pyramids
					// const Vec3 top (0, xs/2, 0);
					// const Vec3 redRockColor (0.6f, 0.1f, 0.0f);
					// const Vec3 orangeRockColor (0.5f, 0.2f, 0.0f);
					// drawTriangle (g+v1, g+v2, g+top, redRockColor);
					// drawTriangle (g+v2, g+v3, g+top, orangeRockColor);
					// drawTriangle (g+v3, g+v4, g+top, redRockColor);
					// drawTriangle (g+v4, g+v1, g+top, orangeRockColor);
				}
				g += alongRow;
			}
			g += nextRow;
		}
#else
#endif
	}

	/**
	 * draw the GCRoute as a series of circles and "wide lines"
	 * (QQQ this should probably be a method of Path (or a
	 * closossup-related utility function) in which case should pass
	 * color in, certainly shouldn't be recomputing it each draw)
	 * @todo Add a <code>Vec3 const* points() const</code> member function to
	 *       SegmentedPath, etc. to allow for faster point access?
	 */
	void drawPath(void)
	{
		const Color pathColor(0, 0.5f, 0.5f);
		const Color sandColor(0.8f, 0.7f, 0.5f);
		const Color color = interpolate(0.1f, sandColor, pathColor);

		// draw a line along each segment of path
		const GCRoute& path = dynamic_cast<GCRoute&>(*m_pathway[0]);
		const Vec3 down(0, -0.1f, 0);
		for (OpenSteer::SegmentedPathway::size_type i = 1;
				i < path.pointCount(); ++i)
		{
			const Vec3 endPoint0 = path.point(i) + down;
			const Vec3 endPoint1 = path.point(i - 1) + down;

			const float legWidth = path.segmentRadius(i - 1);

			drawXZWideLine(endPoint0, endPoint1, color, legWidth * 2);
			drawLine(path.point(i), path.point(i - 1), pathColor);
			drawXZDisk(legWidth, endPoint0, color, 24);
			drawXZDisk(legWidth, endPoint1, color, 24);
		}
	}
#endif

	void drawRandomClumpsOfRocksOnMap()
	{
		///Projects all obstacles over the map using Bresenham's rasterizing algorithms.
		///This is equivalent to an orthographic projection along the y axis, with
		///a camera located at y=+infinity and looking downward.

		//exit if no obstacle
		if (obstacles->empty())
		{
			return;
		}
		//initialize minX, maxX buffers
		int* minX = new int[map->resolution];
		int* maxX = new int[map->resolution];
		for (int i = 0; i < map->resolution; ++i)
		{
			minX[i] = INT_MAX;
			maxX[i] = INT_MIN;
		}
		int minZ = INT_MAX;
		int maxZ = INT_MIN;

		//project each obstacle over map
		OpenSteer::ObstacleGroup::const_iterator iter;
		for (iter = obstacles->begin(); iter != obstacles->end(); ++iter)
		{
			//get map vertices (ccw defined looking down y axis)
			const float dX = map->xSize / 2.0;
			const float dZ = map->zSize / 2.0;
			Vec3 mapV[4] =
			{ map->center + Vec3(-dX, 0, dZ), map->center + Vec3(dX, 0, dZ),
					map->center + Vec3(dX, 0, -dZ), map->center
							+ Vec3(-dX, 0, -dZ) };

			if (dynamic_cast<SphereObstacle*>(*iter))
			{
				SphereObstacle* sphere = dynamic_cast<SphereObstacle*>(*iter);
				//cull away if sphere (projection) is outside map
				if (! rectangleCircleIntersect(mapV, sphere->center,
						sphere->radius))
				{
					continue;
				}

				//(projection) is inside map: get sphere (integer) parameters
				int ir = (int) ceilf(sphere->radius);
				int ixc, izc;
				map->getCoords(sphere->center, ixc, izc);
				//project sphere over the map
				rasterizeCircle(ixc, izc, ir, minX, maxX, &minZ, &maxZ);
			}
			else if (dynamic_cast<BoxObstacle*>(*iter))
			{
				BoxObstacle* box = dynamic_cast<BoxObstacle*>(*iter);
				//box is a convex polyhedron: get its 12 triangles
				//defined front facing (i.e. ccw wrt external normal),
				//and rasterize only those resulting front facing
				//when projecting downward along y axis.
				//
				//		    6--------7
				//		   /|       /|
				//		  / |      / |
				//       /  |     /  | 2h
				//		3--------2   |
				//		|   |    |   |
				//		|   5----|---4              u
				//	    |  /     |  /               |__f
				//		| /      | / 2w            /
				//		|/       |/               s
				//		0--------1
				//          2d
				//
				//get box features
				float w = box->width * 0.5f;
				float h = box->height * 0.5f;
				float d = box->depth * 0.5f;
				Vec3 v[8];
				v[0] = box->globalizePosition(OpenSteer::Vec3(w, -h, -d));
				v[1] = box->globalizePosition(OpenSteer::Vec3(w, -h, d));
				v[2] = box->globalizePosition(OpenSteer::Vec3(w, h, d));
				v[3] = box->globalizePosition(OpenSteer::Vec3(w, h, -d));
				v[4] = box->globalizePosition(OpenSteer::Vec3(-w, -h, d));
				v[5] = box->globalizePosition(OpenSteer::Vec3(-w, -h, -d));
				v[6] = box->globalizePosition(OpenSteer::Vec3(-w, h, -d));
				v[7] = box->globalizePosition(OpenSteer::Vec3(-w, h, d));
				//get triangles
				struct Triangle
				{
					int v0I, v1I, v2I;
				};
				Triangle tri[12] =
				{
				{ 0, 1, 2 },
				{ 0, 2, 3 },
				{ 1, 4, 7 },
				{ 1, 7, 2 },
				{ 4, 5, 6 },
				{ 4, 6, 7 },
				{ 5, 0, 3 },
				{ 5, 3, 6 },
				{ 3, 2, 7 },
				{ 3, 7, 6 },
				{ 1, 0, 5 },
				{ 1, 5, 4 } };
				//try to project triangles
				for (int i = 0; i < 12; ++i)
				{
					Vec3 triV[3];
					triV[0] = v[tri[i].v0I];
					triV[1] = v[tri[i].v1I];
					triV[2] = v[tri[i].v2I];
//					//cull if back facing triangle (i.e. triangle resulting
//					//cw defined looking down y axis): det >= 0 ==> ccw
//					//see "D.H. Eberly: 3D Game Engine Design, 2nd edition"
//					int detI = (triV[2].x - triV[0].x) * (triV[1].z - triV[0].z)
//							- (triV[1].x - triV[0].x) * (triV[2].z - triV[0].z);
//					if (detI < 0)
//					{
//						continue;
//					}
					//general formula of ccw-ness looking down an axis J
					//(V1-V0).cross(V2-V0).dot(J) >= 0 ==> ccw
					//where:
					//	- V0,V1,V2 = triangle vertices
					//	- J = axis vector
					Vec3 VCross;
					VCross.cross(triV[1] - triV[0], triV[2] - triV[0]);
					float detF = VCross.dot(Vec3(0, 1, 0));
					if (detF < 0)
					{
						continue;
					}
					//cull away if triangle (projection) is outside map
					if (! polyIntersect(mapV, 4, triV, 3))
					{
						continue;
					}
					//get box (integer) parameters
					int ix[3], iz[3];
					map->getCoords(triV[0], ix[0], iz[0]);
					map->getCoords(triV[1], ix[1], iz[1]);
					map->getCoords(triV[2], ix[2], iz[2]);
					//project triangle over the map
					rasterizeTriangle(ix[0], iz[0], ix[1], iz[1], ix[2], iz[2],
							minX, maxX, &minZ, &maxZ);
				}
			}
			else if (dynamic_cast<RectangleObstacle*>(*iter))
			{
				//rectangle projection would cover an area on map,
				//unless perpendicular to it: use box instead
			}
			else if (dynamic_cast<PlaneObstacle*>(*iter))
			{
				//plane projection would cover the whole map plane,
				//unless perpendicular to it: don't use
			}
		}
		//free buffers
		delete[] minX;
		delete[] maxX;
	}

	///Check rectangle vs circle 2D intersection.
	///Any point or vertex are first projected over the y=0 plane, and resulting
	///projected vertices are supposed to form a rectangle defined ccw looking down y axis.
	///No check is done.
	///See http://www.geometrictools.com/Documentation/IntersectionRectangleEllipse.pdf.
	bool rectangleCircleIntersect(Vec3* rectVertices, Vec3 circleCenter,
			float circleRadius)
	{
		bool foundIntersection = false;
		//let's work in y=0 plane
		Vec3 rectV[4];
		for (int i = 0; i < 4; ++i)
		{
			rectV[i] = rectVertices[i].setYtoZero();
		}
		Vec3 circleC = circleCenter.setYtoZero();
		/*
			3-----------2      ----
		    |     u2    |     /    \
			|     |     |    /      \
			|    rC--u1 |   |   cC   |
			|           |    \      /
			0-----------1     \    /
		                       ----
		*/
		//get rectangle half dimensions
		float halfDim1 = (rectV[1] - rectV[0]).length() / 2.0;
		float halfDim2 = (rectV[3] - rectV[0]).length() / 2.0;
		assert((halfDim1 > 0) && (halfDim2 > 0));
		//get rectangle frame unit vectors (origin=rectangle center)
		Vec3 rectU1 = (rectV[1] - rectV[0]).normalize();
		Vec3 rectU2 = (rectV[2] - rectV[1]).normalize();
		//get rectangle center
		Vec3 rectC = rectV[0] + rectU1 * halfDim1 + rectU2 * halfDim2;
		//transform circle center wrt the rectangle frame
		Vec3 circleCR = Vec3(rectU1.dot(circleC - rectC), 0,
				rectU2.dot(circleC - rectC));
		//check if circle center is inside rectangle
		if ((abs(circleCR.x) <= halfDim1) && (abs(circleCR.z) <= halfDim2))
		{
			//intersection
			foundIntersection = true;
		}
		else
		{
			//transform rectangle's vertices wrt the rectangle frame
			Vec3 rectVR[4];
			for (int i = 0; i < 4; ++i)
			{
				rectVR[i] = Vec3(rectU1.dot(rectV[i] - rectC), 0,
						rectU2.dot(rectV[i] - rectC));
			}
			//check circle vs rectangle's edges intersection
			//according to which quadrant there is the circle center
			int edge1Idx[2], edge2Idx[2];
			//1st quadrant: check 2-1 and 3-2 edges
			if ((circleCR.x > 0) && (circleCR.z > 0))
			{
				edge1Idx[0] = 1;
				edge1Idx[1] = 2;
				edge2Idx[0] = 2;
				edge2Idx[1] = 3;
			}
			//2nd quadrant: check 3-2 and 0-3 edges
			else if ((circleCR.x < 0) && (circleCR.z > 0))
			{
				edge1Idx[0] = 2;
				edge1Idx[1] = 3;
				edge2Idx[0] = 3;
				edge2Idx[1] = 0;
			}
			//3rd quadrant: check 0-3 and 1-0 edges
			else if ((circleCR.x < 0) && (circleCR.z < 0))
			{
				edge1Idx[0] = 3;
				edge1Idx[1] = 0;
				edge2Idx[0] = 0;
				edge2Idx[1] = 1;
			}
			//4th quadrant: check 1-0 and 2-1 edges
			else if ((circleCR.x > 0) && (circleCR.z < 0))
			{
				edge1Idx[0] = 0;
				edge1Idx[1] = 1;
				edge2Idx[0] = 1;
				edge2Idx[1] = 2;
			}
			foundIntersection = edgeCircleIntersect(rectVR[edge1Idx[0]],
					rectVR[edge1Idx[1]], circleCR, circleRadius);
			if (! foundIntersection)
			{
				foundIntersection = edgeCircleIntersect(rectVR[edge2Idx[0]],
						rectVR[edge2Idx[1]], circleCR, circleRadius);
			}
		}
		//
		return foundIntersection;
	}

	bool edgeCircleIntersect(Vec3 edgeV0, Vec3 edgeV1, Vec3 center,
			float radius)
	{
		//by default no intersection
		bool foundIntersection = false;
		//get quadratic expression: q2*t^2 + q1*t + q0
		float radius2 = radius * radius;
		float q2 = (edgeV1 - edgeV0).lengthSquared() / radius2;
		float q1 = 2 * (edgeV1 - edgeV0).dot(edgeV0 - center) / radius2;
		float q0 = (edgeV0 - center).lengthSquared() / radius2 - 1.0;
		//compute discriminant
		float discriminant = q1 * q1 - 4 * q2 * q0;
		//check if there are solutions: discriminant >= 0
		if (discriminant >= 0.0)
		{
			//there are 1 or 2 solutions: check if:
			//- at least one is in [0, 1] or
			//- they are opposite in sign (meaning that one
			//	is >1 and the other is <0, i.e. edge is contained
			//	entirossup in circle)
			float t1 = (-q1 + sqrt(discriminant)) / (2 * q2);
			float t2 = (-q1 - sqrt(discriminant)) / (2 * q2);
			if ((0 <= t1 && t1 <= 1.0) || (0 <= t2 && t2 <= 1.0)
					|| (t1 * t2 < 0.0))
			{
				foundIntersection = true;
			}
		}
		//
		return foundIntersection;
	}

	///Check two convex polygons 2D intersection (by using separating axis algorithm).
	///Any point or vertex are first projected over the y=0 plane, and resulting
	///projected vertices are supposed to be defined ccw looking down y axis.
	///No check is done.
	///See "D.H. Eberly: 3D Game Engine Design, 2nd edition"
	bool polyIntersect(Vec3* poly1Vert, int vertN1, Vec3* poly2Vert, int vertN2)
	{
		//let's work in y=0 plane
		Vec3* poly1V = new Vec3[vertN1];
		Vec3* poly2V = new Vec3[vertN2];
		for (int i = 0; i < vertN1; ++i)
		{
			poly1V[i] = poly1Vert[i].setYtoZero();
		}
		for (int i = 0; i < vertN2; ++i)
		{
			poly2V[i] = poly2Vert[i].setYtoZero();
		}
		//
		bool foundIntersection = false;
		if (! seekSeparatingAxis(poly1V, vertN1, poly2V, vertN2))
		{
			foundIntersection = ! seekSeparatingAxis(poly2V, vertN2, poly1V,
					vertN1);
		}
		//free resources
		delete[] poly1V;
		delete[] poly2V;
		//
		return foundIntersection;
	}

	bool seekSeparatingAxis(Vec3* poly1V, int vertN1, Vec3* poly2V, int vertN2)
	{
		bool separatingAxisFound = false;
		//check poly1 normals vs poly2 vertices
		//for each poly1 external normal do
		for (int n = 0; n < vertN1; ++n)
		{
			int m = (n + 1) % vertN1;
			//get external normal as unit vector
			Vec3 edge = (poly1V[m] - poly1V[n]);
			Vec3 crossV;
			crossV.cross(edge, Vec3(0, 1, 0));
			Vec3 normal = crossV.normalize();
			//check every poly2 vertex component wrt normal
			for (int t = 0; t < vertN2; ++t)
			{
				//get the poly2 vertex component wrt normal and poly1 vertex:
				//component = (normal) dot (poly2Vert - poly1Vert)
				float component = (poly2V[t] - poly1V[n]).dot(normal);
				if (component < 0.0)
				{
					break;
				}
				if (t == vertN2 - 1)
				{
					//found a separating axis
					separatingAxisFound = true;
				}
			}
			if (separatingAxisFound)
			{
				break;
			}
		}
		return separatingAxisFound;
	}

	///projection (rasterization) functions
	///see https://www.cs.umd.edu/class/fall2003/cmsc427/bresenham.html
	void rasterizeCircle(int xc, int zc, int r, int* minX, int* maxX, int* minZ,
			int* maxZ)
	{
		//calculate boundaries by (not) drawing the circle
		circleBresenham(xc, zc, r, minX, maxX, minZ, maxZ);
		//draw row lines
		for (int row = *minZ; row < *maxZ + 1; ++row)
		{
			for (int col = minX[row]; col < maxX[row] + 1; ++col)
			{
				putPixel(col, row, true);
			}
			//reset X boundaries
			minX[row] = INT_MAX;
			maxX[row] = INT_MIN;
		}
		//reset Z boundaries
		*minZ = INT_MAX;
		*maxZ = INT_MIN;
	}

	void circleBresenham(int xc, int zc, int r, int* minX, int* maxX, int* minZ,
			int* maxZ)
	{
		int x = 0, z = r;
		int d = 3 - 2 * r;

		while (x < z)
		{
			setBoundaries(xc, zc, x, z, minX, maxX, minZ, maxZ);
			x++;
			if (d < 0)
				d = d + 4 * x + 6;
			else
			{
				z--;
				d = d + 4 * (x - z) + 10;
			}
			setBoundaries(xc, zc, x, z, minX, maxX, minZ, maxZ);
		}
	}

	void setBoundaries(int xc, int zc, int x, int z, int* minX, int* maxX,
			int* minZ, int* maxZ)
	{
		checkBoundary(xc + x, zc + z, minX, maxX, minZ, maxZ); //1
		checkBoundary(xc - x, zc + z, minX, maxX, minZ, maxZ); //2
		checkBoundary(xc + x, zc - z, minX, maxX, minZ, maxZ); //3
		checkBoundary(xc - x, zc - z, minX, maxX, minZ, maxZ); //4
		checkBoundary(xc + z, zc + x, minX, maxX, minZ, maxZ); //5
		checkBoundary(xc - z, zc + x, minX, maxX, minZ, maxZ); //6
		checkBoundary(xc + z, zc - x, minX, maxX, minZ, maxZ); //7
		checkBoundary(xc - z, zc - x, minX, maxX, minZ, maxZ); //8
	}

	void checkBoundary(int x, int z, int* minX, int* maxX, int* minZ, int* maxZ)
	{
		//check limits only if (x,z) inside map
		if (((z >= 0) && (z < map->resolution))
				&& ((x >= 0) && (x < map->resolution)))
		{
			//set minX and maxX
			minX[z] > x ? minX[z] = x : 0;
			maxX[z] < x ? maxX[z] = x : 0;
			//set minZ and maxZ
			*minZ > z ? *minZ = z : 0;
			*maxZ < z ? *maxZ = z : 0;
		}
	}

	void rasterizeTriangle(int x1, int z1, int x2, int z2, int x3, int z3,
			int* minX, int* maxX, int* minZ, int* maxZ)
	{
		//calculate boundaries by (not) drawing the edges
		lineBresenham(x1, x2, z1, z2, minX, maxX, minZ, maxZ);
		lineBresenham(x2, x3, z2, z3, minX, maxX, minZ, maxZ);
		lineBresenham(x3, x1, z3, z1, minX, maxX, minZ, maxZ);
		//draw row lines
		for (int row = *minZ; row < *maxZ + 1; ++row)
		{
			for (int col = minX[row]; col < maxX[row] + 1; ++col)
			{
				putPixel(col, row, true);
			}
			//reset X boundaries
			minX[row] = INT_MAX;
			maxX[row] = INT_MIN;
		}
		//reset Z boundaries
		*minZ = INT_MAX;
		*maxZ = INT_MIN;
	}

#define sign(n) ((n > 0)? 1 : ((n < 0)? -1: 0))

	void lineBresenham(int x1, int x2, int z1, int z2, int* minX, int* maxX,
			int* minZ, int* maxZ)
	{
		int x = x1;
		int z = z1;
		int dx = abs(x2 - x1);
		int dz = abs(z2 - z1);
		int s1 = sign(x2 - x1);
		int s2 = sign(z2 - z1);
		int swap = 0;
		if (dz > dx)
		{
			int temp = dx;
			dx = dz;
			dz = temp;
			swap = 1;
		}
		int D = 2 * dz - dx;
		for (int i = 0; i < dx; i++)
		{
			//check limits only if (x,z) inside map
			if (((z >= 0) && (z < map->resolution))
					&& ((x >= 0) && (x < map->resolution)))
			{
				//set minX and maxX
				minX[z] > x ? minX[z] = x : 0;
				maxX[z] < x ? maxX[z] = x : 0;
				//set minZ and maxZ
				*minZ > z ? *minZ = z : 0;
				*maxZ < z ? *maxZ = z : 0;
			}
			//
			while (D >= 0)
			{
				D = D - 2 * dx;
				if (swap)
					x += s1;
				else
					z += s2;
			}
			D = D + 2 * dz;
			if (swap)
				z += s2;
			else
				x += s1;
		}
	}

	void putPixel(int i, int j, bool value)
	{
#ifdef OLDTERRAINMAP
		map->setMapBit(i, j, value);
#else
		map->setType (i, j, CellData::OBSTACLE);

#endif
	}

	void drawBoundaryFencesOnMap(TerrainMap& map)
	{
		// QQQ it would make more sense to do this with a "draw line
		// QQQ on map" primitive, may need that for other things too
		const int cw = map.cellwidth();
		const int ch = map.cellheight();
		const int r = cw - 1;
		const int a = cw >> 3;
		const int b = cw - a;
		const int o = cw >> 4;
		const int p = (cw - o) >> 1;
		const int q = (cw + o) >> 1;
		for (int i = 0; i < cw; i++)
		{
			for (int j = 0; j < ch; j++)
			{
				const bool c = i > a && i < b && (i < p || i > q);
				if (i == 0 || j == 0 || i == r || j == r
						|| (c && (i == j || i + j == r)))
#ifdef OLDTERRAINMAP
					map.setMapBit(i, j, true);
#else
				map.setType (i, j, CellData::IMPASSABLE);
#endif
			}
		}
	}

	void clearCenterOfMap()
	{
		//if map not made return
		if (map == NULL)
		{
			return;
		}
		const int o = map->cellwidth() >> 4;
		const int p = (map->cellwidth() - o) >> 1;
		const int q = (map->cellwidth() + o) >> 1;
		for (int i = p; i <= q; i++)
			for (int j = p; j <= q; j++)
#ifdef OLDTERRAINMAP
				map->setMapBit(i, j, false);
#else
		map->setType (i, j, CellData::CLEAR);
#endif
	}

	void drawPathFencesOnMap(TerrainMap& map, GCRoute& _path)
	{
#ifdef OLDTERRAINMAP
		const float xs = map.xSize / (float) map.resolution;
		const float zs = map.zSize / (float) map.resolution;
		const Vec3 alongRow(xs, 0, 0);
		const Vec3 nextRow(-map.xSize, 0, zs);
		Vec3 g((map.xSize - xs) / -2, 0, (map.zSize - zs) / -2);
		g += map.center;
		//make a flat path at height map.center.y
		Vec3* newPoints = new Vec3[_path.pointCount()];
		for (SegmentedPathway::size_type i = 0; i < _path.pointCount(); ++i)
		{
			newPoints[i] = _path.point(i);
			newPoints[i].y = map.center.y;
		}
		GCRoute path = _path;
		path.movePoints(0, path.pointCount() - (path.isCyclic() ? 1 : 0),
				newPoints);
		delete[] newPoints;
		for (int j = 0; j < map.resolution; j++)
		{
			for (int i = 0; i < map.resolution; i++)
			{
				const float outside = mapPointToOutside(path, g); // path.howFarOutsidePath (g);
				const float wallThickness = 1.0f;
				// set map cells adjacent to the outside edge of the path
				if ((outside > 0) && (outside < wallThickness))
					map.setMapBit(i, j, true);
				// clear all other off-path map cells
				if (outside > wallThickness)
					map.setMapBit(i, j, false);
				g += alongRow;
			}
			g += nextRow;
		}
#else
#endif
	}

	const AVGroup& allVehicles(void)
	{
		return (const AVGroup&) vehicles;
	}

	//create the map based on the the defined pathway and obstacles:
	//if any of them change you must recall this function to update the map
	void makeMap(int resolution)
	{
		//if already made return
		if (map != NULL)
		{
			return;
		}
		//
		typedef OpenSteer::SegmentedPathway::size_type size_type;
		GCRoute* pathway = dynamic_cast<GCRoute*>(m_pathway[0]);
		if (!pathway)
		{
			cerr
					<< "ERROR: MapDrivePlugIn::makeMap():\n"
					"pathway must be a 'PolylineSegmentedPathwaySegmentRadii'"
					<< endl;
			return;
		}
		if (pathway->pointCount() < 2)
		{
			return;
		}
		//take the pathway's center and min/max dimensions
		float minMaxX[2] =
		{	FLT_MAX, -FLT_MAX}; //min,max
		float minMaxY[2] =
		{	FLT_MAX, -FLT_MAX}; //min,max
		float minMaxZ[2] =
		{	FLT_MAX, -FLT_MAX}; //min,max
		Vec3 center = Vec3::zero;
		for (size_type i = 0; i < pathway->pointCount(); ++i)
		{
			Vec3 point = pathway->point(i);
			//x
			if (point.x < minMaxX[0])
			{
				minMaxX[0] = point.x;
			}
			if (point.x > minMaxX[1])
			{
				minMaxX[1] = point.x;
			}
			//y
			if (point.y < minMaxY[0])
			{
				minMaxY[0] = point.y;
			}
			if (point.y > minMaxY[1])
			{
				minMaxY[1] = point.y;
			}
			//z
			if (point.z < minMaxY[0])
			{
				minMaxY[0] = point.z;
			}
			if (point.z > minMaxY[1])
			{
				minMaxY[1] = point.z;
			}
			center = center + point;
		}
		center = center / (float) pathway->pointCount();
		//take the max radius of the pathway's segment(s)
		float maxRadius = 0.0;
		OpenSteer::PolylineSegmentedPathwaySegmentRadii* pathwayRadii =
				dynamic_cast<OpenSteer::PolylineSegmentedPathwaySegmentRadii*>(pathway);
		OpenSteer::PolylineSegmentedPathwaySingleRadius* pathwaySingle =
				dynamic_cast<OpenSteer::PolylineSegmentedPathwaySingleRadius*>(pathway);
		if (pathwayRadii)
		{
			for (size_type i = 0; i < pathwayRadii->segmentCount(); ++i)
			{
				if (pathwayRadii->segmentRadius(i) > maxRadius)
				{
					maxRadius = pathwayRadii->segmentRadius(i);
				}
			}
		}
		else if (pathwaySingle)
		{
			pathwaySingle->radius();
		}
		else
		{
			return;
		}
		//take max spread of the pathway's points
		float dX = minMaxX[1] - minMaxX[0];
		float dY = minMaxY[1] - minMaxY[0];
		float dZ = minMaxZ[1] - minMaxZ[0];
		//
		float tolerance = 5.0;
		worldSize = max(max(dX, dY), dZ) + maxRadius * tolerance;
		worldDiag = sqrtXXX(square(worldSize) / 2);
		worldCenter = center;
		worldResolution = resolution;
#ifdef OLDTERRAINMAP
		map = new TerrainMap(worldCenter, worldSize, worldSize, resolution);
#else
		map = new TerrainMap (worldSize, worldSize, 1);
#endif

		regenerateMap();
	}

	// map of obstacles
	TerrainMap* map;
	// size of the world (the map actually)
	float worldSize;
	float worldDiag;
	Vec3 worldCenter;
	int worldResolution;

	// which of the three demo modes is selected
	int demoSelect;
	bool curvedSteering;

#ifdef OS_DEBUG
	float windowWidth;
#endif

///	MapDriver* vehicle;
	typename MapDriver<Entity>::groupType vehicles; // for allVehicles
	typedef typename MapDriver<Entity>::groupType::const_iterator iterator;

	bool usePathFences;
///	bool useRandomRocks;
};

//MapDrivePlugIn gMapDrivePlugIn;

// ----------------------------------------------------------------------------

} // ossup namespace

#endif /* PLUGIN_MAPDRIVE_H_ */
