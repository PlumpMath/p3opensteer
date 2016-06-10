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
// Draw
//
// This is a first stab at a graphics module for OpenSteerDemo.  It is intended
// to encapsulate all functionality related to 3d graphics as well as windows
// and graphics input devices such as the mouse.
//
// However this is purossup an OpenGL-based implementation.  No special effort
// has been made to keep the "OpenGL way" from leaking through.  Attempting to
// port this to another graphics substrate may run into modularity problems.
//
// In any case, all calls to the underlying graphics substrate should be made
// from this module only.
//
// 10-04-04 bk:  put everything into the OpenSteer namespace
// 06-25-02 cwr: created
//
//
// ----------------------------------------------------------------------------
/**
 * \file Draw.cpp
 *
 * \date 2016-05-13
 * \author consultit
 */

#include <iomanip>
#include <sstream>
#include <OpenSteer/Draw.h>
#include <OpenSteer/Vec3.h>
// To include OpenSteer::round.
#include <OpenSteer/Utilities.h>

#include "Support/OpenSteerLocal/common.h"

#include "Support/OpenSteerLocal/DrawMeshDrawer.h"
#include <reMutex.h>
ossup::DrawMeshDrawer *gDrawer3d, *gDrawer2d;
ReMutex gOpenSteerDebugMutex;

// ----------------------------------------------------------------------------

// Graphics interface
// Collected the available abstractions here as a first step
// to swapping in different graphics libraries

namespace
{
// ------------------------------------------------------------------------
// emit an vertex based on a Vec3

inline void iglVertexVec3(const OpenSteer::Vec3& v)
{
	gDrawer3d->vertex(ossup::OpenSteerVec3ToLVecBase3f(v));
}

// ----------------------------------------------------------------------------
// draw 3d "graphical annotation" lines, used for debugging

inline void iDrawLine(const OpenSteer::Vec3& startPoint,
		const OpenSteer::Vec3& endPoint, const OpenSteer::Color& color)
{
	gDrawer3d->setColor(ossup::OpenSteerColorToLVecBase4f(color));
	gDrawer3d->begin(ossup::DrawMeshDrawer::DRAW_LINES);
	glVertexVec3(startPoint);
	glVertexVec3(endPoint);
	gDrawer3d->end();
}

// ----------------------------------------------------------------------------
// Draw a single triangle given three Vec3 vertices.

inline void iDrawTriangle(const OpenSteer::Vec3& a, const OpenSteer::Vec3& b,
		const OpenSteer::Vec3& c, const OpenSteer::Color& color)
{
	gDrawer3d->setColor(ossup::OpenSteerColorToLVecBase4f(color));
	gDrawer3d->begin(ossup::DrawMeshDrawer::DRAW_TRIS);
	{
		OpenSteer::glVertexVec3(a);
		OpenSteer::glVertexVec3(b);
		OpenSteer::glVertexVec3(c);
	}
	gDrawer3d->end();
}

// ------------------------------------------------------------------------
// Draw a single quadrangle given four Vec3 vertices, and color.

inline void iDrawQuadrangle(const OpenSteer::Vec3& a, const OpenSteer::Vec3& b,
		const OpenSteer::Vec3& c, const OpenSteer::Vec3& d,
		const OpenSteer::Color& color)
{
	gDrawer3d->setColor(ossup::OpenSteerColorToLVecBase4f(color));
	gDrawer3d->begin(ossup::DrawMeshDrawer::DRAW_QUADS);
	{
		OpenSteer::glVertexVec3(a);
		OpenSteer::glVertexVec3(b);
		OpenSteer::glVertexVec3(c);
		OpenSteer::glVertexVec3(d);
	}
	gDrawer3d->end();
}

// ------------------------------------------------------------------------
// Between matched sets of these two calls, assert that all polygons
// will be drawn "double sided", that is, without back-face culling

inline void beginDoubleSidedDrawing(void)
{
	gDrawer3d->setTwoSided(true);
}

inline void endDoubleSidedDrawing(void)
{
	gDrawer3d->setTwoSided(false);
}

}   // end anonymous namespace

namespace OpenSteer
{

void glVertexVec3(const Vec3& v)
{
	iglVertexVec3(v);
}

// ----------------------------------------------------------------------------

void drawLine(const Vec3& startPoint, const Vec3& endPoint,
		const Color& color)
{
	iDrawLine(startPoint, endPoint, color);
}

// ----------------------------------------------------------------------------
// draw a line with alpha blending

void drawLineAlpha(const Vec3& startPoint, const Vec3& endPoint,
		const Color& color, const float alpha)
{
	gDrawer3d->setColor(LVecBase4f(color.r(), color.g(), color.b(), alpha));
	gDrawer3d->begin(ossup::DrawMeshDrawer::DRAW_LINES);
	glVertexVec3(startPoint);
	glVertexVec3(endPoint);
	gDrawer3d->end();
}

void drawTriangle(const Vec3& a, const Vec3& b, const Vec3& c,
		const Color& color)
{
	iDrawTriangle(a, b, c, color);
}

void drawQuadrangle(const Vec3& a, const Vec3& b, const Vec3& c,
		const Vec3& d, const Color& color)
{
	iDrawQuadrangle(a, b, c, d, color);
}

// ------------------------------------------------------------------------
// draws a "wide line segment": a rectangle of the given width and color
// whose mid-line connects two given endpoints

void drawXZWideLine(const Vec3& startPoint, const Vec3& endPoint,
		const Color& color, float width)
{
	const Vec3 offset = endPoint - startPoint;
	const Vec3 along = offset.normalize();
	const Vec3 perp = gGlobalSpace.localRotateForwardToSide(along);
	const Vec3 radius = perp * width / 2;

	const Vec3 a = startPoint + radius;
	const Vec3 b = endPoint + radius;
	const Vec3 c = endPoint - radius;
	const Vec3 d = startPoint - radius;

	iDrawQuadrangle(a, b, c, d, color);
}

// ------------------------------------------------------------------------
// General purpose circle/disk drawing routine.  Draws circles or disks (as
// specified by "filled" argument) and handles both special case 2d circles
// on the XZ plane or arbitrary circles in 3d space (as specified by "in3d"
// argument)

void drawCircleOrDisk(const float radius, const Vec3& axis,
		const Vec3& center, const Color& color, const int segments,
		const bool filled, const bool in3d)
{
	LocalSpace ls;
	if (in3d)
	{
		// define a local space with "axis" as the Y/up direction
		// (XXX should this be a method on  LocalSpace?)
		const Vec3 unitAxis = axis.normalize();
		const Vec3 unitPerp = findPerpendicularIn3d(axis).normalize();
		ls.setUp(unitAxis);
		ls.setForward(unitPerp);
		ls.setPosition(center);
		ls.setUnitSideFromForwardAndUp();
	}

	// make disks visible (not culled) from both sides
	if (filled)
		beginDoubleSidedDrawing();

	// point to be rotated about the (local) Y axis, angular step size
	Vec3 pointOnCircle(radius, 0, 0);
	const float step = (2 * OPENSTEER_M_PI) / segments;

	// set drawing color
	gDrawer3d->setColor(ossup::OpenSteerColorToLVecBase4f(color));

	// begin drawing a triangle fan (for disk) or line loop (for circle)
	gDrawer3d->begin(
			filled ?
					ossup::DrawMeshDrawer::DRAW_TRIFAN :
					ossup::DrawMeshDrawer::DRAW_LINELOOP);

	// for the filled case, first emit the center point
	if (filled)
		iglVertexVec3(in3d ? ls.position() : center);

	// rotate p around the circle in "segments" steps
	float sin = 0, cos = 0;
	const int vertexCount = filled ? segments + 1 : segments;
	for (int i = 0; i < vertexCount; i++)
	{
		// emit next point on circle, either in 3d (globalized out
		// of the local space), or in 2d (offset from the center)
		iglVertexVec3(
				in3d ? ls.globalizePosition(pointOnCircle) : (Vec3) (pointOnCircle
								+ center));

		// rotate point one more step around circle
		pointOnCircle = pointOnCircle.rotateAboutGlobalY(step, sin, cos);
	}

	// close drawing operation
	gDrawer3d->end();
	if (filled)
		endDoubleSidedDrawing();
}

// ------------------------------------------------------------------------

void draw3dCircleOrDisk(const float radius, const Vec3& center,
		const Vec3& axis, const Color& color, const int segments,
		const bool filled)
{
	// draw a circle-or-disk in the given local space
	drawCircleOrDisk(radius, axis, center, color, segments, filled, true);
}

// ------------------------------------------------------------------------
// drawing utility used by both drawXZCircle and drawXZDisk

void drawXZCircleOrDisk(const float radius, const Vec3& center,
		const Color& color, const int segments, const bool filled)
{
	// draw a circle-or-disk on the XZ plane
	drawCircleOrDisk(radius, Vec3::zero, center, color, segments, filled,
			false);
}

// ------------------------------------------------------------------------
// draw a circular arc on the XZ plane, from a start point, around a center,
// for a given arc length, in a given number of segments and color.  The
// sign of arcLength determines the direction in which the arc is drawn.
//
// XXX maybe this should alow allow "in3d" cricles about an given axis
// XXX maybe there should be a "filled" version of this
// XXX maybe this should be merged in with drawCircleOrDisk

void drawXZArc(const Vec3& start, const Vec3& center,
		const float arcLength, const int segments, const Color& color)
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

	// set drawing color
	gDrawer3d->setColor(ossup::OpenSteerColorToLVecBase4f(color));

	// begin drawing a series of connected line segments
	gDrawer3d->begin(ossup::DrawMeshDrawer::DRAW_LINESTRIP);

	// draw each segment along arc
	float sin = 0, cos = 0;
	for (int i = 0; i < segments; i++)
	{
		// emit next point on arc
		iglVertexVec3(spoke + center);

		// rotate point to next step around circle
		spoke = spoke.rotateAboutGlobalY(step, sin, cos);
	}

	// close drawing operation
	gDrawer3d->end();
}

// ------------------------------------------------------------------------
// a simple 2d vehicle on the XZ plane

void drawBasic2dCircularVehicle(const AbstractVehicle& vehicle,
		const Color& color)
{
	// "aspect ratio" of body (as seen from above)
	const float x = 0.5f;
	const float y = sqrtXXX(1 - (x * x));

	// radius and position of vehicle
	const float r = vehicle.radius();
	const Vec3& p = vehicle.position();

	// shape of triangular body
	const Vec3 u = r * 0.05f * Vec3(0, 1, 0); // slightly up
	const Vec3 f = r * vehicle.forward();
	const Vec3 s = r * vehicle.side() * x;
	const Vec3 b = r * vehicle.forward() * -y;

	// draw double-sided triangle (that is: no (back) face culling)
	beginDoubleSidedDrawing();
	iDrawTriangle(p + f + u, p + b - s + u, p + b + s + u, color);
	endDoubleSidedDrawing();

	// draw the circular collision boundary
	drawXZCircle(r, p + u, gWhite, 20);
}

// ------------------------------------------------------------------------
// a simple 3d vehicle

void drawBasic3dSphericalVehicle(const AbstractVehicle& vehicle,
		const Color& color)
{
	// "aspect ratio" of body (as seen from above)
	const float x = 0.5f;
	const float y = sqrtXXX(1 - (x * x));

	// radius and position of vehicle
	const float r = vehicle.radius();
	const Vec3& p = vehicle.position();

	// body shape parameters
	const Vec3 f = r * vehicle.forward();
	const Vec3 s = r * vehicle.side() * x;
	const Vec3 u = r * vehicle.up() * x * 0.5f;
	const Vec3 b = r * vehicle.forward() * -y;

	// vertex positions
	const Vec3 nose = p + f;
	const Vec3 side1 = p + b - s;
	const Vec3 side2 = p + b + s;
	const Vec3 top = p + b + u;
	const Vec3 bottom = p + b - u;

	// colors
	const float j = +0.05f;
	const float k = -0.05f;
	const Color color1 = color + Color(j, j, k);
	const Color color2 = color + Color(j, k, j);
	const Color color3 = color + Color(k, j, j);
	const Color color4 = color + Color(k, j, k);
	const Color color5 = color + Color(k, k, j);

	// draw body
	iDrawTriangle(nose, side1, top, color1);  // top, side 1
	iDrawTriangle(nose, top, side2, color2);  // top, side 2
	iDrawTriangle(nose, bottom, side1, color3);  // bottom, side 1
	iDrawTriangle(nose, side2, bottom, color4);  // bottom, side 2
	iDrawTriangle(side1, side2, top, color5);  // top back
	iDrawTriangle(side2, side1, bottom, color5);  // bottom back
}

// drawBasic3dSphericalVehicle with a supplied draw routine
// provided so non-OpenGL based apps can draw a boid

void drawBasic3dSphericalVehicle(drawTriangleRoutine draw,
		const AbstractVehicle& vehicle, const Color& color)
{
	// "aspect ratio" of body (as seen from above)
	const float x = 0.5f;
	const float y = sqrtXXX(1 - (x * x));

	// radius and position of vehicle
	const float r = vehicle.radius();
	const Vec3& p = vehicle.position();

	// body shape parameters
	const Vec3 f = r * vehicle.forward();
	const Vec3 s = r * vehicle.side() * x;
	const Vec3 u = r * vehicle.up() * x * 0.5f;
	const Vec3 b = r * vehicle.forward() * -y;

	// vertex positions
	const Vec3 nose = p + f;
	const Vec3 side1 = p + b - s;
	const Vec3 side2 = p + b + s;
	const Vec3 top = p + b + u;
	const Vec3 bottom = p + b - u;

	// colors
	const float j = +0.05f;
	const float k = -0.05f;
	const Color color1 = color + Color(j, j, k);
	const Color color2 = color + Color(j, k, j);
	const Color color3 = color + Color(k, j, j);
	const Color color4 = color + Color(k, j, k);
	const Color color5 = color + Color(k, k, j);

	// draw body
	draw(nose, side1, top, color1);  // top, side 1
	draw(nose, top, side2, color2);  // top, side 2
	draw(nose, bottom, side1, color3);  // bottom, side 1
	draw(nose, side2, bottom, color4);  // bottom, side 2
	draw(side1, side2, top, color5);  // top back
	draw(side2, side1, bottom, color5);  // bottom back
}

// ------------------------------------------------------------------------
// draw a square grid of lines on the XZ (horizontal) plane.
//
// ("size" is the length of a side of the overall grid, "subsquares" is the
// number of subsquares along each edge (for example a standard checkboard
// has eight), "center" is the 3d position of the center of the grid, lines
// are drawn in the specified "color".)

void drawXZLineGrid(const float size, const int subsquares,
		const Vec3& center, const Color& color)
{
	const float half = size / 2;
	const float spacing = size / subsquares;

	// set grid drawing color
	gDrawer3d->setColor(ossup::OpenSteerColorToLVecBase4f(color));

	// draw a square XZ grid with the given size and line count
	gDrawer3d->begin(ossup::DrawMeshDrawer::DRAW_LINES);
	float q = -half;
	for (int i = 0; i < (subsquares + 1); i++)
	{
		const Vec3 x1(q, 0, +half); // along X parallel to Z
		const Vec3 x2(q, 0, -half);
		const Vec3 z1(+half, 0, q); // along Z parallel to X
		const Vec3 z2(-half, 0, q);

		iglVertexVec3(x1 + center);
		iglVertexVec3(x2 + center);
		iglVertexVec3(z1 + center);
		iglVertexVec3(z2 + center);

		q += spacing;
	}
	gDrawer3d->end();
}

// ------------------------------------------------------------------------
// draw the three axes of a LocalSpace: three lines parallel to the
// basis vectors of the space, centered at its origin, of lengths
// given by the coordinates of "size".

void drawAxes(const AbstractLocalSpace& ls, const Vec3& size,
		const Color& color)
{
	const Vec3 x(size.x / 2, 0, 0);
	const Vec3 y(0, size.y / 2, 0);
	const Vec3 z(0, 0, size.z / 2);

	iDrawLine(ls.globalizePosition(x), ls.globalizePosition(x * -1), color);
	iDrawLine(ls.globalizePosition(y), ls.globalizePosition(y * -1), color);
	iDrawLine(ls.globalizePosition(z), ls.globalizePosition(z * -1), color);
}

// ------------------------------------------------------------------------
// draw the edges of a box with a given position, orientation, size
// and color.  The box edges are aligned with the axes of the given
// LocalSpace, and it is centered at the origin of that LocalSpace.
// "size" is the main diagonal of the box.
//
// use gGlobalSpace to draw a box aligned with global space

void drawBoxOutline(const AbstractLocalSpace& localSpace,
		const Vec3& size, const Color& color)
{
	const Vec3 s = size / 2.0f;  // half of main diagonal

	const Vec3 a(+s.x, +s.y, +s.z);
	const Vec3 b(+s.x, -s.y, +s.z);
	const Vec3 c(-s.x, -s.y, +s.z);
	const Vec3 d(-s.x, +s.y, +s.z);

	const Vec3 e(+s.x, +s.y, -s.z);
	const Vec3 f(+s.x, -s.y, -s.z);
	const Vec3 g(-s.x, -s.y, -s.z);
	const Vec3 h(-s.x, +s.y, -s.z);

	const Vec3 A = localSpace.globalizePosition(a);
	const Vec3 B = localSpace.globalizePosition(b);
	const Vec3 C = localSpace.globalizePosition(c);
	const Vec3 D = localSpace.globalizePosition(d);

	const Vec3 E = localSpace.globalizePosition(e);
	const Vec3 F = localSpace.globalizePosition(f);
	const Vec3 G = localSpace.globalizePosition(g);
	const Vec3 H = localSpace.globalizePosition(h);

	iDrawLine(A, B, color);
	iDrawLine(B, C, color);
	iDrawLine(C, D, color);
	iDrawLine(D, A, color);

	iDrawLine(A, E, color);
	iDrawLine(B, F, color);
	iDrawLine(C, G, color);
	iDrawLine(D, H, color);

	iDrawLine(E, F, color);
	iDrawLine(F, G, color);
	iDrawLine(G, H, color);
	iDrawLine(H, E, color);
}

void draw2dLine(const Vec3& startPoint, const Vec3& endPoint,
		const Color& color, float w, float h)
{
	gDrawer2d->setColor(ossup::OpenSteerColorToLVecBase4f(color));
	gDrawer2d->begin(ossup::DrawMeshDrawer::DRAW_LINES);
	glVertexVec3(startPoint);
	glVertexVec3(endPoint);
	gDrawer2d->end();
}

// ------------------------------------------------------------------------
// draw a reticle at the center of the window.  Currently it is small
// crosshair with a gap at the center, drawn in white with black borders

void drawReticle(float w, float h)
{
	const int a = 10;
	const int b = 30;

	draw2dLine(Vec3(w + a, h, 0), Vec3(w + b, h, 0), gWhite, w, h);
	draw2dLine(Vec3(w, h + a, 0), Vec3(w, h + b, 0), gWhite, w, h);
	draw2dLine(Vec3(w - a, h, 0), Vec3(w - b, h, 0), gWhite, w, h);
	draw2dLine(Vec3(w, h - a, 0), Vec3(w, h - b, 0), gWhite, w, h);

	gDrawer3d->setSize(3);
	draw2dLine(Vec3(w + a, h, 0), Vec3(w + b, h, 0), gBlack, w, h);
	draw2dLine(Vec3(w, h + a, 0), Vec3(w, h + b, 0), gBlack, w, h);
	draw2dLine(Vec3(w - a, h, 0), Vec3(w - b, h, 0), gBlack, w, h);
	draw2dLine(Vec3(w, h - a, 0), Vec3(w, h - b, 0), gBlack, w, h);
	gDrawer3d->setSize(1);
}

}

namespace
{

// ----------------------------------------------------------------------------
// deferred draw line
//
// For use during simulation phase.
// Stores description of lines to be drawn later.

class DeferredLine
{
public:

	static void addToBuffer(const OpenSteer::Vec3& s, const OpenSteer::Vec3& e,
			const OpenSteer::Color& c)
	{
		DeferredLine dl;
		dl.startPoint = s;
		dl.endPoint = e;
		dl.color = c;

		lines.push_back(dl);
	}

	static void drawAll(void)
	{
		// draw all deferred lines
		for (DeferredLines::iterator i = lines.begin(); i < lines.end(); i++)
		{
			DeferredLine& dl = *i;
			iDrawLine(dl.startPoint, dl.endPoint, dl.color);
		}

		// clear list of deferred lines
		lines.clear();
	}

	typedef std::vector<DeferredLine> DeferredLines;

private:

	OpenSteer::Vec3 startPoint;
	OpenSteer::Vec3 endPoint;
	OpenSteer::Color color;

	static DeferredLines lines;
};

DeferredLine::DeferredLines DeferredLine::lines;

} // anonymous namespace

namespace OpenSteer
{
void deferredDrawLine(const Vec3& startPoint, const Vec3& endPoint,
		const Color& color)
{
	DeferredLine::addToBuffer(startPoint, endPoint, color);
}

void drawAllDeferredLines(void)
{
	DeferredLine::drawAll();
}

}

namespace
{

// ----------------------------------------------------------------------------
// deferred draw circle
// XXX for now, just a modified copy of DeferredLine
//
// For use during simulation phase.
// Stores description of circles to be drawn later.

class DeferredCircle
{
public:

	static void addToBuffer(const float radius, const OpenSteer::Vec3& axis,
			const OpenSteer::Vec3& center, const OpenSteer::Color& color,
			const int segments, const bool filled, const bool in3d)
	{
		DeferredCircle dc;
		dc.radius = radius;
		dc.axis = axis;
		dc.center = center;
		dc.color = color;
		dc.segments = segments;
		dc.filled = filled;
		dc.in3d = in3d;
		circles.push_back(dc);
	}

	static void drawAll(void)
	{
		// draw all deferred circles
		for (DeferredCircles::iterator i = circles.begin(); i < circles.end();
				i++)
		{
			DeferredCircle& dc = *i;
			drawCircleOrDisk(dc.radius, dc.axis, dc.center, dc.color,
					dc.segments, dc.filled, dc.in3d);
		}

		// clear list of deferred circles
		circles.clear();
	}

	typedef std::vector<DeferredCircle> DeferredCircles;

private:

	float radius;
	OpenSteer::Vec3 axis;
	OpenSteer::Vec3 center;
	OpenSteer::Color color;
	int segments;
	bool filled;
	bool in3d;

	static DeferredCircles circles;
};

DeferredCircle::DeferredCircles DeferredCircle::circles;

} // anonymous namesopace

namespace OpenSteer
{

void deferredDrawCircleOrDisk(const float radius, const Vec3& axis,
		const Vec3& center, const Color& color, const int segments,
		const bool filled, const bool in3d)
{
	DeferredCircle::addToBuffer(radius, axis, center, color, segments, filled,
			in3d);
}

void drawAllDeferredCirclesOrDisks(void)
{
	DeferredCircle::drawAll();
}

void draw2dTextAt3dLocation(const char& text, const Vec3& location,
		const Color& color, float w, float h)
{
	gDrawer3d->drawText(std::string(&text),
			ossup::OpenSteerVec3ToLVecBase3f(location),
			ossup::OpenSteerColorToLVecBase4f(color));
}

void draw2dTextAt3dLocation(const std::ostringstream& text,
		const Vec3& location, const Color& color, float w, float h)
{
	draw2dTextAt3dLocation(*text.str().c_str(), location, color, w, h);
}

void draw2dTextAt2dLocation(const char& text, const Vec3 location,
		const Color& color, float w, float h)
{
	gDrawer2d->drawText(std::string(&text),
			ossup::OpenSteerVec3ToLVecBase3f(location),
			ossup::OpenSteerColorToLVecBase4f(color));
}

void draw2dTextAt2dLocation(const std::ostringstream& text,
		const Vec3 location, const Color& color, float w, float h)
{
	draw2dTextAt2dLocation(*text.str().c_str(), location, color, w, h);
}

// ----------------------------------------------------------------------------

// This class provides methods to draw spheres.  The shape is represented
// as a "geodesic" mesh of triangles generated by subviding an icosahedron
// until an edge length criteria is met.  Supports wireframe and unshaded
// triangle drawing styles.  Provides front/back/both culling of faces.
//
// see drawSphere below
//
class DrawSphereHelper
{
public:
	Vec3 center;
	float radius;
	float maxEdgeLength;
	bool filled;
	Color color;
	bool drawFrontFacing;
	bool drawBackFacing;
	Vec3 viewpoint;

	// default constructor (at origin, radius=1, white, wireframe, nocull)
	DrawSphereHelper() :
			center(Vec3::zero), radius(1.0f), maxEdgeLength(1.0f), filled(
					false), color(gWhite), drawFrontFacing(true), drawBackFacing(
					true), viewpoint(Vec3::zero)
	{
	}

	// "kitchen sink" constructor (allows specifying everything)
	DrawSphereHelper(const Vec3 _center, const float _radius,
			const float _maxEdgeLength, const bool _filled, const Color& _color,
			const bool _drawFrontFacing, const bool _drawBackFacing,
			const Vec3& _viewpoint) :
			center(_center), radius(_radius), maxEdgeLength(_maxEdgeLength), filled(
					_filled), color(_color), drawFrontFacing(_drawFrontFacing), drawBackFacing(
					_drawBackFacing), viewpoint(_viewpoint)
	{
	}

	// draw as an icosahedral geodesic sphere
	void draw(void) const
	{
		// Geometry based on Paul Bourke's excellent article:
		//   Platonic Solids (Regular polytopes in 3D)
		//   http://astronomy.swin.edu.au/~pbourke/polyhedra/platonic/
		const float sqrt5 = sqrtXXX(5.0f);
		const float phi = (1.0f + sqrt5) * 0.5f; // "golden ratio"
		// ratio of edge length to radius
		const float ratio = sqrtXXX(10.0f + (2.0f * sqrt5)) / (4.0f * phi);
		const float a = (radius / ratio) * 0.5;
		const float b = (radius / ratio) / (2.0f * phi);

		// define the icosahedron's 12 vertices:
		const Vec3 v1 = center + Vec3(0, b, -a);
		const Vec3 v2 = center + Vec3(b, a, 0);
		const Vec3 v3 = center + Vec3(-b, a, 0);
		const Vec3 v4 = center + Vec3(0, b, a);
		const Vec3 v5 = center + Vec3(0, -b, a);
		const Vec3 v6 = center + Vec3(-a, 0, b);
		const Vec3 v7 = center + Vec3(0, -b, -a);
		const Vec3 v8 = center + Vec3(a, 0, -b);
		const Vec3 v9 = center + Vec3(a, 0, b);
		const Vec3 v10 = center + Vec3(-a, 0, -b);
		const Vec3 v11 = center + Vec3(b, -a, 0);
		const Vec3 v12 = center + Vec3(-b, -a, 0);

		// draw the icosahedron's 20 triangular faces:
		drawMeshedTriangleOnSphere(v1, v2, v3);
		drawMeshedTriangleOnSphere(v4, v3, v2);
		drawMeshedTriangleOnSphere(v4, v5, v6);
		drawMeshedTriangleOnSphere(v4, v9, v5);
		drawMeshedTriangleOnSphere(v1, v7, v8);
		drawMeshedTriangleOnSphere(v1, v10, v7);
		drawMeshedTriangleOnSphere(v5, v11, v12);
		drawMeshedTriangleOnSphere(v7, v12, v11);
		drawMeshedTriangleOnSphere(v3, v6, v10);
		drawMeshedTriangleOnSphere(v12, v10, v6);
		drawMeshedTriangleOnSphere(v2, v8, v9);
		drawMeshedTriangleOnSphere(v11, v9, v8);
		drawMeshedTriangleOnSphere(v4, v6, v3);
		drawMeshedTriangleOnSphere(v4, v2, v9);
		drawMeshedTriangleOnSphere(v1, v3, v10);
		drawMeshedTriangleOnSphere(v1, v8, v2);
		drawMeshedTriangleOnSphere(v7, v10, v12);
		drawMeshedTriangleOnSphere(v7, v11, v8);
		drawMeshedTriangleOnSphere(v5, v12, v6);
		drawMeshedTriangleOnSphere(v5, v9, v11);
	}

	// given two points, take midpoint and project onto this sphere
	inline Vec3 midpointOnSphere(const Vec3& a, const Vec3& b) const
	{
		const Vec3 midpoint = (a + b) * 0.5f;
		const Vec3 unitRadial = (midpoint - center).normalize();
		return center + (unitRadial * radius);
	}

	// given three points on the surface of this sphere, if the triangle
	// is "small enough" draw it, otherwise subdivide it into four smaller
	// triangles and recursivossup draw each of them.
	void drawMeshedTriangleOnSphere(const Vec3& a, const Vec3& b,
			const Vec3& c) const
	{
		// if all edges are short enough
		if ((((a - b).length()) < maxEdgeLength)
				&& (((b - c).length()) < maxEdgeLength)
				&& (((c - a).length()) < maxEdgeLength))
		{
			// draw triangle
			drawTriangleOnSphere(a, b, c);
		}
		else // otherwise subdivide and recurse
		{
			// find edge midpoints
			const Vec3 ab = midpointOnSphere(a, b);
			const Vec3 bc = midpointOnSphere(b, c);
			const Vec3 ca = midpointOnSphere(c, a);

			// recurse on four sub-triangles
			drawMeshedTriangleOnSphere(a, ab, ca);
			drawMeshedTriangleOnSphere(ab, b, bc);
			drawMeshedTriangleOnSphere(ca, bc, c);
			drawMeshedTriangleOnSphere(ab, bc, ca);
		}
	}

	// draw one mesh element for drawMeshedTriangleOnSphere
	void drawTriangleOnSphere(const Vec3& a, const Vec3& b, const Vec3& c) const
	{
		// draw triangle, subject to the camera orientation criteria
		// (according to drawBackFacing and drawFrontFacing)
		const Vec3 triCenter = (a + b + c) / 3.0f;
		const Vec3 triNormal = triCenter - center; // not unit length
		const Vec3 view = triCenter - viewpoint;
		const float dot = view.dot(triNormal); // project normal on view
		const bool seen = ((dot > 0.0f) ? drawBackFacing : drawFrontFacing);
		if (seen)
		{
			if (filled)
			{
				// draw filled triangle
				if (drawFrontFacing)
					drawTriangle(c, b, a, color);
				else
					drawTriangle(a, b, c, color);
			}
			else
			{
				// draw triangle edges (use trick to avoid drawing each
				// edge twice (for each adjacent triangle) unless we are
				// culling and this tri is near the sphere's silhouette)
				const float unitDot = view.dot(triNormal.normalize());
				const float t = 0.05f; // near threshold
				const bool nearSilhouette = (unitDot < t) || (unitDot > -t);
				if (nearSilhouette && !(drawBackFacing && drawFrontFacing))
				{
					drawLine(a, b, color);
					drawLine(b, c, color);
					drawLine(c, a, color);
				}
				else
				{
					drawMeshedTriangleLine(a, b, color);
					drawMeshedTriangleLine(b, c, color);
					drawMeshedTriangleLine(c, a, color);
				}
			}
		}
	}

	// Draws line from A to B but not from B to A: assumes each edge
	// will be drawn in both directions, picks just one direction for
	// drawing according to an arbitary but reproducable criterion.
	void drawMeshedTriangleLine(const Vec3& a, const Vec3& b,
			const Color& color) const
	{
		if (a.x != b.x)
		{
			if (a.x > b.x)
				drawLine(a, b, color);
		}
		else
		{
			if (a.y != b.y)
			{
				if (a.y > b.y)
					drawLine(a, b, color);
			}
			else
			{
				if (a.z > b.z)
					drawLine(a, b, color);
			}
		}
	}

};

// draw a sphere (wireframe or opaque, with front/back/both culling)
void drawSphere(const Vec3 center, const float radius,
		const float maxEdgeLength, const bool filled, const Color& color,
		const bool drawFrontFacing, const bool drawBackFacing,
		const Vec3& viewpoint)
{
	const DrawSphereHelper s(center, radius, maxEdgeLength, filled, color,
			drawFrontFacing, drawBackFacing, viewpoint);
	s.draw();
}

// draw a SphereObstacle
void drawSphereObstacle(const SphereObstacle& so, const float maxEdgeLength,
		const bool filled, const Color& color, const Vec3& viewpoint)
{
	bool front, back;
	switch (so.seenFrom())
	{
	default:
	case Obstacle::both:
		front = true;
		back = true;
		break;
	case Obstacle::inside:
		front = false;
		back = true;
		break;
	case Obstacle::outside:
		front = true;
		back = false;
		break;
	}
	drawSphere(so.center, so.radius, maxEdgeLength, filled, color, front, back,
			viewpoint);
}

///XXX
//Flags and functions expected by OpenSteer as defined
bool enableAnnotation = false;
bool drawPhaseActive = true;
bool updatePhaseActive = false;
void drawCameraLookAt(Vec3 const&, Vec3 const&,
		Vec3 const&)
{
}
void warnIfInUpdatePhase2(const char*)
{
}

} // namespace OpenSteer

// ----------------------------------------------------------------------------
