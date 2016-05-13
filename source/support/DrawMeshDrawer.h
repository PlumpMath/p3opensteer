/*
 *   This file is part of Ely.
 *
 *   Ely is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   Ely is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with Ely.  If not, see <http://www.gnu.org/licenses/>.
 */
/**
 * \file /Ely/include/Support/OpenSteerLocal/DrawMeshDrawer.h
 *
 * \date 2013-11-30 
 * \author consultit
 */
#ifndef DRAWMESHDRAWER_H_
#define DRAWMESHDRAWER_H_

#include <meshDrawer.h>

namespace ossup
{
/// MeshDrawer draw implementation.
class DrawMeshDrawer
{
public:
	enum DrawPrimitive
	{
		DRAW_POINTS,
		DRAW_LINES,
		DRAW_LINELOOP,
		DRAW_LINESTRIP,
		DRAW_TRIS,
		DRAW_TRIFAN,
		DRAW_TRISTRIP,
		DRAW_QUADS,
		NULL_PRIM
	};
protected:
	///The render node path.
	NodePath m_render;
	///The camera node path.
	NodePath m_camera;
	///Depth Mask.
	bool m_depthMask;
	///Texture.
	bool m_texture;
	///Inner MeshDrawers.
	std::vector<MeshDrawer*> m_generators;
	///Current MeshDrawer index.
	int m_meshDrawerIdx;
	///Current MeshDrawers number.
	int m_meshDrawersSize;
	///Budget.
	int m_budget;
	///Single mesh flag.
	bool m_singleMesh;
	///The current GeomPrimitive and draw type.
	DrawPrimitive m_prim;
	///Size (for points and lines).
	float m_size;
	///Two sided draw.
	bool m_twoSided;
	///Vertex color.
	LVector4f m_color;
	///Line previous store.
	LVecBase3f m_lineVertex, m_vertexLoop0;
	LVecBase4f m_lineColor, m_colorLoop0;
	LVecBase2f m_lineUV, m_uvLoop0;
	int m_lineIdx;
	///Triangle previous store.
	LVecBase3f m_triVertex[2];
	LVecBase4f m_triColor[2];
	LVecBase2f m_triUV[2];
	bool m_triStripUp;
	int m_triIdx;
	///Quad previous store.
	LVecBase3f m_quadVertex[3];
	LVecBase4f m_quadColor[3];
	LVecBase2f m_quadUV[3];
	int m_quadIdx;
	///Inner TextNode node paths.
	std::vector<NodePath> m_textNodes;
	///Current and previous TextNode index.
	int m_textNodeIdx, m_prevTextNodeIdx;
	///Current TextNodes number.
	int m_textNodesSize;
	///Text scale.
	float m_textScale;

public:
	DrawMeshDrawer(NodePath render, NodePath camera, int budget = 50,
			float textScale = 0.75, bool singleMesh = false);
	virtual ~DrawMeshDrawer();

	void reset();
	void clear();

	void begin(DrawPrimitive prim);
	void vertex(const LVector3f& vertex, const LVector2f& uv =
			LVector2f::zero());
	void end();
	void setColor(const LVector4f& color)
	{
		m_color = color;
	}
	void setSize(float size)
	{
		m_size = size / 50.0;
	}
	void setTwoSided(bool enable)
	{
		m_twoSided = enable;
	}
	void depthMask(bool state)
	{
		m_depthMask = state;
	}
	void texture(bool state)
	{
		m_texture = state;
	}
	void drawText(const std::string& text, const LPoint3f& location,
			const LVecBase4& color);
	void setTextScale(float textScale)
	{
		m_textScale = textScale;
	}
};
} //namespace ossup

#endif /* DRAWMESHDRAWER_H_ */
