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
 * \file /Ely/src/Support/OpenSteerLocal/DrawMeshDrawer.cpp
 *
 * \date 2013-11-30 
 * \author consultit
 */

#include "Support/OpenSteerLocal/DrawMeshDrawer.h"
#include "Support/OpenSteerLocal/common.h"
#include <omniBoundingVolume.h>
#include <textNode.h>

namespace ossup
{

DrawMeshDrawer::DrawMeshDrawer(NodePath render, NodePath camera, int budget,
		float textScale, bool singleMesh) :
		m_render(render), m_camera(camera), m_depthMask(true), m_meshDrawersSize(
				0), m_budget(budget), m_singleMesh(singleMesh), m_size(
				1.0 / 50.0), m_twoSided(false), m_textNodeIdx(0), m_prevTextNodeIdx(-1), m_textNodesSize(
				0), m_textScale(textScale)
{
	reset();
}

DrawMeshDrawer::~DrawMeshDrawer()
{
	for (unsigned int i = 0; i < m_generators.size(); ++i)
	{
		delete m_generators[i];
	}
}

void DrawMeshDrawer::reset()
{
	//reset current MeshDrawer index
	m_meshDrawerIdx = 0;
	m_prim = NULL_PRIM;
	//clear text on not used text nodes
	for (int idx = m_textNodeIdx; idx <= m_prevTextNodeIdx; ++idx)
	{
		DCAST(TextNode, m_textNodes[idx].node())->clear_text();
	}
	//reset current and previous TextNode indexes
	m_prevTextNodeIdx = m_textNodeIdx - 1;
	m_textNodeIdx = 0;
}

void DrawMeshDrawer::clear()
{
	//reset to initial values
	m_meshDrawerIdx = 0;
	m_meshDrawersSize = 0;
	m_prim = NULL_PRIM;
	m_textNodeIdx = 0;
	m_prevTextNodeIdx = -1;
	m_textNodesSize = 0;
	//clear internal storage
	for (unsigned int i; i < m_generators.size(); ++i)
	{
		delete m_generators[i];
	}
	for (unsigned int i; i < m_generators.size(); ++i)
	{
		m_textNodes[i].remove_node();
	}
	m_generators.clear();
	m_textNodes.clear();
}

void DrawMeshDrawer::begin(DrawPrimitive prim)
{
	//dynamically allocate MeshDrawers if necessary
	if (m_meshDrawerIdx >= m_meshDrawersSize)
	{
		//allocate a new MeshDrawer
		m_generators.push_back(new MeshDrawer());
		//common MeshDrawers setup
		m_generators.back()->set_budget(m_budget);
		m_generators.back()->get_root().set_transparency(
				TransparencyAttrib::M_alpha);
		m_generators.back()->get_root().node()->set_bounds(
				new OmniBoundingVolume());
		m_generators.back()->get_root().node()->set_final(true);
		m_generators.back()->get_root().reparent_to(m_render);
		//update number of MeshDrawers
		m_meshDrawersSize = m_generators.size();
	}
	//setup current MeshDrawer
	m_generators[m_meshDrawerIdx]->get_root().set_depth_write(m_depthMask);
//	m_generator[m_meshDrawerIdx]->get_root().set_render_mode_thickness(m_size);
	m_generators[m_meshDrawerIdx]->get_root().set_two_sided(m_twoSided);
	//begin current MeshDrawer
	m_generators[m_meshDrawerIdx]->begin(m_camera, m_render);
	m_prim = prim;
	m_lineIdx = m_triIdx = m_quadIdx = 0;
	m_triStripUp = true;
}

void DrawMeshDrawer::end()
{
	//close line loop (if any)
	if (m_prim == DRAW_LINELOOP and (m_lineVertex != m_vertexLoop0))
	{
		m_generators[m_meshDrawerIdx]->segment(m_lineVertex, m_vertexLoop0,
				LVector4f(m_lineUV.get_x(), m_lineUV.get_y(), m_uvLoop0.get_x(),
						m_uvLoop0.get_y()), m_size, m_colorLoop0);
	}
	//end current MeshDrawer
	m_generators[m_meshDrawerIdx]->end();
	//increase MeshDrawer index only if multiple mesh
	if (not m_singleMesh)
	{
		++m_meshDrawerIdx;
	}
	m_prim = NULL_PRIM;
}

void DrawMeshDrawer::vertex(const LVector3f& vertex, const LVector2f& uv)
{
	switch (m_prim)
	{
	case DRAW_POINTS:
		m_generators[m_meshDrawerIdx]->billboard(vertex,
				LVector4f(uv.get_x(), uv.get_y(), uv.get_x(), uv.get_y()),
				m_size, m_color);
		break;
	case DRAW_LINES:
		if ((m_lineIdx % 2) == 1)
		{
			m_generators[m_meshDrawerIdx]->segment(m_lineVertex, vertex,
					LVector4f(m_lineUV.get_x(), m_lineUV.get_y(), uv.get_x(),
							uv.get_y()), m_size, m_color);
			m_lineIdx = 0;
		}
		else
		{
			m_lineVertex = vertex;
			m_lineColor = m_color;
			m_lineUV = uv;
			++m_lineIdx;
		}
		break;
	case DRAW_LINESTRIP:
		if ((m_lineIdx % 2) == 1)
		{
			m_generators[m_meshDrawerIdx]->segment(m_lineVertex, vertex,
					LVector4f(m_lineUV.get_x(), m_lineUV.get_y(), uv.get_x(),
							uv.get_y()), m_size, m_color);
			m_lineVertex = vertex;
		}
		else
		{
			m_lineVertex = vertex;
			m_lineColor = m_color;
			m_lineUV = uv;
			++m_lineIdx;
		}
		break;
	case DRAW_LINELOOP:
		if ((m_lineIdx % 2) == 1)
		{
			m_generators[m_meshDrawerIdx]->segment(m_lineVertex, vertex,
					LVector4f(m_lineUV.get_x(), m_lineUV.get_y(), uv.get_x(),
							uv.get_y()), m_size, m_color);
			m_lineVertex = vertex;
		}
		else
		{
			m_lineVertex = m_vertexLoop0 = vertex;
			m_lineColor = m_colorLoop0 = m_color;
			m_lineUV = m_uvLoop0 = uv;
			++m_lineIdx;
		}
		break;
	case DRAW_TRIS:
		if ((m_triIdx % 3) == 2)
		{
			m_generators[m_meshDrawerIdx]->tri(m_triVertex[0], m_triColor[0],
					m_triUV[0], m_triVertex[1], m_triColor[1], m_triUV[1],
					vertex, m_color, uv);
			m_triIdx = 0;
		}
		else
		{
			m_triVertex[m_triIdx] = vertex;
			m_triColor[m_triIdx] = m_color;
			m_triUV[m_triIdx] = uv;
			++m_triIdx;
		}
		break;
	case DRAW_TRIFAN:
		if ((m_triIdx % 3) == 2)
		{
			m_generators[m_meshDrawerIdx]->tri(m_triVertex[0], m_triColor[0],
					m_triUV[0], m_triVertex[1], m_triColor[1], m_triUV[1],
					vertex, m_color, uv);
			//store last vertex into vertex[1]
			m_triVertex[1] = vertex;
			m_triColor[1] = m_color;
			m_triUV[1] = uv;
		}
		else
		{
			m_triVertex[m_triIdx] = vertex;
			m_triColor[m_triIdx] = m_color;
			m_triUV[m_triIdx] = uv;
			++m_triIdx;
		}
		break;
	case DRAW_TRISTRIP:
		if ((m_triIdx % 3) == 2)
		{
			m_generators[m_meshDrawerIdx]->tri(m_triVertex[0], m_triColor[0],
					m_triUV[0], m_triVertex[1], m_triColor[1], m_triUV[1],
					vertex, m_color, uv);
			//slide vertices and toggle stripUp
			if (m_triStripUp)
			{
				m_triVertex[0] = vertex;
				m_triColor[0] = m_color;
				m_triUV[0] = uv;
				m_triStripUp = false;
			}
			else
			{
				m_triVertex[1] = vertex;
				m_triColor[1] = m_color;
				m_triUV[1] = uv;
				m_triStripUp = true;
			}
		}
		else
		{
			m_triVertex[m_triIdx] = vertex;
			m_triColor[m_triIdx] = m_color;
			m_triUV[m_triIdx] = uv;
			++m_triIdx;
		}
		break;
	case DRAW_QUADS:
		if ((m_quadIdx % 4) == 3)
		{
			m_generators[m_meshDrawerIdx]->tri(m_quadVertex[0], m_quadColor[0],
					m_quadUV[0], m_quadVertex[1], m_quadColor[1], m_quadUV[1],
					m_quadVertex[2], m_quadColor[2], m_quadUV[2]);
			m_generators[m_meshDrawerIdx]->tri(m_quadVertex[0], m_quadColor[0],
					LVector2f::zero(), m_quadVertex[2], m_quadColor[2],
					LVector2f::zero(), vertex, m_color, uv);
			m_quadIdx = 0;
		}
		else
		{
			m_quadVertex[m_quadIdx] = vertex;
			m_quadColor[m_quadIdx] = m_color;
			m_quadUV[m_quadIdx] = uv;
			++m_quadIdx;
		}
		break;
	case NULL_PRIM:
	default:
		break;
	};
}

void DrawMeshDrawer::drawText(const std::string& text, const LPoint3f& location,
		const LVecBase4& color)
{
	//dynamically allocate TextNodes if necessary
	if (m_textNodeIdx >= m_textNodesSize)
	{
		//allocate a new TextNode
		std::string textNum =
				dynamic_cast<std::ostringstream&>(std::ostringstream().operator <<(
						m_textNodeIdx)).str();
		m_textNodes.push_back(NodePath(new TextNode("TextNode-" + textNum)));
		//common TextNodes setup
		m_textNodes.back().reparent_to(m_render);
		m_textNodes.back().set_scale(m_textScale);
		m_textNodes.back().set_bin("fixed", 50);
		m_textNodes.back().set_depth_write(false);
		m_textNodes.back().set_depth_test(false);
		m_textNodes.back().set_billboard_point_eye();
		//update number of TextNodes
		m_textNodesSize = m_textNodes.size();
	}
	//setup current TextNode
	DCAST(TextNode, m_textNodes[m_textNodeIdx].node())->set_text(text);
	m_textNodes[m_textNodeIdx].set_pos(location);
	m_textNodes[m_textNodeIdx].set_color(color);
	//increase index
	++m_textNodeIdx;
}

}  // namespace ossup

