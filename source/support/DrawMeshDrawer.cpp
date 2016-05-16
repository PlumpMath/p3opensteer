/**
 * \file DrawMeshDrawer.cpp
 *
 * \date 2016-05-13
 * \author consultit
 */

#include "DrawMeshDrawer.h"
#include "common.h"
#include <omniBoundingVolume.h>
#include <textNode.h>
#include <geomPoints.h>
#include <geomLines.h>

namespace ossup
{

DrawMeshDrawer::DrawMeshDrawer(NodePath render, NodePath camera, int budget,
		float textScale, bool singleMesh) :
		m_render(render), m_camera(camera), m_depthMask(true), m_generatorsSize(
				0), m_generatorsSizeLast(0), m_budget(budget), m_singleMesh(
				singleMesh), m_size(1.0 / 50.0), m_twoSided(false), m_textNodeIdx(
				0), m_prevTextNodeIdx(-1), m_textNodesSize(0), m_textScale(
				textScale)
{
	initialize();
}

DrawMeshDrawer::~DrawMeshDrawer()
{
	for (unsigned int i = 0; i < m_generators.size(); ++i)
	{
		delete m_generators[i];
	}
}

void DrawMeshDrawer::initialize()
{
	//reset current MeshDrawer in this frame
	m_generatorIdx = 0;
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

void DrawMeshDrawer::finalize()
{
	//clean drawing of unused generators in this frame:
	//m_meshDrawerIdx = number of most recently used generators
	///NOTE: alternatively unused generators can be deallocated:
	/// it should be less performant.
	if (m_generatorIdx < m_generatorsSizeLast)
	{
		//
		for (int i = m_generatorIdx; i < m_generatorsSizeLast; ++i)
		{
			m_generators[i]->begin(m_camera, m_render);
			m_generators[i]->end();
		}
	}
	//update m_meshDrawersSizeLast
	m_generatorsSizeLast = m_generatorIdx;
}

void DrawMeshDrawer::clear()
{
	//reset to initial values
	m_generatorIdx = 0;
	m_generatorsSize = 0;
	m_prim = NULL_PRIM;
	m_textNodeIdx = 0;
	m_prevTextNodeIdx = -1;
	m_textNodesSize = 0;
	//clear internal storage
	{
		std::vector<MeshDrawer*>::iterator iter;
		for (iter = m_generators.begin(); iter != m_generators.end(); ++iter)
		{
			delete (*iter);
		}
	}
	{
		std::vector<NodePath>::iterator iter;
		for (iter = m_textNodes.begin(); iter != m_textNodes.end(); ++iter)
		{
			m_textNodes[iter - m_textNodes.begin()].remove_node();
		}
	}
	m_generators.clear();
	m_textNodes.clear();
}

void DrawMeshDrawer::begin(DrawPrimitive prim)
{
	//dynamically allocate MeshDrawers if necessary
	if (m_generatorIdx >= m_generatorsSize)
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
		m_generatorsSize = m_generators.size();
	}
	//setup current MeshDrawer
	m_generators[m_generatorIdx]->get_root().set_depth_write(m_depthMask);
//	m_generator[m_generatorIdx]->get_root().set_render_mode_thickness(m_size);
	m_generators[m_generatorIdx]->get_root().set_two_sided(m_twoSided);
	//begin current MeshDrawer
	m_generators[m_generatorIdx]->begin(m_camera, m_render);
	m_prim = prim;
	m_lineIdx = m_triIdx = m_quadIdx = 0;
	m_triStripUp = true;
}

void DrawMeshDrawer::end()
{
	//close line loop (if any)
	if (m_prim == DRAW_LINELOOP && (m_lineVertex != m_vertexLoop0))
	{
		m_generators[m_generatorIdx]->segment(m_lineVertex, m_vertexLoop0,
				LVector4f(m_lineUV.get_x(), m_lineUV.get_y(), m_uvLoop0.get_x(),
						m_uvLoop0.get_y()), m_size, m_colorLoop0);
	}

	//m_lineIdx can assume 0,1,3 values: 1 means an incomplete line
	ASSERT_TRUE((m_lineIdx == 0) || (m_lineIdx == 3))
	//m_triIdx can assume 0,1,2,5 values: 1,2 means an incomplete triangle
	ASSERT_TRUE((m_triIdx == 0) || (m_triIdx == 5))
	ASSERT_TRUE(m_quadIdx == 0)

	//end current MeshDrawer
	m_generators[m_generatorIdx]->end();
	//increase MeshDrawer index only if multiple mesh
	if (!m_singleMesh)
	{
		++m_generatorIdx;
	}
	m_prim = NULL_PRIM;
}

void DrawMeshDrawer::vertex(const LVector3f& vertex, const LVector2f& uv)
{
	switch (m_prim)
	{
	case DRAW_POINTS:
		m_generators[m_generatorIdx]->billboard(vertex,
				LVector4f(uv.get_x(), uv.get_y(), uv.get_x(), uv.get_y()),
				m_size, m_color);
		break;
	case DRAW_LINES:
		if ((m_lineIdx % 2) == 1)
		{
			m_generators[m_generatorIdx]->segment(m_lineVertex, vertex,
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
			m_generators[m_generatorIdx]->segment(m_lineVertex, vertex,
					LVector4f(m_lineUV.get_x(), m_lineUV.get_y(), uv.get_x(),
							uv.get_y()), m_size, m_color);
			m_lineVertex = vertex;
#ifdef OS_DEBUG
			m_lineIdx = 3; //flag: it signals that at least 2 points have been used.
#endif //OS_DEBUG
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
			m_generators[m_generatorIdx]->segment(m_lineVertex, vertex,
					LVector4f(m_lineUV.get_x(), m_lineUV.get_y(), uv.get_x(),
							uv.get_y()), m_size, m_color);
			m_lineVertex = vertex;
#ifdef OS_DEBUG
			m_lineIdx = 3; //flag: it signals that at least 2 points have been used.
#endif //OS_DEBUG
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
			m_generators[m_generatorIdx]->tri(m_triVertex[0], m_triColor[0],
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
			m_generators[m_generatorIdx]->tri(m_triVertex[0], m_triColor[0],
					m_triUV[0], m_triVertex[1], m_triColor[1], m_triUV[1],
					vertex, m_color, uv);
			//store last vertex into vertex[1]
			m_triVertex[1] = vertex;
			m_triColor[1] = m_color;
			m_triUV[1] = uv;
#ifdef OS_DEBUG
			m_triIdx = 5; //flag: it signals that at least 3 points have been used.
#endif //OS_DEBUG
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
			m_generators[m_generatorIdx]->tri(m_triVertex[0], m_triColor[0],
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
#ifdef OS_DEBUG
			m_triIdx = 5; //flag: it signals that at least 3 points have been used.
#endif //OS_DEBUG
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
			m_generators[m_generatorIdx]->tri(m_quadVertex[0], m_quadColor[0],
					m_quadUV[0], m_quadVertex[1], m_quadColor[1], m_quadUV[1],
					m_quadVertex[2], m_quadColor[2], m_quadUV[2]);
			m_generators[m_generatorIdx]->tri(m_quadVertex[0], m_quadColor[0],
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

////////////////////////////////////////////////////////////////////////////////////////////////////

DebugDrawPanda3d::DebugDrawPanda3d(NodePath render) :
		m_render(render), m_depthMask(true), m_texture(true), m_vertexIdx(0), m_prim(
				DrawMeshDrawer::DRAW_TRIS), m_size(0), m_quadCurrIdx(0)
{
}

DebugDrawPanda3d::~DebugDrawPanda3d()
{
}

void DebugDrawPanda3d::depthMask(bool state)
{
	m_depthMask = state;
}

void DebugDrawPanda3d::texture(bool state)
{
	m_texture = state;
}

void DebugDrawPanda3d::begin(DrawMeshDrawer::DrawPrimitive prim, float size)
{
	m_vertexData = new GeomVertexData("VertexData",
			GeomVertexFormat::get_v3c4t2(), Geom::UH_static);
	m_vertex = GeomVertexWriter(m_vertexData, "vertex");
	m_color = GeomVertexWriter(m_vertexData, "color");
	m_texcoord = GeomVertexWriter(m_vertexData, "texcoord");
	switch (prim)
	{
	case DrawMeshDrawer::DRAW_POINTS:
		m_geomPrim = new GeomPoints(Geom::UH_static);
		break;
	case DrawMeshDrawer::DRAW_LINES:
		m_geomPrim = new GeomLines(Geom::UH_static);
		break;
	case DrawMeshDrawer::DRAW_TRIS:
		m_geomPrim = new GeomTriangles(Geom::UH_static);
		break;
	case DrawMeshDrawer::DRAW_QUADS:
		m_geomPrim = new GeomTriangles(Geom::UH_static);
		m_quadCurrIdx = 0;
		break;
	default:
		break;
	};
	m_prim = prim;
	m_size = size;
	m_vertexIdx = 0;
}

void DebugDrawPanda3d::vertex(const LVector3f& vertex, const LVector4f& color,
		const LVector2f& uv)
{
	if (m_prim != DrawMeshDrawer::DRAW_QUADS)
	{
		m_vertex.add_data3f(vertex);
		m_color.add_data4f(color);
		m_texcoord.add_data2f(uv);
		//
		m_geomPrim->add_vertex(m_vertexIdx);
		++m_vertexIdx;
	}
	else
	{
		int quadCurrIdxMod = m_quadCurrIdx % 4;
		LVector3f currVertex = vertex;
		LVector4f currColor = color;
		LVector2f currUV = uv;
		switch (quadCurrIdxMod)
		{
		case 0:
			m_quadFirstVertex = currVertex;
			m_quadFirstColor = currColor;
			m_quadFirstUV = currUV;
			++m_quadCurrIdx;
			break;
		case 2:
			m_quadThirdVertex = currVertex;
			m_quadThirdColor = currColor;
			m_quadThirdUV = currUV;
			++m_quadCurrIdx;
			break;
		case 3:
			//first
			m_vertex.add_data3f(m_quadFirstVertex);
			m_color.add_data4f(m_quadFirstColor);
			m_texcoord.add_data2f(m_quadFirstUV);
			//
			m_geomPrim->add_vertex(m_vertexIdx);
			++m_vertexIdx;
			//last
			m_vertex.add_data3f(m_quadThirdVertex);
			m_color.add_data4f(m_quadThirdColor);
			m_texcoord.add_data2f(m_quadThirdUV);
			//
			m_geomPrim->add_vertex(m_vertexIdx);
			++m_vertexIdx;
			m_quadCurrIdx = 0;
			break;
		case 1:
			++m_quadCurrIdx;
			break;
		default:
			break;
		};
		//current vertex
		///
		m_vertex.add_data3f(currVertex);
		m_color.add_data4f(currColor);
		m_texcoord.add_data2f(currUV);
		//
		m_geomPrim->add_vertex(m_vertexIdx);
		++m_vertexIdx;
		///
	}
}

void DebugDrawPanda3d::end()
{
	m_geomPrim->close_primitive();
	m_geom = new Geom(m_vertexData);
	m_geom->add_primitive(m_geomPrim);
	m_geomNodeNP =
			NodePath(
					new GeomNode(
							"DebugDrawPanda3d_GeomNode_"
									+ dynamic_cast<std::ostringstream&>(std::ostringstream().operator <<(
											m_geomNodeNPCollection.size())).str()));
	DCAST(GeomNode, m_geomNodeNP.node())->add_geom(m_geom);
	m_geomNodeNP.reparent_to(m_render);
	m_geomNodeNP.set_depth_write(m_depthMask);
	m_geomNodeNP.set_transparency(TransparencyAttrib::M_alpha);
	m_geomNodeNP.set_render_mode_thickness(m_size);
	//add to geom node paths.
	m_geomNodeNPCollection.push_back(m_geomNodeNP);
}

NodePath DebugDrawPanda3d::getGeomNode(int i)
{
	return m_geomNodeNPCollection[i];
}

int DebugDrawPanda3d::getGeomNodesNum()
{
	return m_geomNodeNPCollection.size();
}

void DebugDrawPanda3d::reset()
{
	std::vector<NodePath>::iterator iter;
	for (iter = m_geomNodeNPCollection.begin();
			iter != m_geomNodeNPCollection.end(); ++iter)
	{
		(*iter).remove_node();
	}
	m_geomNodeNPCollection.clear();
}

}  // namespace ossup

