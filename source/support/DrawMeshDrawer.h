/**
 * \file DrawMeshDrawer.h
 *
 * \date 2016-05-13
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
	///Index of the currently used generator.
	int m_generatorIdx;
	///Number of currently allocated generators.
	int m_generatorsSize;
	///Number of used generators during last frame.
	int m_generatorsSizeLast;
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

	void initialize();
	void finalize();
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
	float getTextScale()
	{
		return m_textScale;
	}
};

/// Panda3d debug draw implementation.
class DebugDrawPanda3d
{
protected:
	///The render node path.
	NodePath m_render;
	///Depth Mask.
	bool m_depthMask;
	///Texture.
	bool m_texture;
	///The current GeomVertexData.
	PT(GeomVertexData) m_vertexData;
	///The current vertex index.
	int m_vertexIdx;
	///The current GeomVertexWriters.
	GeomVertexWriter m_vertex, m_color, m_texcoord;
	///The current GeomPrimitive and draw type.
	PT(GeomPrimitive) m_geomPrim;
	DrawMeshDrawer::DrawPrimitive m_prim;
	///Size (for points)
	float m_size;
	///The current Geom.
	PT(Geom) m_geom;
	///The current GeomNode node path.
	NodePath m_geomNodeNP;
	///The GeomNodes' node paths.
	std::vector<NodePath> m_geomNodeNPCollection;
	///QUADS stuff.
	int m_quadCurrIdx;
	LVector3f m_quadFirstVertex, m_quadThirdVertex;
	LVector4f m_quadFirstColor, m_quadThirdColor;
	LVector2f m_quadFirstUV, m_quadThirdUV;

public:
	DebugDrawPanda3d(NodePath render);
	virtual ~DebugDrawPanda3d();

	void reset();

	virtual void depthMask(bool state);
	virtual void texture(bool state);
	virtual void begin(DrawMeshDrawer::DrawPrimitive prim, float size = 1.0f);
	void vertex(const LVector3f& vertex, const LVector4f& color,
			const LVector2f& uv = LVecBase2f::zero());
	virtual void end();

	NodePath getGeomNode(int i);
	int getGeomNodesNum();

};

} //namespace ossup

#endif /* DRAWMESHDRAWER_H_ */
