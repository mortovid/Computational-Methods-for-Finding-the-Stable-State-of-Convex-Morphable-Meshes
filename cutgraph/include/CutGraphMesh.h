#ifndef _CUT_GRAPH_MESH_
#define _CUT_GRAPH_MESH_

#include "Mesh/BaseMesh.h"
#include "Mesh/Edge.h"
#include "Mesh/Face.h"
#include "Mesh/HalfEdge.h"
#include "Mesh/Vertex.h"

#include "Mesh/Boundary.h"
#include "Mesh/Iterators.h"
#include "Parser/parser.h"

namespace MeshLib
{
  class CCutGraphVertex;
  class CCutGraphEdge;
  class CCutGraphFace;
  class CCutGraphHalfEdge;

  class CCurveVertex;

  /*! \brief CCutGraphVertex class
   *
   * Vertex class for the cut graph algorithm.
   * Trait: Vertex valence.
   */
  class CCutGraphVertex : public CVertex
  {
    public:
      /*! Constructor */
      CCutGraphVertex() 
        : m_valence(0), m_curvature(0.0f), m_height(0.0f), m_embedded(false), m_emb_point() 
      {}

      /*! Vertex valence */
      int& valence() { return m_valence; };

      float& curvature() { return m_curvature; };

      float& height() { return m_height; };

      bool& embedded() { return m_embedded; };

      CPoint& embPoint() { return m_emb_point; };
    
    protected:
      /*! Vertex valence */
      int m_valence;

      /*! Vertex curvature */
      float m_curvature;

      /*! Vertex height */
      float m_height;

      /*! Has vertex been embedded yet */
      bool m_embedded;

      /*! Embedded coordinates */
      CPoint m_emb_point;
  };

  /*! \brief CCutGraphEdge class
   *
   * Edge class for the cut graph algorithm.
   * Trait: Edge sharpness.
   */
  class CCutGraphEdge : public CEdge
  {
    public:
      /*! Constructor */
      CCutGraphEdge() 
        : m_sharp(false), m_power(0.0f) 
      {}

      /*! Sharp edge flag */
      bool& sharp() { return m_sharp; };

      float& power() { return m_power; };
    
    protected:
      /*! Sharp edge flag */
      bool m_sharp;

      /*! Edge power */
      float m_power;
  };

  /*! \brief CCutGraphFace class
   *
   * Face class for the cut graph algorithm.
   * Trait: Face touched flag.
   */
  class CCutGraphFace : public CFace
  {
    public:
      /*! Constructor */
      CCutGraphFace() 
        : m_touched(false), m_normal() 
      {}

      /*! Face touched flag */
      bool& touched() { return m_touched; };

      /*! Face normal */
      CPoint& normal() { return m_normal; };
    
    protected:
      /*! Face touched flag */
      bool m_touched;

      /*! Face normal */
      CPoint m_normal;
  };

  /*! \brief CCutGraphHalfEdge class
   *
   * HalfEdge class for the cut graph algorithm.
   */
  class CCutGraphHalfEdge : public CHalfEdge
  { 
    public:
      /*! Constructor */
      CCutGraphHalfEdge() 
        : m_diAngle(0.0f), m_vertAngle(0.0f), m_length(0.0f), m_power(0.0f) 
      {}

      /*! Dihedral angle */
      float& diAngle() { return m_diAngle; };

      /*! Vertical angle (between edge and (0,0,-1)) */
      float& vertAngle() { return m_vertAngle; };

      /*! Edge length */
      float& length() { return m_length; };

      /*! HalfEdge power */
      float& power() { return m_power; };
    
    protected:
      /*! Dihedral angle */
      float m_diAngle;

      /*! Vertical angle between edge and (0, 0, -1) */
      float m_vertAngle;

      /*! Length of the edge */
      float m_length;

      /*! Power of the halfedge */
      float m_power;
  };

  /*! \brief CCutGraphMesh class
   *
   * Mesh class for the cut graph algorithm.
   */
  template <typename V, typename E, typename F, typename H>
  class TCutGraphMesh : public CBaseMesh<V, E, F, H>
  {
    public:
      typedef V CVertex;
      typedef E CEdge;
      typedef F CFace;
      typedef H CHalfEdge;

      typedef CBoundary<V, E, F, H>                   CBoundary;
      typedef CLoop<V, E, F, H>                       CLoop;

      typedef MeshVertexIterator<V, E, F, H>          MeshVertexIterator;
      typedef MeshEdgeIterator<V, E, F, H>            MeshEdgeIterator;
      typedef MeshFaceIterator<V, E, F, H>            MeshFaceIterator;
      typedef MeshHalfEdgeIterator<V, E, F, H>        MeshHalfEdgeIterator;

      typedef VertexVertexIterator<V, E, F, H>        VertexVertexIterator;
      typedef VertexEdgeIterator<V, E, F, H>          VertexEdgeIterator;
      typedef VertexFaceIterator<V, E, F, H>          VertexFaceIterator;
      typedef VertexInHalfedgeIterator<V, E, F, H>    VertexInHalfedgeIterator;
      typedef VertexOutHalfedgeIterator<V, E, F, H>   VertexOutHalfedgeIterator;

      typedef FaceVertexIterator<V, E, F, H>          FaceVertexIterator;
      typedef FaceEdgeIterator<V, E, F, H>            FaceEdgeIterator;
      typedef FaceHalfedgeIterator<V, E, F, H>        FaceHalfedgeIterator;

      /*! Normal vector of the base mesh */
      CPoint& normalVec() { return m_normal; };

    protected:
      CPoint m_normal;
  };

  typedef TCutGraphMesh<CCutGraphVertex, CCutGraphEdge, CCutGraphFace, CCutGraphHalfEdge> CCutGraphMesh;

} // namespace MeshLib

#endif // !_CUT_GRAPH_MESH_
