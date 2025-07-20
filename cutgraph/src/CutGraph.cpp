#include <queue>
#include "CutGraph.h"

const double pi = 3.14159265358979323846;

void MeshLib::CCutGraph::cut_graph()
{
    _dual_spanning_tree();

    // The cut graph contains all edges which their duals are not
    // in the spanning tree.
    for (CCutGraphMesh::MeshEdgeIterator eiter(m_pMesh); !eiter.end(); ++eiter)
    {
        CCutGraphEdge* pE = *eiter;
        pE->sharp() = !pE->sharp();
    }

    _prune();
}

/*----------------------------------------------------------------------------

Modify the method CCutGraph::_dual_spanning_tree()

------------------------------------------------------------------------------*/

void MeshLib::CCutGraph::_dual_spanning_tree()
{
    // Mark all sharp flags false
    for (CCutGraphMesh::MeshEdgeIterator eiter(m_pMesh); !eiter.end(); ++eiter)
    {
        CCutGraphEdge* pE = *eiter;
        pE->sharp() = false;
    }

    // Mark all touched flags false, and select a face
    CCutGraphFace* pHeadFace = NULL;
    for (CCutGraphMesh::MeshFaceIterator fiter(m_pMesh); !fiter.end(); ++fiter)
    {
        CCutGraphFace* pF = *fiter;
        pF->touched() = false; //select a face?
        pHeadFace = pF;
    }

    // 1. Construct the dual mesh conceptually.

    // 2. Generate a minimal spanning tree of the vertices in the dual mesh.
    std::queue<CCutGraphFace*> fQueue;
    fQueue.push(pHeadFace);
    pHeadFace->touched() = true;
    while (!fQueue.empty())
    {
        CCutGraphFace* pF = fQueue.front();
        fQueue.pop();

        for (CCutGraphMesh::FaceHalfedgeIterator fhiter(pF); !fhiter.end(); ++fhiter)
        {
            CCutGraphHalfEdge* pH = *fhiter;
            CCutGraphHalfEdge* pSymH = m_pMesh->halfedgeSym(pH);
            if (pSymH != NULL)
            {
                CCutGraphFace* pSymFace = m_pMesh->halfedgeFace(pSymH);
                if (!pSymFace->touched())
                {
                    //insert your code here
                }
            }
        }
    }
}

/*----------------------------------------------------------------------------

Modify the method _CCutGraph::_prune()

------------------------------------------------------------------------------*/

void MeshLib::CCutGraph::_prune()
{
    // A queue used to store valence-1 vertices
    std::queue<CCutGraphVertex*> vQueue;

    // 1. Compute the valence of each vertex, and record all valence-1 vertices.
    for (CCutGraphMesh::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter)
    {
        CCutGraphVertex* pV = *viter;
        // pV->valence() = 0;

        for (CCutGraphMesh::VertexEdgeIterator veiter(pV); !veiter.end(); ++veiter)
        {
            CCutGraphEdge* pE = *veiter;
            //insert your code here
        }

        if (pV->valence() == 1)
            vQueue.push(pV);
    }

    // 2. Remove the segments which attached to valence-1 vertices.
    while (!vQueue.empty())
    {
        CCutGraphVertex* pV = vQueue.front();
        vQueue.pop();

        for (CCutGraphMesh::VertexEdgeIterator veiter(pV); !veiter.end(); ++veiter)
        {
            CCutGraphEdge* pE = *veiter;
            CCutGraphVertex* pW = m_pMesh->edgeVertex1(pE) == pV ?
                m_pMesh->edgeVertex2(pE) : m_pMesh->edgeVertex1(pE);
            if (pE->sharp())
            {
                //insert your code here
                break;
            }
        }
    }
}

void MeshLib::embed(CCutGraphVertex v0, CCutGraphVertex v1, CCutGraphVertex v2, CPoint* proj0, CPoint* proj1, CPoint* proj2) {
    double proj01 = sqrt(pow((v0.point() - v1.point()).norm(), 2) - pow(v0.height() - v1.height(), 2));
    double proj02 = sqrt(pow((v0.point() - v2.point()).norm(), 2) - pow(v0.height() - v2.height(), 2));
    double proj12 = sqrt(pow((v1.point() - v2.point()).norm(), 2) - pow(v1.height() - v2.height(), 2));
    double angle102 = acos((proj01 * proj01 + proj02 * proj02 - proj12 * proj12) / (2 * proj01 * proj02));

    *proj0 = CPoint(0, 0, 0);
    *proj1 = CPoint(proj01, 0, 0);
    *proj2 = CPoint(proj02 * cos(angle102), proj02 * sin(angle102), 0);
}


bool MeshLib::CCutGraph::computeCurvature() {
    for (CCutGraphMesh::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter) {
        CCutGraphVertex* v0 = *viter;

        if (!v0->boundary()) {
            double result = 0;

            for (CCutGraphMesh::VertexOutHalfedgeIterator vheiter(m_pMesh, v0); !vheiter.end(); ++vheiter) {
                CCutGraphHalfEdge* he = *vheiter;
                CCutGraphVertex* v1 = m_pMesh->idVertex(he->target()->id());
                CCutGraphVertex* v2 = m_pMesh->idVertex(he->he_next()->target()->id());

                if (abs(v0->height() - v1->height()) > (v0->point() - v1->point()).norm()
                    || abs(v0->height() - v2->height()) > (v0->point() - v2->point()).norm()
                    || abs(v1->height() - v2->height()) > (v1->point() - v2->point()).norm()) {
                    return false;
                }

                double proj01 = sqrt(pow((v0->point() - v1->point()).norm(), 2) - pow(v0->height() - v1->height(), 2));
                double proj02 = sqrt(pow((v0->point() - v2->point()).norm(), 2) - pow(v0->height() - v2->height(), 2));
                double proj12 = sqrt(pow((v1->point() - v2->point()).norm(), 2) - pow(v1->height() - v2->height(), 2));
                result += acos((proj01 * proj01 + proj02 * proj02 - proj12 * proj12) / (2 * proj01 * proj02));
            }

            v0->curvature() = 2 * pi - result;
        }
        else {
            v0->curvature() = 0;
        }
    }
    return true;
}

void MeshLib::CCutGraph::computeDihedralVertAngles() { // REDO
    for (CCutGraphMesh::MeshHalfEdgeIterator heiter(m_pMesh); !heiter.end(); ++heiter) {
        CCutGraphHalfEdge* he = *heiter;

        CCutGraphVertex* v0 = m_pMesh->idVertex(he->source()->id());
        CCutGraphVertex* v1 = m_pMesh->idVertex(he->target()->id());
        CCutGraphVertex* v2 = m_pMesh->idVertex(he->he_prev()->source()->id()); // this is the opposite vertex
        CPoint proj0, proj1, proj2;
        embed(*v0, *v1, *v2, &proj0, &proj1, &proj2);

        proj1[2] += std::abs(v1->height()) - std::abs(v0->height());
        proj2[2] += std::abs(v2->height()) - std::abs(v0->height());

        // common edge is 01

        CPoint vecb0(0, 0, -1);

        he->vertAngle() = std::acos((vecb0 * proj1) / proj1.norm());

        
        CPoint alt2 = proj2 - proj1 * (proj1 * proj2) / (proj1.norm() * proj1.norm());
        CPoint altB = vecb0 - proj1 * std::cos(he->vertAngle()) / proj1.norm();
        he->diAngle() = std::acos((alt2 * altB) / (alt2.norm() * altB.norm()));
        /*
        CPoint cross12 = (proj1) ^ proj2;
        CPoint cross1b = (proj1) ^ vecb0;
        he->diAngle() = std::acos((cross12 * cross1b) / (cross12.norm() * cross1b.norm()));
        */
        /*
        if (!he->source()->boundary() || !he->target()->boundary()) {
            if (he->diAngle() > 1.58) {
                std::cout << "(" << proj0[0] << ", " << proj0[1] << ", " << proj0[2] << "), " << "\n"
                    << "(" << proj1[0] << ", " << proj1[1] << ", " << proj1[2] << "), " << "\n"
                    << "(" << proj2[0] << ", " << proj2[1] << ", " << proj2[2] << "), " << "\n" << he->vertAngle() << ", " << he->diAngle() << "\n";
            }
        } */
    }
}

void MeshLib::CCutGraph::computeEdgePower() { // REDO
    for (CCutGraphMesh::MeshHalfEdgeIterator heiter(m_pMesh); !heiter.end(); ++heiter) {
        CCutGraphHalfEdge* he = *heiter;
        if (!he->source()->boundary() || !he->target()->boundary()) {
            double result = (std::cos(he->diAngle()) / std::sin(he->diAngle())) + (std::cos(m_pMesh->CCutGraphMesh::halfedgeSym(he)->diAngle()) / std::sin(m_pMesh->CCutGraphMesh::halfedgeSym(he)->diAngle()));
            result /= (he->length() * std::sin(he->vertAngle()) * std::sin(he->vertAngle()));
            he->power() = result;
        }
        else { he->power() = 0; };
    }
}

double MeshLib::CCutGraph::computeTSC() { // REDO
    double result = 0;
    for (CCutGraphMesh::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter) {
        CCutGraphVertex* v = *viter;
        if (!v->boundary()) {
            result += std::abs(v->height()) * v->curvature();
        }
    }
    for (CCutGraphMesh::MeshHalfEdgeIterator heiter(m_pMesh); !heiter.end(); ++heiter) {
        CCutGraphHalfEdge* he = *heiter;
        if (he->source()->boundary() && he->target()->boundary()) {
            result += he->length() * (pi - he->diAngle());
        }
        else {
            result += he->length() * (pi - he->diAngle() - m_pMesh->halfedgeSym(he)->diAngle());
        }
    }

    return result;
}

bool MeshLib::CCutGraph::checkConvex() {
    for (CCutGraphMesh::MeshHalfEdgeIterator heiter(m_pMesh); !heiter.end(); ++heiter) {
        CCutGraphHalfEdge* he = *heiter;
        if (isnan(he->diAngle())) {
            return false;
        }
    }
    for (CCutGraphMesh::MeshHalfEdgeIterator heiter(m_pMesh); !heiter.end(); ++heiter) {
        CCutGraphHalfEdge* he = *heiter;
        if (!he->source()->boundary() || !he->target()->boundary()) {
            if (he->diAngle() + m_pMesh->halfedgeSym(he)->diAngle() > pi + 0.0003) {
                return false;
            }
        }
    }
    return true;
}

void MeshLib::CCutGraph::initGraph() {
    for (CCutGraphMesh::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter) {
        CCutGraphVertex* v = *viter;
        v->height() = 0;
        v->embedded() = false;
        int valence = 0;
        for (CCutGraphMesh::VertexEdgeIterator veiter(v); !veiter.end(); ++veiter) {
            valence++;
        }
        int fCounter = 0;
        for (CCutGraphMesh::VertexFaceIterator vfiter(v); !vfiter.end(); ++vfiter) {
            fCounter++;
        }
        if (valence > fCounter) {
            v->boundary() = true;
        }
        else {
            v->boundary() = false;
        }
    }
    for (CCutGraphMesh::MeshHalfEdgeIterator heiter(m_pMesh); !heiter.end(); ++heiter) {
        CCutGraphHalfEdge* he = *heiter;
        he->length() = (he->source()->point() - he->target()->point()).norm();
    }
}