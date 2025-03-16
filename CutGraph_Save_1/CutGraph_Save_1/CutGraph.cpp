#include <queue>
#include "CutGraph.h"

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
    float proj01 = sqrt(pow((v0.point() - v1.point()).norm(), 2) - pow(v0.height() - v1.height(), 2));
    float proj02 = sqrt(pow((v0.point() - v2.point()).norm(), 2) - pow(v0.height() - v2.height(), 2));
    float proj12 = sqrt(pow((v1.point() - v2.point()).norm(), 2) - pow(v1.height() - v2.height(), 2));
    float angle = acos((proj01 * proj01 + proj02 * proj02 - proj12 * proj12) / ((float)2 * proj01 * proj02));

    if (proj12 != proj12) { // debugging stuff
        std::cout << "proj error -- please exit \n";
        std::cout << proj01 << ", " << proj02 << ", " << proj12 << " are the sides \n";
        std::cout << pow((v1.point() - v2.point()).norm(), 1) << " for norm, " << pow(v1.height() - v2.height(), 1) << " for height, "
            << pow((v1.point() - v2.point()).norm(), 1) - pow(v1.height() - v2.height(), 1) << " for full \n";
        std::cout << (v0.point() - v1.point()).norm() << ", " << v0.height() - v1.height() << ". \n";
        std::string temp;
        std::cin >> temp;
    }
    else if (angle != angle) {
        std::cout << "angle error -- please exit \n";
        std::cout << proj01 << ", " << proj02 << ", " << proj12 << " are the sides \n";
        std::string temp;
        std::cin >> temp;
    }

    *proj0 = MeshLib::CPoint(0, 0, 0);
    *proj1 = MeshLib::CPoint(proj01, 0, 0);
    *proj2 = MeshLib::CPoint(proj02 * cos(angle), proj02 * sin(angle), 0);
}


void MeshLib::CCutGraph::compCurvature() {

    for (CCutGraphMesh::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter) {
        CCutGraphVertex* v0 = *viter;

        if (!v0->boundary()) {
            float result = 0;

            for (CCutGraphMesh::VertexOutHalfedgeIterator vheiter(m_pMesh, v0); !vheiter.end(); ++vheiter) {
                CCutGraphHalfEdge* he = *vheiter;

                CCutGraphVertex* v1 = m_pMesh->idVertex(he->target()->id());
                CCutGraphVertex* v2 = m_pMesh->idVertex(he->he_prev()->source()->id());

                CPoint proj0, proj1, proj2;
                embed(*v0, *v1, *v2, &proj0, &proj1, &proj2);

                float l01 = (proj0 - proj1).norm();
                float l02 = (proj0 - proj2).norm();
                float l12 = (proj1 - proj2).norm();

                result += std::acos((l01 * l01 + l02 * l02 - l12 * l12) / (2 * l01 * l02)); // here
            }

            v0->curvature() = std::atan(1) * 8 - result;
            if (v0->curvature() < -0.000001) {
                std::cout << "\nNegative curvature: " << std::atan(1) * 8 - result << ")\n";
                std::cout << "ID " << v0->id() << ": (" << v0->point()(0) << ", " << v0->point()(1) << ", " << v0->point()(2) << "), " << v0->height() << "\n \n";
                for (CCutGraphMesh::VertexVertexIterator vviter(v0); !vviter.end(); ++vviter) {
                    CCutGraphVertex* v = *vviter;
                    std::cout << "ID " << v->id() << ": (" << v->point()(0) << ", " << v->point()(1) << ", " << v->point()(2) << "), " << v->height() << "\n";
                }
                exit(2);
                std::string temp;
                std::cin >> temp;
            }
        }
        else {
            v0->curvature() = -1;
        }
    }
}

void MeshLib::CCutGraph::compDihedralVertAngles() {
    for (CCutGraphMesh::MeshHalfEdgeIterator heiter(m_pMesh); !heiter.end(); ++heiter) {
        CCutGraphHalfEdge* he = *heiter;

        CCutGraphVertex* v0 = m_pMesh->idVertex(he->source()->id());
        CCutGraphVertex* v1 = m_pMesh->idVertex(he->target()->id());
        CCutGraphVertex* v2 = m_pMesh->idVertex(he->he_prev()->source()->id()); // this is the opposite vertex
        CPoint proj0, proj1, proj2;
        embed(*v0, *v1, *v2, &proj0, &proj1, &proj2);

        proj1[2] += v1->height() - v0->height();
        proj2[2] += v2->height() - v0->height();

        // common edge is 01

        CPoint vecb0(0, 0, -1);

        he->vertAngle() = std::acos((vecb0 * proj1) / proj1.norm());

        CPoint alt2 = proj2 - proj1 * (proj1 * proj2) / (proj1.norm() * proj2.norm());
        CPoint altB = vecb0 - proj1 * std::cos(he->vertAngle());
        he->diAngle() = std::acos(alt2 * altB / (alt2.norm() * altB.norm()));
    }
}

void MeshLib::CCutGraph::compEdgePower() {
    for (CCutGraphMesh::MeshHalfEdgeIterator heiter(m_pMesh); !heiter.end(); ++heiter) {
        CCutGraphHalfEdge* he = *heiter;
        if (!he->source()->boundary() || !he->target()->boundary()) {
            float result = (std::cos(he->diAngle()) / std::sin(he->diAngle())) + (std::cos(m_pMesh->CCutGraphMesh::halfedgeSym(he)->diAngle()) / std::sin(m_pMesh->CCutGraphMesh::halfedgeSym(he)->diAngle()));
            result /= (he->length() * std::sin(he->vertAngle()) * std::sin(he->vertAngle()));
            he->power() = result;
        }
        else { he->power() = 0; };
    }
}

float MeshLib::CCutGraph::compTSC() {
    float result = 0;
    for (CCutGraphMesh::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter) {
        CCutGraphVertex* v = *viter;
        if (!v->boundary()) {
            result += v->height() * v->curvature();
        }
    }
    for (CCutGraphMesh::MeshHalfEdgeIterator heiter(m_pMesh); !heiter.end(); ++heiter) {
        CCutGraphHalfEdge* he = *heiter;
        if (he->source()->boundary() && he->target()->boundary()) {
            result += he->length() * (std::atan(1) * 4 - he->diAngle());
        }
        else {
            result += he->length() * (std::atan(1) * 8 - he->diAngle() - m_pMesh->halfedgeSym(he)->diAngle());
        }
    }

    return result;
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