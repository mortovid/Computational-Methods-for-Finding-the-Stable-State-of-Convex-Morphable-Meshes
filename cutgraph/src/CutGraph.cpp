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
    
        // Mark all touched flags false, and select a face as the starting point
        CCutGraphFace* pHeadFace = NULL;
        for (CCutGraphMesh::MeshFaceIterator fiter(m_pMesh); !fiter.end(); ++fiter)
        {
            CCutGraphFace* pF = *fiter;
            pF->touched() = false; // reset touched flag
            pHeadFace = pF;        // choose a face (here, the last face encountered)
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
                        // --- Begin inserted code ---
                        // Mark the neighbor face as visited and add it to the queue.
                        pSymFace->touched() = true;
                        fQueue.push(pSymFace);
    
                        // Find the common (primal) edge shared by the current face and its neighbor.
                        // We assume that the edge corresponding to the halfedge pH is the one connecting its source and target.
                        CCutGraphVertex* v0 = static_cast<CCutGraphVertex*>(pH->source());
                        CCutGraphVertex* v1 = static_cast<CCutGraphVertex*>(pH->target());

                        CCutGraphEdge* pEdge = NULL;
                        for (CCutGraphMesh::VertexEdgeIterator veiter(v0); !veiter.end(); ++veiter)
                        {
                            CCutGraphEdge* candidate = *veiter;
                            if ((m_pMesh->edgeVertex1(candidate) == v0 && m_pMesh->edgeVertex2(candidate) == v1) ||
                                (m_pMesh->edgeVertex1(candidate) == v1 && m_pMesh->edgeVertex2(candidate) == v0))
                            {
                                pEdge = candidate;
                                break;
                            }
                        }
                        // If the common edge is found, mark it as part of the spanning tree.
                        if (pEdge != NULL)
                            pEdge->sharp() = true;
                        // --- End inserted code ---
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
    // A queue used to store vertices with valence-1.
    std::queue<CCutGraphVertex*> vQueue;

    // 1. Compute the valence of each vertex, and record all valence-1 vertices.
    for (CCutGraphMesh::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter)
    {
        CCutGraphVertex* pV = *viter;
        pV->valence() = 0; // Reset the vertex valence.
        
        for (CCutGraphMesh::VertexEdgeIterator veiter(pV); !veiter.end(); ++veiter)
        {
            CCutGraphEdge* pE = *veiter;
            // Count only edges that are part of the cut (sharp edges).
            if (pE->sharp()) {
                pV->valence()++;
            }
        }

        // If the vertex has exactly one sharp edge, add it to the queue.
        if (pV->valence() == 1)
            vQueue.push(pV);
    }

    // 2. Remove the segments attached to valence-1 vertices.
    while (!vQueue.empty())
    {
        CCutGraphVertex* pV = vQueue.front();
        vQueue.pop();

        // For each edge incident to this vertex
        for (CCutGraphMesh::VertexEdgeIterator veiter(pV); !veiter.end(); ++veiter)
        {
            CCutGraphEdge* pE = *veiter;
            // Determine the adjacent vertex.
            CCutGraphVertex* pW = (m_pMesh->edgeVertex1(pE) == pV) ?
                m_pMesh->edgeVertex2(pE) : m_pMesh->edgeVertex1(pE);

            // If the edge is part of the cut graph
            if (pE->sharp())
            {
                // Remove the segment.
                pE->sharp() = false;
                // Decrement the valence of the neighboring vertex.
                pW->valence()--;
                // If the neighbor now has a valence of 1, add it to the queue.
                if (pW->valence() == 1)
                    vQueue.push(pW);
                break; // Exit after processing the unique incident sharp edge.
            }
        }
    }
}

template <typename T>
T clamp(T x, T lower, T upper) {
    return std::max(lower, std::min(x, upper));
}

// Robust embed function
void MeshLib::embed(CCutGraphVertex v0, CCutGraphVertex v1, CCutGraphVertex v2, CPoint *proj0, CPoint *proj1, CPoint *proj2) {
    float d01 = (v0.point() - v1.point()).norm();
    float d02 = (v0.point() - v2.point()).norm();
    float d12 = (v1.point() - v2.point()).norm();
    float h01 = v0.height() - v1.height();
    float h02 = v0.height() - v2.height();
    float h12 = v1.height() - v2.height();

    float proj01_sq = d01 * d01 - h01 * h01;
    float proj02_sq = d02 * d02 - h02 * h02;
    float proj12_sq = d12 * d12 - h12 * h12;

    if (proj01_sq <= 0 || proj02_sq <= 0 || proj12_sq <= 0) {
        throw std::runtime_error("embed(): Negative projection length squared - degenerate triangle.");
    }

    float proj01 = std::sqrt(proj01_sq);
    float proj02 = std::sqrt(proj02_sq);
    float proj12 = std::sqrt(proj12_sq);

    float cos_angle = (proj01_sq + proj02_sq - proj12_sq) / (2.0f * proj01 * proj02);
    cos_angle = clamp(cos_angle, -1.0f, 1.0f);
    float angle = std::acos(cos_angle);

    *proj0 = MeshLib::CPoint(0, 0, 0);
    *proj1 = MeshLib::CPoint(proj01, 0, 0);
    *proj2 = MeshLib::CPoint(proj02 * std::cos(angle), proj02 * std::sin(angle), 0);
}

// Safe acos helper
inline float safe_acos(float x) {
    return std::acos(clamp(x, -1.0f, 1.0f));
}

void MeshLib::CCutGraph::compCurvature() {
    constexpr float two_pi = 2.0f * 3.14159265358979323846f;
    for (CCutGraphMesh::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter) {
        CCutGraphVertex* v0 = *viter;
        if (!v0->boundary()) {
            float result = 0.0f;
            for (CCutGraphMesh::VertexOutHalfedgeIterator vheiter(m_pMesh, v0); !vheiter.end(); ++vheiter) {
                CCutGraphHalfEdge* he = *vheiter;
                CCutGraphVertex* v1 = m_pMesh->idVertex(he->target()->id());
                CCutGraphVertex* v2 = m_pMesh->idVertex(he->he_prev()->source()->id());
                CPoint proj0, proj1, proj2;
                try {
                    embed(*v0, *v1, *v2, &proj0, &proj1, &proj2);
                } catch (...) { continue; }
                float l01 = (proj0 - proj1).norm();
                float l02 = (proj0 - proj2).norm();
                float l12 = (proj1 - proj2).norm();
                if (l01 * l02 < 1e-6f) continue;
                float cos_angle = (l01*l01 + l02*l02 - l12*l12) / (2.0f * l01 * l02);
                float angle = safe_acos(cos_angle);
                if (!std::isfinite(angle)) continue;
                result += angle;
            }
            float curv = two_pi - result;
            v0->curvature() = std::max(curv, 0.0f);
        } else {
            v0->curvature() = -1;
        }
    }
}

void MeshLib::CCutGraph::compDihedralVertAngles() {
    for (CCutGraphMesh::MeshHalfEdgeIterator heiter(m_pMesh); !heiter.end(); ++heiter) {
        CCutGraphHalfEdge* he = *heiter;
        CCutGraphVertex* v0 = m_pMesh->idVertex(he->source()->id());
        CCutGraphVertex* v1 = m_pMesh->idVertex(he->target()->id());
        CCutGraphVertex* v2 = m_pMesh->idVertex(he->he_prev()->source()->id());
        CPoint proj0, proj1, proj2;
        try {
            embed(*v0, *v1, *v2, &proj0, &proj1, &proj2);
        } catch (...) { continue; }

        proj1[2] += v1->height() - v0->height();
        proj2[2] += v2->height() - v0->height();
        CPoint vecb0(0, 0, -1);
        float n1 = proj1.norm();
        if (n1 < 1e-6f) continue;
        float dot = (vecb0 * proj1) / n1;
        he->vertAngle() = safe_acos(dot);

        CPoint alt2 = proj2 - proj1 * ((proj1 * proj2) / (proj1.norm() * proj2.norm()));
        CPoint altB = vecb0 - proj1 * std::cos(he->vertAngle());
        float n_alt2 = alt2.norm(), n_altB = altB.norm();
        if (n_alt2 < 1e-6f || n_altB < 1e-6f) continue;
        he->diAngle() = safe_acos((alt2 * altB) / (n_alt2 * n_altB));
    }
}

void MeshLib::CCutGraph::compEdgePower() {
    for (CCutGraphMesh::MeshHalfEdgeIterator heiter(m_pMesh); !heiter.end(); ++heiter) {
        CCutGraphHalfEdge* he = *heiter;
        if (!he->source()->boundary() || !he->target()->boundary()) {
            CCutGraphHalfEdge* sym_he = m_pMesh->CCutGraphMesh::halfedgeSym(he);
            if (!sym_he) { he->power() = 0.0f; continue; }

            float a1 = he->diAngle();
            float a2 = sym_he->diAngle();
            float s1 = std::sin(a1), s2 = std::sin(a2);
            if (std::abs(s1) < 1e-4f || std::abs(s2) < 1e-4f) {
                he->power() = 0.0f;
                continue;
            }

            float cot1 = std::cos(a1) / s1;
            float cot2 = std::cos(a2) / s2;
            float vert_angle = he->vertAngle();
            float sin_vert = std::sin(vert_angle);
            float denom = he->length() * sin_vert * sin_vert;
            if (denom < 1e-6f) {
                he->power() = 0.0f;
                continue;
            }

            he->power() = (cot1 + cot2) / denom;
        } else {
            he->power() = 0.0f;
        }
    }
}



//     void MeshLib::embed(CCutGraphVertex v0, CCutGraphVertex v1, CCutGraphVertex v2, CPoint *proj0, CPoint *proj1, CPoint *proj2) {
//         float proj01 = sqrt(pow((v0.point() - v1.point()).norm(), 2) - pow(v0.height() - v1.height(), 2));
//         float proj02 = sqrt(pow((v0.point() - v2.point()).norm(), 2) - pow(v0.height() - v2.height(), 2));
//         float proj12 = sqrt(pow((v1.point() - v2.point()).norm(), 2) - pow(v1.height() - v2.height(), 2));
//         float angle = acos((proj01 * proj01 + proj02 * proj02 - proj12 * proj12) / ((float)2 * proj01 * proj02));

//         if (proj12 != proj12) { // debugging stuff
//             std::cout << "proj error -- please exit \n";
//             std::cout << proj01 << ", " << proj02 << ", " << proj12 << " are the sides \n";
//             std::cout << pow((v1.point() - v2.point()).norm(), 1) << " for norm, " << pow(v1.height() - v2.height(), 1) << " for height, "
//                 << pow((v1.point() - v2.point()).norm(), 1) - pow(v1.height() - v2.height(), 1) << " for full \n";
//             std::cout << (v0.point() - v1.point()).norm() << ", " << v0.height() - v1.height() << ". \n";
//             std::string temp;
//             std::cin >> temp;
//         }
//         else if (angle != angle) {
//             std::cout << "angle error -- please exit \n";
//             std::cout << proj01 << ", " << proj02 << ", " << proj12 << " are the sides \n";
//             std::string temp;
//             std::cin >> temp;
//         }

//         *proj0 = MeshLib::CPoint(0, 0, 0);
//         *proj1 = MeshLib::CPoint(proj01, 0, 0);
//         *proj2 = MeshLib::CPoint(proj02 * cos(angle), proj02 * sin(angle), 0);
//     }


    
// void MeshLib::CCutGraph::compCurvature() {
//     // Use an explicit constant for 2*pi for clarity.
//     constexpr float two_pi = 2.0f * 3.14159265358979323846f;
    
//     for (CCutGraphMesh::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter) {
//         CCutGraphVertex* v0 = *viter;

//         if (!v0->boundary()) {
//             float result = 0.0f;

//             for (CCutGraphMesh::VertexOutHalfedgeIterator vheiter(m_pMesh, v0); !vheiter.end(); ++vheiter) {
//                 CCutGraphHalfEdge* he = *vheiter;

//                 CCutGraphVertex* v1 = m_pMesh->idVertex(he->target()->id());
//                 CCutGraphVertex* v2 = m_pMesh->idVertex(he->he_prev()->source()->id());

//                 CPoint proj0, proj1, proj2;
//                 // Embed the vertices. If embed can fail, consider adding error checking here.
//                 embed(*v0, *v1, *v2, &proj0, &proj1, &proj2);

//                 float l01 = (proj0 - proj1).norm();
//                 float l02 = (proj0 - proj2).norm();
//                 float l12 = (proj1 - proj2).norm();

//                 // Prevent division by zero (or near-zero) issues:
//                 if (l01 * l02 < std::numeric_limits<float>::epsilon()) {
//                     continue;
//                 }

//                 // Compute the cosine of the angle using the cosine law.
//                 float cos_angle = (l01 * l01 + l02 * l02 - l12 * l12) / (2.0f * l01 * l02);
//                 // Clamp the value to the valid domain for acos to avoid NaN results.
//                 cos_angle = std::max(-1.0f, std::min(1.0f, cos_angle));
//                 float angle = std::acos(cos_angle);

//                 result += angle;
//             }

//             // Compute the Gaussian curvature as 2*pi minus the summed angles.
//             v0->curvature() = two_pi - result;
//             if (v0->curvature() < -0.0001f) {
//                 std::stringstream ss;
//                 ss << "\nNegative curvature encountered: " << v0->curvature() << "\n";
//                 ss << "Vertex ID " << v0->id() << ": (" 
//                    << v0->point()(0) << ", " 
//                    << v0->point()(1) << ", " 
//                    << v0->point()(2) << "), Height: " 
//                    << v0->height() << "\n\n";
//                 for (CCutGraphMesh::VertexVertexIterator vviter(v0); !vviter.end(); ++vviter) {
//                     CCutGraphVertex* v = *vviter;
//                     ss << "Neighbor Vertex ID " << v->id() << ": (" 
//                        << v->point()(0) << ", " 
//                        << v->point()(1) << ", " 
//                        << v->point()(2) << "), Height: " 
//                        << v->height() << "\n";
//                 }
//                 // Throwing an exception is preferable over an abrupt exit.
//                 throw std::runtime_error(ss.str());
//             }
//         }
//         else {
//             v0->curvature() = -1;
//         }
//     }
// }

//     void MeshLib::CCutGraph::compDihedralVertAngles() { // REDO
//         for (CCutGraphMesh::MeshHalfEdgeIterator heiter(m_pMesh); !heiter.end(); ++heiter) {
//             CCutGraphHalfEdge* he = *heiter;

//             CCutGraphVertex* v0 = m_pMesh->idVertex(he->source()->id());
//             CCutGraphVertex* v1 = m_pMesh->idVertex(he->target()->id());
//             CCutGraphVertex* v2 = m_pMesh->idVertex(he->he_prev()->source()->id()); // this is the opposite vertex
//             CPoint proj0, proj1, proj2;
//             embed(*v0, *v1, *v2, &proj0, &proj1, &proj2);

//             proj1[2] += v1->height() - v0->height();
//             proj2[2] += v2->height() - v0->height();

//             // common edge is 01

//             CPoint vecb0(0, 0, -1);

//             he->vertAngle() = std::acos((vecb0 * proj1) / proj1.norm());

//             CPoint alt2 = proj2 - proj1 * (proj1 * proj2) / (proj1.norm() * proj2.norm());
//             CPoint altB = vecb0 - proj1 * std::cos(he->vertAngle());
//             he->diAngle() = std::acos(alt2 * altB / (alt2.norm() * altB.norm()));
//         }
//     }

//     void MeshLib::CCutGraph::compEdgePower() {
//         for (CCutGraphMesh::MeshHalfEdgeIterator heiter(m_pMesh); !heiter.end(); ++heiter) {
//             CCutGraphHalfEdge* he = *heiter;
    
//             // Check if either endpoint is on the boundary
//             if (!he->source()->boundary() || !he->target()->boundary()) {
//                 CCutGraphHalfEdge* sym_he = m_pMesh->CCutGraphMesh::halfedgeSym(he);
    
//                 // Dihedral angle-based weight
//                 float angle = he->diAngle();
//                 float sym_angle = sym_he->diAngle();
//                 float cot1 = std::cos(angle) / std::sin(angle);
//                 float cot2 = std::cos(sym_angle) / std::sin(sym_angle);
    
//                 // Vertex angle for scaling
//                 float vert_angle = he->vertAngle();
//                 float sin_vert = std::sin(vert_angle);
    
//                 // Final edge power calculation
//                 float result = (cot1 + cot2) / (he->length() * sin_vert * sin_vert);
//                 he->power() = result;
//             } else {
//                 he->power() = 0.0f;
//             }
//         }
//     }
    

    float MeshLib::CCutGraph::compTSC() {
        float result = 0.0f;
    
        // === Vertex contribution ===
        for (CCutGraphMesh::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter) {
            CCutGraphVertex* v = *viter;
    
            // Only include interior vertices
            if (!v->boundary()) {
                result += v->height() * v->curvature();  // likely energy from scalar field (e.g. conformal factor)
            }
        }
    
        // === Edge (half-edge) contribution ===
        const float PI = std::atan(1.0f) * 4.0f;
    
        for (CCutGraphMesh::MeshHalfEdgeIterator heiter(m_pMesh); !heiter.end(); ++heiter) {
            CCutGraphHalfEdge* he = *heiter;
            float length = he->length();
            float angle1 = he->diAngle();
    
            if (he->source()->boundary() && he->target()->boundary()) {
                // Boundary edge (both ends on boundary): contribution from single face
                result += length * (PI - angle1);
            } else {
                // Interior edge: contribution from both adjacent faces
                CCutGraphHalfEdge* sym_he = m_pMesh->halfedgeSym(he);
                float angle2 = sym_he ? sym_he->diAngle() : 0.0f;
    
                result += length * (2 * PI - angle1 - angle2);
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
