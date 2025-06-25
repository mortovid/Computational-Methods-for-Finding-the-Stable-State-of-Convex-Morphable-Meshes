#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef MAC_OS
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif // MAC_OS

#include "viewer/Arcball.h" /*  Arc Ball  Interface  */
#include "CutGraphMesh.h"
#include "CutGraph.h"

#include "Eigen/Dense"

using namespace MeshLib;

/* window width and height */
int g_win_width, g_win_height;
int g_button;
int g_startx, g_starty;
int g_shade_flag = 0;


/* rotation quaternion and translation vector for the object */
CQrot g_obj_rot(0, 0, 1, 0);
CPoint g_obj_trans(0, 0, 0);

/* arcball object */
CArcball g_arcball;

/* global g_mesh */
CCutGraphMesh g_mesh;

/*! setup the object, transform from the world to the object coordinate system */
void setupObject(void)
{
    double rot[16];

    glTranslated(g_obj_trans[0], g_obj_trans[1], g_obj_trans[2]);
    g_obj_rot.convert(rot);
    glMultMatrixd((GLdouble*)rot);
}

/*! the eye is always fixed at world z = +5 */
void setupEye(void)
{
    glLoadIdentity();
    gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0);
}

/*! setup light */
void setupLight()
{
    GLfloat lightOnePosition[4] = { 0, 0, 1, 0 };
    GLfloat lightTwoPosition[4] = { 0, 0, -1, 0 };
    glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
    glLightfv(GL_LIGHT2, GL_POSITION, lightTwoPosition);
}

/*! draw g_mesh */
void drawMesh()
{
    glEnable(GL_LIGHTING);

    glLineWidth(1.0);
    glColor3f(229.0 / 255.0, 162.0 / 255.0, 141.0 / 255.0);
    for (CCutGraphMesh::MeshFaceIterator fiter(&g_mesh); !fiter.end(); ++fiter)
    {
        glBegin(GL_POLYGON);
        CCutGraphFace* pF = *fiter;
        for (CCutGraphMesh::FaceVertexIterator fviter(pF); !fviter.end(); ++fviter)
        {
            CCutGraphVertex* pV = *fviter;
            CPoint& p = pV->point();
            CPoint n;
            switch (g_shade_flag)
            {
            case 0:
                n = pF->normal();
                break;
            case 1:
                n = pV->normal();
                break;
            }
            glNormal3d(n[0], n[1], n[2]);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
    }
}

void drawSharpEdges()
{
    glDisable(GL_LIGHTING);

    glLineWidth(2.);
    glColor3f(1.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    for (CCutGraphMesh::MeshEdgeIterator eiter(&g_mesh); !eiter.end(); ++eiter)
    {
        CCutGraphEdge* pE = *eiter;
        if (pE->sharp() == true)
        {
            CCutGraphVertex* p0 = g_mesh.edgeVertex1(pE);
            CCutGraphVertex* p1 = g_mesh.edgeVertex2(pE);
            glVertex3f(p0->point()[0], p0->point()[1], p0->point()[2]);
            glVertex3f(p1->point()[0], p1->point()[1], p1->point()[2]);
        }
    }
    glEnd();
}

/*! display call back function
 */
void display()
{
    /* clear frame buffer */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    setupLight();
    /* transform from the eye coordinate system to the world system */
    setupEye();
    glPushMatrix();
    /* transform from the world to the ojbect coordinate system */
    setupObject();

    /* draw sharp edges */
    drawSharpEdges();
    /* draw the mesh */
    drawMesh();

    glPopMatrix();
    glutSwapBuffers();
}

/*! Called when a "resize" event is received by the window. */
void reshape(int w, int h)
{
    float ar;

    g_win_width = w;
    g_win_height = h;

    ar = (float)(w) / h;
    glViewport(0, 0, w, h); /* Set Viewport */
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(40.0, /* field of view in degrees */
        ar,   /* aspect ratio */
        0.1,  /* Z near */
        100.0 /* Z far */);

    glMatrixMode(GL_MODELVIEW);

    glutPostRedisplay();
}

/*! helper function to remind the user about commands, hot keys */
void help()
{
    printf("w  -  Wireframe Display\n");
    printf("f  -  Flat Shading \n");
    printf("s  -  Smooth Shading\n");
    printf("?  -  Help Information\n");
    printf("esc - quit\n");
}

/*! Keyboard call back function */
void keyBoard(unsigned char key, int x, int y)
{
    switch (key)
    {
    case 'f':
        // Flat Shading
        glPolygonMode(GL_FRONT, GL_FILL);
        g_shade_flag = 0;
        break;
    case 's':
        // Smooth Shading
        glPolygonMode(GL_FRONT, GL_FILL);
        g_shade_flag = 1;
        break;
    case 'w':
        // Wireframe mode
        glPolygonMode(GL_FRONT, GL_LINE);
        break;
    case '?':
        help();
        break;
    case 27:
        exit(0);
        break;
    }
    glutPostRedisplay();
}

/*! setup GL states */
void setupGLstate()
{
    GLfloat lightOneColor[] = { 1, 1, 1, 1.0 };
    GLfloat globalAmb[] = { .1, .1, .1, 1 };
    GLfloat lightOnePosition[] = { .0, 0.0, 1.0, 1.0 };
    GLfloat lightTwoPosition[] = { .0, 0.0, -1.0, 1.0 };

    glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.35, 0.53, 0.70, 0);
    glShadeModel(GL_SMOOTH);

    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHT2);
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);

    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightOneColor);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, lightOneColor);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globalAmb);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
    glLightfv(GL_LIGHT2, GL_POSITION, lightTwoPosition);

    const GLfloat specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    glLightfv(GL_LIGHT1, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 64.0f);

    GLfloat mat_ambient[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    GLfloat mat_diffuse[] = { 0.01f, 0.01f, 0.01f, 1.0f };
    GLfloat mat_specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    GLfloat mat_shininess[] = { 32 };

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
}

/*! mouse click call back function */
void mouseClick(int button, int state, int x, int y)
{
    /* set up an g_arcball around the Eye's center
    switch y coordinates to right handed system  */

    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        g_button = GLUT_LEFT_BUTTON;
        g_arcball = CArcball(g_win_width,
            g_win_height,
            x - g_win_width / 2,
            g_win_height - y - g_win_height / 2);
    }

    if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN)
    {
        g_startx = x;
        g_starty = y;
        g_button = GLUT_MIDDLE_BUTTON;
    }

    if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
    {
        g_startx = x;
        g_starty = y;
        g_button = GLUT_RIGHT_BUTTON;
    }
    return;
}

/*! mouse motion call back function */
void mouseMove(int x, int y)
{
    CPoint trans;
    CQrot rot;

    /* rotation, call g_arcball */
    if (g_button == GLUT_LEFT_BUTTON)
    {
        rot = g_arcball.update(x - g_win_width / 2, g_win_height - y - g_win_height / 2);
        g_obj_rot = rot * g_obj_rot;
        glutPostRedisplay();
    }

    /*xy translation */
    if (g_button == GLUT_MIDDLE_BUTTON)
    {
        double scale = 10. / g_win_height;
        trans = CPoint(scale * (x - g_startx), scale * (g_starty - y), 0);
        g_startx = x;
        g_starty = y;
        g_obj_trans = g_obj_trans + trans;
        glutPostRedisplay();
    }

    /* zoom in and out */
    if (g_button == GLUT_RIGHT_BUTTON)
    {
        double scale = 10. / g_win_height;
        trans = CPoint(0, 0, scale * (g_starty - y));
        g_startx = x;
        g_starty = y;
        g_obj_trans = g_obj_trans + trans;
        glutPostRedisplay();
    }
}

/*! Normalize g_mesh
 * \param pMesh the input g_mesh
 */
double revBase = 0;
void normalizeMesh(CCutGraphMesh* pMesh)
{
    CPoint s(0, 0, 0);
    for (CCutGraphMesh::MeshVertexIterator viter(pMesh); !viter.end(); ++viter)
    {
        CCutGraphVertex* v = *viter;
        s = s + v->point();
    }
    s = s / pMesh->numVertices();

    revBase -= s[2];
    for (CCutGraphMesh::MeshVertexIterator viter(pMesh); !viter.end(); ++viter)
    {
        CCutGraphVertex* v = *viter;
        CPoint p = v->point();
        p = p - s;
        v->point() = p;
    }

    double d = 0;
    for (CCutGraphMesh::MeshVertexIterator viter(pMesh); !viter.end(); ++viter)
    {
        CCutGraphVertex* v = *viter;
        CPoint p = v->point();
        for (int k = 0; k < 3; k++)
        {
            d = (d > fabs(p[k])) ? d : fabs(p[k]);
        }
    }

    revBase /= d;
    for (CCutGraphMesh::MeshVertexIterator viter(pMesh); !viter.end(); ++viter)
    {
        CCutGraphVertex* v = *viter;
        CPoint p = v->point();
        p = p / d;
        v->point() = p;
    }
};

/*! Compute the face normal and vertex normal
 * \param pMesh the input g_mesh
 */
void computeNormal(CCutGraphMesh* pMesh)
{
    for (CCutGraphMesh::MeshVertexIterator viter(pMesh); !viter.end(); ++viter)
    {
        CCutGraphVertex* v = *viter;
        CPoint n(0, 0, 0);
        for (CCutGraphMesh::VertexFaceIterator vfiter(v); !vfiter.end(); ++vfiter)
        {
            CCutGraphFace* pF = *vfiter;

            CPoint p[3];
            CHalfEdge* he = pF->halfedge();
            for (int k = 0; k < 3; k++)
            {
                p[k] = he->target()->point();
                he = he->he_next();
            }

            CPoint fn = (p[1] - p[0]) ^ (p[2] - p[0]);
            pF->normal() = fn / fn.norm();
            n += fn;
        }

        n = n / n.norm();
        v->normal() = n;
    }
};

void initOpenGL(int argc, char* argv[])
{
    /* glut stuff */
    glutInit(&argc, argv); /* Initialize GLUT */
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(600, 600);
    glutCreateWindow("Mesh Viewer"); /* Create window with given title */
    glViewport(0, 0, 600, 600);

    glutDisplayFunc(display); /* Set-up callback functions */
    glutReshapeFunc(reshape);
    glutMouseFunc(mouseClick);
    glutMotionFunc(mouseMove);
    glutKeyboardFunc(keyBoard);
    setupGLstate();

    glutMainLoop(); /* Start GLUT event-processing loop */
}

void cut_graph(CCutGraphMesh* pMesh)
{
    CCutGraph cg(pMesh);
    cg.cut_graph();
    std::cout << "Done with cutgraph. \n";
    cg.initGraph();
    std::cout << "Done with initializations. \n";
    cg.computeCurvature();
    std::cout << "Done with curvature. \n";
    cg.computeDihedralVertAngles();
    std::cout << "Done with dihedral angles. \n";
    cg.computeEdgePower();
    std::cout << "Done with edge powers. \n";
}

/*! Computes the gradient for the total scalar curvature */
Eigen::VectorXf computeGrad(CCutGraphMesh* p_mesh) {
    Eigen::VectorXf grad = Eigen::VectorXf::Zero(p_mesh->numVertices());

    for (CCutGraphMesh::MeshVertexIterator viter(p_mesh); !viter.end(); ++viter) {
        CCutGraphVertex* v = *viter;
        if (!v->boundary()) {
            grad(v->id() - 1) = v->curvature();
        }
    }
    return grad;
}

/*! Computes the Hessian matrix */
Eigen::MatrixXf computeHessian(CCutGraphMesh* p_mesh) { // REDO
    Eigen::MatrixXf hessian = Eigen::MatrixXf::Zero(p_mesh->numVertices(), p_mesh->numVertices());
    for (CCutGraphMesh::MeshVertexIterator viter1(p_mesh); !viter1.end(); ++viter1) {
        CCutGraphVertex* v1 = *viter1;

        for (CCutGraphMesh::VertexOutHalfedgeIterator vheiter(p_mesh, v1); !vheiter.end(); ++vheiter) {
            CCutGraphHalfEdge* he = *vheiter;
            CVertex* v2 = he->target();

            hessian(v1->id() - 1, v1->id() - 1) -= he->power();

            if (!v1->boundary() && !v2->boundary()) {
                hessian(v1->id() - 1, v2->id() - 1) = he->power();
            }
        }
    }
    return hessian;
}

float gradientDescent() { // output is TSC
    CCutGraph vc(&g_mesh);
    int numIters = 1;

    while (true) {
        Eigen::VectorXf grad = computeGrad(&g_mesh); // current gradient
        float TSC = vc.computeTSC();

        float max = 0;
        for (float i : grad) {
            max = std::max(max, std::abs(i));
        }
        if (max < 0.001) {
            std::cout << "max of " << max << " is small enough; exit \n";
            return TSC;
        }
        float stepSize = 0.001 / max;
        std::cout << "Iteration " << numIters << ": the max is " << max << " and the TSC is " << TSC << ". \n";

        for (CCutGraphMesh::MeshVertexIterator viter(&g_mesh); !viter.end(); ++viter) {
            CCutGraphVertex* v = *viter;
            v->height() += 0.01 * grad(v->id() - 1);
        }
        
        //  std::cout << grad << "\n";
        vc.computeCurvature();
        vc.computeDihedralVertAngles();
        numIters++;
        // if (numIters == 287) { return TSC; };
    }
}

float newtonMethod() { // output is TSC
    CCutGraph vc(&g_mesh);
    int numIters = 1;

    while (true) {
        Eigen::VectorXf grad = computeGrad(&g_mesh); // current gradient
        Eigen::MatrixXf hess = computeHessian(&g_mesh); // current Hessian matrix
        Eigen::VectorXf addToHeights = hess.llt().solve(grad);
        float TSC = vc.computeTSC();

        float max = 0;
        for (float i : addToHeights) {
            max = std::max(max, std::abs(i));
        }
        if (max < 0.001) {
            std::cout << "max of " << max << " is small enough; exit \n";
            return TSC;
        }
        float stepSize = 0.001 / max;

        std::cout << "Iteration " << numIters << ": the max is " << max << ", and the TSC is " << TSC << ". \n";

        for (CCutGraphMesh::MeshVertexIterator viter(&g_mesh); !viter.end(); ++viter) {
            CCutGraphVertex* v = *viter;
            v->height() += stepSize * addToHeights(v->id() - 1);
        }

        vc.computeCurvature();
        vc.computeDihedralVertAngles();
        vc.computeEdgePower();
        numIters++;
        // if (numIters == 1050) { return TSC; };
    }
}

/*! main function for viewer */
int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        printf("Usage: %s input.m\n", argv[0]);
        return EXIT_FAILURE;
    }

    std::string mesh_name(argv[1]);

    if (strutil::endsWith(mesh_name, ".m"))
    {
        g_mesh.read_m(mesh_name.c_str());
    }
    else
    {
        printf("Only file format .m supported.\n");
        return EXIT_FAILURE;
    }

    computeNormal(&g_mesh);
    cut_graph(&g_mesh);
    CCutGraph vc(&g_mesh);
    float TSC;

    std::cout << "Input G for Gradient Descent and N for Newton's Method: ";
    char method;
    std::cin >> method;
    if (method == 'G') {
        TSC = gradientDescent();
    }
    else {
        TSC = newtonMethod();
    }

    std::cout << "The final TSC is " << TSC << ". \n";
    std::cout << "The final heights are: \n ( ";
    for (CCutGraphMesh::MeshVertexIterator viter(&g_mesh); !viter.end(); ++viter) {
        CCutGraphVertex* v = *viter;
        if (!v->boundary()) {
            std::cout << v->height() << " ";
        }
    }
    std::cout << ") \n";

    // now do breadth-first to assemble mesh

    CCutGraphFace* fInit = g_mesh.faces().front();
    CCutGraphVertex* vOrigin1;
    CCutGraphVertex* vOrigin2;
    CCutGraphVertex* vOrigin3;
    int counter = 0;
    for (CCutGraphMesh::FaceVertexIterator fviter(fInit); !fviter.end(); ++fviter) {
        if (counter == 0) { vOrigin1 = *fviter; };
        if (counter == 1) { vOrigin2 = *fviter; };
        if (counter == 2) { vOrigin3 = *fviter; };
        counter++;
    }
    vOrigin1->embPoint() = { 0, 0, 0 };
    vOrigin1->embedded() = true;

    float projInit12 = sqrt(pow((vOrigin2->point() - vOrigin1->point()).norm(), 2) - pow(vOrigin2->height() - vOrigin1->height(), 2));
    vOrigin2->embPoint() = { projInit12, 0, 0 };
    vOrigin2->embedded() = true;

    float projInit13 = sqrt(pow((vOrigin3->point() - vOrigin1->point()).norm(), 2) - pow(vOrigin3->height() - vOrigin1->height(), 2));
    float projInit23 = sqrt(pow((vOrigin2->point() - vOrigin3->point()).norm(), 2) - pow(vOrigin2->height() - vOrigin3->height(), 2));
    float angInit = acos((pow(projInit12, 2) + pow(projInit13, 2) - pow(projInit23, 2)) / (2 * projInit12 * projInit13));
    vOrigin3->embPoint() = { projInit13 * cos(angInit), projInit13 * sin(angInit), 0 };
    vOrigin3->embedded() = true;

    std::list<CCutGraphVertex*> toEmbed;
    for (CCutGraphMesh::VertexVertexIterator vviter(vOrigin1); !vviter.end(); ++vviter) {
        toEmbed.push_back(*vviter);
    }
    for (CCutGraphMesh::VertexVertexIterator vviter(vOrigin2); !vviter.end(); ++vviter) {
        toEmbed.push_back(*vviter);
    }
    for (CCutGraphMesh::VertexVertexIterator vviter(vOrigin3); !vviter.end(); ++vviter) {
        toEmbed.push_back(*vviter);
    }

    while (toEmbed.size() > 0) {
        // pop first element in list
        CCutGraphVertex* vNext = toEmbed.front();
        toEmbed.pop_front();
        // if first element is not embedded:
        if (!vNext->embedded()) {
            // if a face with two adjacent vertices is connected
            for (CCutGraphMesh::VertexFaceIterator vfiter(vNext); !vfiter.end(); ++vfiter) {
                CCutGraphFace* f = *vfiter;
                int embCounter = 0;
                for (CCutGraphMesh::FaceVertexIterator fviter(f); !fviter.end(); ++fviter) {
                    CCutGraphVertex* vTemp = *fviter;
                    if (vTemp->embedded()) {
                        embCounter++;
                    }
                }

                if (embCounter == 2) {
                    // embed the vertex
                    CCutGraphVertex* v2;
                    CCutGraphVertex* v3;
                    bool v2ac = false;

                    for (CCutGraphMesh::FaceVertexIterator fviter(f); !fviter.end(); ++fviter) {
                        CCutGraphVertex* vTemp = *fviter;
                        if (vTemp->id() != vNext->id()) {
                            if (v2ac) {
                                v3 = vTemp;
                                break;
                            }
                            else {
                                v2 = vTemp;
                                v2ac = true;
                            }
                        }
                    }

                    float proj12 = sqrt(pow((vNext->point() - v2->point()).norm(), 2) - pow(vNext->height() - v2->height(), 2));
                    float proj13 = sqrt(pow((vNext->point() - v3->point()).norm(), 2) - pow(vNext->height() - v3->height(), 2));
                    float proj23 = sqrt(pow((v2->point() - v3->point()).norm(), 2) - pow(v2->height() - v3->height(), 2));

                    float ang23 = atan2(v3->embPoint()[1] - v2->embPoint()[1], v3->embPoint()[0] - v2->embPoint()[0]);
                    float ang123 = acos((proj12 * proj12 + proj23 * proj23 - proj13 * proj13) / (2 * proj12 * proj23));
                    CPoint emb1 = v2->embPoint() + CPoint{ proj12 * cos(ang23 + ang123), proj12 * sin(ang23 + ang123), 0 };
                    CPoint emb2 = v2->embPoint() + CPoint{ proj12 * cos(ang23 - ang123), proj12 * sin(ang23 - ang123), 0 };


                    CCutGraphEdge* embE;
                    for (CCutGraphMesh::FaceEdgeIterator feiter(f); !feiter.end(); ++feiter) {
                        CCutGraphEdge* tempE = *feiter;
                        if (!tempE->boundary()) {
                            if (tempE->halfedge(0)->source()->id() == v2->id() && tempE->halfedge(0)->target()->id() == v3->id()) { embE = tempE; };
                            if (tempE->halfedge(1)->source()->id() == v2->id() && tempE->halfedge(1)->target()->id() == v3->id()) { embE = tempE; };
                        }
                    }
                    CCutGraphFace* oppF;
                    if (g_mesh.CCutGraphMesh::edgeFace1(embE)->id() == f->id()) {
                        oppF = g_mesh.CCutGraphMesh::edgeFace2(embE);
                    }
                    else {
                        oppF = g_mesh.CCutGraphMesh::edgeFace1(embE);
                    }
                    CCutGraphVertex* oppV;
                    for (CCutGraphMesh::FaceVertexIterator fviter(oppF); !fviter.end(); ++fviter) {
                        CCutGraphVertex* tempV = *fviter;
                        if (tempV->id() != v2->id() && tempV->id() != v3->id()) {
                            oppV = tempV;
                        }
                    }

                    float slope = (v3->embPoint()[1] - v2->embPoint()[1]) / (v3->embPoint()[0] - v2->embPoint()[0]);
                    float constant = v2->embPoint()[1] - v2->embPoint()[0] * slope;

                    // figure out which embPoint to use

                    bool oppAboveLine = oppV->embPoint()[1] - (oppV->embPoint()[0] * slope) > constant;
                    bool emb1AboveLine = emb1[1] - (emb1[0] * slope) > constant;

                    if (oppAboveLine && emb1AboveLine) { vNext->embPoint() = emb2; }
                    else if (oppAboveLine && !emb1AboveLine) { vNext->embPoint() = emb1; }
                    else if (!oppAboveLine && emb1AboveLine) { vNext->embPoint() = emb1; }
                    else { vNext->embPoint() = emb2; };

                    // mark it as embedded
                    vNext->embedded() = true;
                    // add adjacent vertices to the list
                    for (CCutGraphMesh::VertexVertexIterator vviter(vNext); !vviter.end(); ++vviter) {
                        toEmbed.push_back(*vviter);
                    }

                    break;
                }
            }
            if (!vNext->embedded()) {
                toEmbed.push_back(vNext);
            }
        }
    }

    // add the heights as z-coords

    for (CCutGraphMesh::MeshVertexIterator viter(&g_mesh); !viter.end(); ++viter) {
        CCutGraphVertex* v = *viter;
        v->embPoint()[2] = v->height();
    }

    for (CCutGraphMesh::MeshVertexIterator viter(&g_mesh); !viter.end(); ++viter) {
        CCutGraphVertex* v = *viter;
        v->point() = v->embPoint();
    }

    std::ofstream output;
    output.open("C:/projects/2022_CCG_Assignment_1_skeleton/data/output.obj", std::ofstream::out | std::ofstream::trunc);
    for (CCutGraphMesh::MeshVertexIterator viter(&g_mesh); !viter.end(); ++viter) {
        CCutGraphVertex* v = *viter;
        output << "v " << "  " << v->point()[0] << " " << v->point()[1] << " " << v->point()[2] << ". \n";
    }
    output << "\n";
    for (CCutGraphMesh::MeshFaceIterator fiter(&g_mesh); !fiter.end(); ++fiter) {
        CCutGraphFace* f = *fiter;
        CVertex* v1 = f->halfedge()->source();
        CVertex* v2 = f->halfedge()->he_prev()->source();
        CVertex* v3 = f->halfedge()->he_next()->source();
        output << "f " << "  " << v1->id() << " " << v2->id() << " " << v3->id() << ". \n";
    }
    output.close();

    initOpenGL(argc, argv);

    return EXIT_SUCCESS;
}