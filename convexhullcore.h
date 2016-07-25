#ifndef CONVEXHULLCORE_H
#define CONVEXHULLCORE_H


#include "GUI/managers/convexhullmanager.h"
#include "ui_convexhullmanager.h"
#include <QFrame>
#include <iomanip>
#include "lib/dcel/drawable_dcel.h"
#include "GUI/mainwindow.h"
#include "common.h"
#include "GUI/managers/dcelmanager.h"
#include "lib/common/timer.h"
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <GUI/conflictgraph.h>


class ConvexHullCore{
public:
    //method
    ConvexHullCore(DrawableDcel *dcel);
    void findConvexHull();
    
    //variable
    std::vector<Dcel::Vertex*> vertexS;

private:
    //method
    bool verifyEuleroProperty();
    void getVertexs();
    void executePermutation();
    bool isCoplanar();
    void setTetrahedron();
    std::list<Dcel::HalfEdge*> getHorizon(std::set<Dcel::Face*>* facesVisibleByVertex);
    void removeFacesVisibleByVertex(std::set<Dcel::Face*>* facesVisibleByVertex);
    std::vector<Dcel::Face*> createNewFaces(std::list<Dcel::HalfEdge*> horizon, Dcel::Vertex*);
    bool isNormalFaceTurnedTowardsThePoint();

    //variable
    DrawableDcel* dcel;
    int numberVertex=0;
    
};

#endif // CONVEXHULLCORE_H
