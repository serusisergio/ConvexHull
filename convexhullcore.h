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
#include <GUI/ConvexHullCore/conflictgraph.h>


class ConvexHullCore{
public:
    //method
    ConvexHullCore(DrawableDcel *dcel,MainWindow* mainWindow, bool isClicked);
    void findConvexHull();
    
private:
    //method
    bool verifyEuleroProperty() const;
    void getVertexs();
    void executePermutation();
    bool isCoplanar() const;
    void setTetrahedron();
    std::list<Dcel::HalfEdge*> getHorizon(std::set<Dcel::Face*>* facesVisibleByVertex) const;
    void removeFacesVisibleByVertex(std::set<Dcel::Face*>* facesVisibleByVertex);
    std::vector<Dcel::Face*> createNewFaces(std::list<Dcel::HalfEdge*> horizon, Dcel::Vertex*);
    bool isNormalFaceTurnedTowardsThePoint() const;

    //variable
    DrawableDcel* dcel;
    MainWindow* mainWindow;
    int numberVertex;
    std::vector<Dcel::Vertex*> vertexS;
    const bool isClicked;

};

#endif // CONVEXHULLCORE_H
