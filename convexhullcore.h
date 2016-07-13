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
#include <iostream>

using namespace Eigen;
using namespace std;

class ConvexHullCore
{
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

    //variable
    DrawableDcel* dcel;
    
};

#endif // CONVEXHULLCORE_H
