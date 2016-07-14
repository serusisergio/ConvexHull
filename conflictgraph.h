#ifndef CONFLICTGRAPH_H
#define CONFLICTGRAPH_H

#include <eigen3/Eigen/Dense>
#include "GUI/managers/dcelmanager.h"
#include "lib/dcel/drawable_dcel.h"

using namespace Eigen;
using namespace std;

class ConflictGraph
{
public:
    //metodi
    ConflictGraph(DrawableDcel* dcel, std::vector<Dcel::Vertex*> &vertexS);
    void initializeCG();
    std::list<Dcel::Face*> getFacesVisibleByVertex(Dcel::Vertex* vertex);
    std::list<Dcel::Vertex*> getVertexVisibleByFace(Dcel::Face* face);

    //Oggetti-Variabili passati da convexhull core
    DrawableDcel* dcel;
    std::vector<Dcel::Vertex*> vertexS;

    //OggettiVariali creati in questa classe
    //bisogna usare una lista, vista l'esigenza di eliminare ed aggiornare spesso è la soluzione migliore, il vettore è poco dinamico, per eliminare un oggetto bisogna
    //successivmente risistemare tutti gli elementi del vettore. Verrà usata la lista, come consigliato a lezione e tutoraggio.
    std::map<Dcel::Face*, std::list<Dcel::Vertex*>> f_conflict;
    std::map<Dcel::Vertex*, std::list<Dcel::Face*>> v_conflict;

private:
    int numberVertex=0;
    void addFaceToVertex(Dcel::Face* face, Dcel::Vertex* vertex);
    void addVertexToFace(Dcel::Vertex* vertex, Dcel::Face* face);
    void deleteFaceFromVertex(Dcel::Face* face, Dcel::Vertex* vertex);
    void deleteVertexFromFace(Dcel::Vertex* vertex, Dcel::Face* face);
};

#endif // CONFLICTGRAPH_H
