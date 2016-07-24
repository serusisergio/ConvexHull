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
    bool isVisible(Dcel::Vertex* vertex,Dcel::Face* face);
    std::set<Dcel::Face*>* getFacesVisibleByVertex(Dcel::Vertex* vertex);
    std::set<Dcel::Vertex*>* getVertexVisibleByFace(Dcel::Face* face);
    void getPossibleVertex(unsigned int point_i);



    //Oggetti-Variabili passati da convexhull core
    DrawableDcel* dcel;
    std::vector<Dcel::Vertex*> vertexS;

    //OggettiVariali creati in questa classe
    //bisogna usare una lista, vista l'esigenza di eliminare ed aggiornare spesso è la soluzione migliore, il vettore è poco dinamico, per eliminare un oggetto bisogna
    //successivmente risistemare tutti gli elementi del vettore. Verrà usata la lista, come consigliato a lezione e tutoraggio.
    std::map<Dcel::Face*, std::set<Dcel::Vertex*>*> f_conflict;//Modificato, notata la lentezza ora uso un puntatore alla lista di puntatori
    std::map<Dcel::Vertex*, std::set<Dcel::Face*>*> v_conflict;
    void deleteVertexFromFace(Dcel::Vertex* vertex);
    void deleteFaceAndVertex(std::set<Dcel::Face*>* faces, Dcel::Vertex* vertex);
    void deleteFaceFromVertex(std::set<Dcel::Face*>* faces);
    void updateCG(Dcel::Face* face);


private:
    int numberVertex=0;
    void addFaceToVertex(Dcel::Face* face, Dcel::Vertex* vertex);
    void addVertexToFace(Dcel::Vertex* vertex, Dcel::Face* face);

};

#endif // CONFLICTGRAPH_H
