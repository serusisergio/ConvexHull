#ifndef CONFLICTGRAPH_H
#define CONFLICTGRAPH_H

#include <eigen3/Eigen/Dense>
#include "GUI/managers/dcelmanager.h"
#include "lib/dcel/drawable_dcel.h"



class ConflictGraph{

public:
    //metodi
    ConflictGraph(DrawableDcel* dcel,const std::vector<Dcel::Vertex*> &vertexS);
    void initializeCG();
    bool isVisible(Dcel::Vertex* vertex,Dcel::Face* face) const;
    std::set<Dcel::Face*>* getFacesVisibleByVertex(Dcel::Vertex* vertex) const;
    std::set<Dcel::Vertex*>* getVertexVisibleByFace(Dcel::Face* face) const;
    void deleteVertex(Dcel::Vertex* vertex);
    void deleteFaces(std::set<Dcel::Face*>* faces);
    void updateCG(Dcel::Face* faceToUpdate, std::set<Dcel::Vertex*>* setVertexForFace);
    std::map<Dcel::HalfEdge*, std::set<Dcel::Vertex*>*> getVertexMapToControlForTheNewFace(std::list<Dcel::HalfEdge*> horizon) const;



    //OggettiVariali creati in questa classe
    //HO usato una mappa di set, vista l'esigenza di eliminare ed aggiornare spesso è la soluzione migliore, il vettore è poco dinamico, per eliminare un oggetto bisogna
    //successivmente risistemare tutti gli elementi del vettore.
    std::map<Dcel::Face*, std::set<Dcel::Vertex*>*> v_conflict;
    std::map<Dcel::Vertex*, std::set<Dcel::Face*>*> f_conflict;



private:
    int numberVertex;

    //Oggetti-Variabili passati da convexhull core
    DrawableDcel* dcel;
    std::vector<Dcel::Vertex*> vertexS;


    void addFaceToVertex(Dcel::Face* face, Dcel::Vertex* vertex);
    void addVertexToFace(Dcel::Vertex* vertex, Dcel::Face* face);

};

#endif // CONFLICTGRAPH_H
