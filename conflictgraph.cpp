#include "conflictgraph.h"

/**
 * @brief ConvexHullCore::ConflictGraph(DrawableDcel *dcel, std::vector<Dcel::Vertex *> vertexS)
 * This method is the constructor of the ConflictGraph class, it receive the pointer of the dcel
 * and the points.
 */
ConflictGraph::ConflictGraph(DrawableDcel *dcel, std::vector<Dcel::Vertex*> &vertexS){
    this->dcel = dcel;
    this->vertexS = vertexS;
    this->numberVertex = this->vertexS.size();
}

/**
 * @brief ConvexHullCore::initializeCG()
 * This method is the used to initialized the ConflictGraph (composed by Face conflict and Points conflict).
 * inizializeCF() inizialized the ConflictGraph with all the visibile couples (Pt , f) with f face in dcel and
 * t > 4 (because the 4 points are already in the dcel)
 */
void ConflictGraph::initializeCG(){

}

