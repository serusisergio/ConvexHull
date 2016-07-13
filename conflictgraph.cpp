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
    Matrix<double,4,4> matrix;
    
    for (Dcel::FaceIterator fit = dcel->faceBegin(); fit != dcel->faceEnd(); ++fit){
        Dcel::Face* face= *fit;

        int k=0;
        for(Dcel::Face::IncidentVertexIterator vit = face->incidentVertexBegin(); vit != face->incidentVertexEnd(); ++vit,k++){
            Dcel::Vertex* vertex=*vit;
            Pointd p= vertex->getCoordinate();
            matrix(k,0) = p.x();
            matrix(k,1) = p.y();
            matrix(k,2) = p.z();
            matrix(k,3) = 1;
        }

        for(int point=4; point<numberVertex; point++){
            Pointd p=vertexS[point]->getCoordinate();
            matrix(3,0) = p.x();
            matrix(3,1) = p.y();
            matrix(3,2) = p.z();
            matrix(3,3) = 1;

            if(matrix.determinant() > std::numeric_limits<double>::epsilon()){//nella

            }
        }
    }
}

/**
 * @brief ConvexHullCore::addFaceToVertex()
 * This method is the used to insert the face f that is in conflict with the vertex v
 */
void ConflictGraph::addFaceToVertex(){
    
}

/**
 * @brief ConvexHullCore::addVertexToFace()
 * This method is the used to insert the vertex v that is in conflict with the face f
 */
void ConflictGraph::addVertexToFace(){
    
}

/**
 * @brief ConvexHullCore::deleteFaceFromVertex()
 * This method is the used to delete the face f from the vertex v, because the face f is not in conflict
 */
void ConflictGraph::deleteFaceFromVertex(){
    
}

/**
 * @brief ConvexHullCore::deleteVertexFromFace()
 * This method is the used to delete the vertex v from the face f, because the vertex v is not in conflict
 */
void ConflictGraph::deleteVertexFromFace(){
    
}



