#include "conflictgraph.h"

/**
 * @brief ConflictGraph::ConflictGraph(DrawableDcel *dcel, std::vector<Dcel::Vertex *> vertexS)
 * This method is the constructor of the ConflictGraph class, it receive the pointer of the dcel
 * and the points.
 */
ConflictGraph::ConflictGraph(DrawableDcel *dcel, std::vector<Dcel::Vertex*> &vertexS){
    this->dcel = dcel;
    this->vertexS = vertexS;
    this->numberVertex = this->vertexS.size();
}

/**
 * @brief ConflictGraph::initializeCG()
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
                addFaceToVertex( face, vertexS[point]);
                addVertexToFace( vertexS[point], face);
            }
        }

        std::list<Dcel::Vertex*> stampa= f_conflict[face];
        std::list<Dcel::Vertex*>::iterator p;
        int z=0;
        for (p = stampa.begin(); p != stampa.end(); p++){
                cout << "Elemento " << z++ << ": " << *p <<" della lista della faccia "<<face->getId()<< endl;
        }
        
    }

    
}



/**
 * @brief ConflictGraph::addFaceToVertex()
 * This method is the used to insert the face f that is in conflict with the vertex v
 */
void ConflictGraph::addFaceToVertex(Dcel::Face* face, Dcel::Vertex* vertex){
    std::list<Dcel::Face*> faceList = v_conflict[vertex];
    faceList.push_back(face);
    v_conflict[vertex] = faceList;
}

/**
 * @brief ConflictGraph::addVertexToFace()
 * This method is the used to insert the vertex v that is in conflict with the face f
 */
void ConflictGraph::addVertexToFace(Dcel::Vertex* vertex,Dcel::Face* face){
    std::list<Dcel::Vertex*> vertexList = f_conflict[face];
    vertexList.push_back(vertex);
    f_conflict[face] = vertexList;
}

/**
 * @brief ConflictGraph::deleteFaceFromVertex()
 * This method is the used to delete the face f from the vertex v, because the face f is not in conflict
 */
void ConflictGraph::deleteFaceFromVertex(Dcel::Face* face, Dcel::Vertex* vertex){
    
}

/**
 * @brief ConflictGraph::deleteVertexFromFace()
 * This method is the used to delete the vertex v from the face f, because the vertex v is not in conflict
 */
void ConflictGraph::deleteVertexFromFace(Dcel::Vertex* vertex,Dcel::Face* face){
    
}


/**
 * @brief ConflictGraph::getFacesVisibleByVertex()
 * This method return the faces that are in conflict with vertex
 */
std::list<Dcel::Face *> ConflictGraph::getFacesVisibleByVertex(Dcel::Vertex *vertex){
    auto iter =this->v_conflict.find(vertex);
    if(iter!=v_conflict.end()){
        return v_conflict.at(vertex);
    }else{
        std::list<Dcel::Face *> c;
        return c;
    }
}

/**
 * @brief ConflictGraph::getVertexVisibleByFace()
 * This method return the vertexs that are in conflict with the face f
 */
std::list<Dcel::Vertex *> ConflictGraph::getVertexVisibleByFace(Dcel::Face *face){
    auto iter =this->f_conflict.find(face);
    if(iter!=f_conflict.end()){
        return f_conflict.at(face);
    }else{
        std::list<Dcel::Vertex *> c;
        return c;
    }
}
