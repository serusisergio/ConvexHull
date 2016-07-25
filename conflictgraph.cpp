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
        Dcel::HalfEdge* halfEdge = face->getOuterHalfEdge();



        for(int i=0;i<3;i++,k++){
            Dcel::Vertex* vertex = halfEdge->getFromVertex();
            Pointd p= vertex->getCoordinate();
            matrix(k,0) = p.x();
            matrix(k,1) = p.y();
            matrix(k,2) = p.z();
            matrix(k,3) = 1;
            halfEdge = halfEdge->getNext();
        }

        for(int point=4; point<numberVertex; point++){
            Pointd p=vertexS[point]->getCoordinate();
            matrix(3,0) = p.x();
            matrix(3,1) = p.y();
            matrix(3,2) = p.z();
            matrix(3,3) = 1;

            if(matrix.determinant() <- std::numeric_limits<double>::epsilon()){//nella
                addFaceToVertex( face, vertexS[point]);
                addVertexToFace( vertexS[point], face);
            }
        }

    }

}

/**
 * @brief ConflictGraph::isVisible()
 * This method is the used to verify if the vertex see the face
 */
bool ConflictGraph::isVisible(Dcel::Vertex *vertex, Dcel::Face *face){
    Matrix<double,4,4> matrix;
    int k=0;
    for(Dcel::Face::IncidentVertexIterator vit = face->incidentVertexBegin(); vit != face->incidentVertexEnd(); ++vit,k++){
        Dcel::Vertex* vertex=*vit;
        Pointd p= vertex->getCoordinate();
        matrix(k,0) = p.x();
        matrix(k,1) = p.y();
        matrix(k,2) = p.z();
        matrix(k,3) = 1;
    }
    Pointd p=vertex->getCoordinate();
    matrix(3,0) = p.x();
    matrix(3,1) = p.y();
    matrix(3,2) = p.z();
    matrix(3,3) = 1;

    return ((matrix.determinant()) < -std::numeric_limits<double>::epsilon());
}



/**
 * @brief ConflictGraph::addFaceToVertex()
 * This method is the used to insert the face f that is in conflict with the vertex v
 */
void ConflictGraph::addFaceToVertex(Dcel::Face* face, Dcel::Vertex* vertex){

    //std::list<Dcel::Face*>* faceList = v_conflict[vertex];
    /*
    if(faceList==nullptr){
        v_conflict[vertex]=new std::list<Dcel::Face*>();
    }

    v_conflict[vertex]->push_back(face);
    */
    auto iter =this->f_conflict.find(vertex);
    if(iter!=f_conflict.end()){
        std::set<Dcel::Face*>* faceList = f_conflict[vertex];
        faceList->insert(face);
    }else{
        std::set<Dcel::Face*>* faceList = new std::set<Dcel::Face*>();
        faceList->insert(face);
        f_conflict[vertex]=faceList;
    }
}

/**
 * @brief ConflictGraph::addVertexToFace()
 * This method is the used to insert the vertex v that is in conflict with the face f
 */
void ConflictGraph::addVertexToFace(Dcel::Vertex* vertex,Dcel::Face* face){
    /*
    std::list<Dcel::Vertex*>* vertexList = f_conflict[face];

    if(vertexList==nullptr){
        f_conflict[face]=new std::list<Dcel::Vertex*>();
    }
    f_conflict[face]->push_back(vertex);
    */
    auto iter =this->v_conflict.find(face);
    if(iter!=v_conflict.end()){
        std::set<Dcel::Vertex*>* vertexList = v_conflict[face];
        vertexList->insert(vertex);
    }else{
        std::set<Dcel::Vertex*>* vertexList = new std::set<Dcel::Vertex*>();
        vertexList->insert(vertex);
        v_conflict[face]=vertexList;
    }
}

/**
 * @brief ConflictGraph::deleteFaces()
 * This method is the used to delete the face f from the vertex v, because the face f is not in conflict
 */
void ConflictGraph::deleteFaces(std::set<Dcel::Face*> *faces){
    //Per ogni faccia
    for(std::set<Dcel::Face*>::iterator fit = faces->begin(); fit != faces->end(); ++fit){
        Dcel::Face* currentFace=*fit;
        std::set<Dcel::Vertex*>* vertexFromRemoveFace = getVertexVisibleByFace(currentFace);

        //Per ogni vertice che vede la faccia, elimino il riferimento ad essa
        for(std::set<Dcel::Vertex*>::iterator vit=vertexFromRemoveFace->begin(); vit!= vertexFromRemoveFace->end();++vit){
            Dcel::Vertex* currentVertex = *vit;
            f_conflict[currentVertex] -> erase(currentFace);
        }

    }
    //Elimino le facce
    for(std::set<Dcel::Face*>::iterator fit = faces->begin(); fit != faces->end(); ++fit){
        Dcel::Face* currentFace=*fit;
        v_conflict.erase(currentFace);
    }

}

/**
 * @brief ConflictGraph::deleteVertexFromFace()
 * This method is the used to delete the vertex v from the face f, because the vertex v is not in conflict
 */
void ConflictGraph::deleteVertex(Dcel::Vertex* vertex){

    std::set<Dcel::Face*>* facesToRemoveVertex = getFacesVisibleByVertex(vertex);

    for(std::set<Dcel::Face*>::iterator fit = facesToRemoveVertex->begin(); fit != facesToRemoveVertex->end(); ++fit){
        Dcel::Face* currentFace=*fit;
        v_conflict[currentFace]->erase(vertex);
    }

    f_conflict.erase(vertex);
}


/**
 * @brief ConflictGraph::getFacesVisibleByVertex()
 * This method return the faces that are in conflict with vertex
 */
std::set<Dcel::Face *>* ConflictGraph::getFacesVisibleByVertex(Dcel::Vertex *vertex){
    auto iter =this->f_conflict.find(vertex);
    if(iter!=f_conflict.end()){
        return f_conflict.at(vertex);
    }else{
        return new std::set<Dcel::Face*>();
    }
}

/**
 * @brief ConflictGraph::getVertexVisibleByFace()
 * This method return the vertexs that are in conflict with the face f
 */
std::set<Dcel::Vertex *>* ConflictGraph::getVertexVisibleByFace(Dcel::Face *face){
    auto iter =this->v_conflict.find(face);
    if(iter!=v_conflict.end()){
        return v_conflict.at(face);
    }else{
        return new std::set<Dcel::Vertex *>();
    }
}


/**
 * @brief ConflictGraph::UpdateCG()
 * This method return the vertexs that are in conflict with the face f
 */
void ConflictGraph::updateCG(Dcel::Face* faceToUpdate,std::set<Dcel::Vertex*>* setVertexForFace){

    for(std::set<Dcel::Vertex*>::iterator vit= setVertexForFace->begin(); vit != setVertexForFace->end(); ++vit){
        Dcel::Vertex* currentVertex = *vit;
        if(isVisible(currentVertex, faceToUpdate)){
            addFaceToVertex( faceToUpdate, currentVertex );
            addVertexToFace( currentVertex, faceToUpdate );
        }
    }
}

/**
 * @brief ConflictGraph::getVertexMapToControlForTheNewFace()
 * This method return the vertexs that are in conflict with the face f
 */
std::map<Dcel::HalfEdge *, std::set<Dcel::Vertex*> *> ConflictGraph::getVertexMapToControlForTheNewFace(std::list<Dcel::HalfEdge *> horizon){
    std::map<Dcel::HalfEdge *, std::set<Dcel::Vertex*>*> vertexMap;

    for(std::list<Dcel::HalfEdge*>::iterator hit = horizon.begin(); hit != horizon.end(); ++hit){
        Dcel::HalfEdge* currentHalfEdge = *hit;
        std::set<Dcel::Vertex*>* unionVertex= new std::set<Dcel::Vertex*>();

        std::set<Dcel::Vertex*>* vertexFaceHorizon     = getVertexVisibleByFace(currentHalfEdge->getFace());
        std::set<Dcel::Vertex*>* vertexFaceTwinHorizon = getVertexVisibleByFace(currentHalfEdge->getTwin()->getFace());

        unionVertex->insert(vertexFaceHorizon->begin(),         vertexFaceHorizon->end());
        unionVertex->insert(vertexFaceTwinHorizon->begin(), vertexFaceTwinHorizon->end());
        vertexMap[currentHalfEdge] = unionVertex;
    }
    return vertexMap;
}

void ConflictGraph::updateCGIgnorante(unsigned int point_i){
    f_conflict.clear();
    v_conflict.clear();
    Matrix<double,4,4> matrix;

    for (Dcel::FaceIterator fit = dcel->faceBegin(); fit != dcel->faceEnd(); ++fit){
        Dcel::Face* face= *fit;

        int k=0;
        Dcel::HalfEdge* halfEdge = face->getOuterHalfEdge();

        for(int i=0;i<3;i++,k++){
            Dcel::Vertex* vertex = halfEdge->getFromVertex();
            Pointd p= vertex->getCoordinate();
            matrix(k,0) = p.x();
            matrix(k,1) = p.y();
            matrix(k,2) = p.z();
            matrix(k,3) = 1;
            halfEdge = halfEdge->getNext();
        }

        for(int point=(point_i); point<numberVertex; point++){
            Pointd p=vertexS[point]->getCoordinate();
            matrix(3,0) = p.x();
            matrix(3,1) = p.y();
            matrix(3,2) = p.z();
            matrix(3,3) = 1;

            if(matrix.determinant() <- std::numeric_limits<double>::epsilon()){//nella
                addFaceToVertex( face, vertexS[point]);
                addVertexToFace( vertexS[point], face);
            }
        }

    }
}


