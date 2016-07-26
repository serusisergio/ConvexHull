#include "conflictgraph.h"


/*********************************************************************
 * Convex Hull Algorithm, developed by Sergio Serusi 65041           *
 *********************************************************************
 *                                                                   *
 * In questa classe è presente lo sviluppo del conflict graph che    *
 * serve da appoggio al convex hull. Nel pdf allegato al progetto è  *
 * presente la desrizione di tutti i metodi, se qualcosa non è       *
 * chiaro non esitate a contattarmi: serusisergio@hotmail.it         *
 *********************************************************************/

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
    Eigen::Matrix<double,4,4> matrix;
    
    //Scorro le 4 facce presenti nella dcel
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
        //Controllo per ogni faccia quali vertici siano in conflitto
        for(int point=4; point<numberVertex; point++){
            Pointd p=vertexS[point]->getCoordinate();
            matrix(3,0) = p.x();
            matrix(3,1) = p.y();
            matrix(3,2) = p.z();
            matrix(3,3) = 1;
            //se il determinante è <0 vuol dire che la faccia vede il punto quindi gli isnerisco nel CG
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
    //Data la faccia ed un vertice, verifico se sono in conflitto
    Eigen::Matrix<double,4,4> matrix;
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
    //Se il det è <0 allora sono in conflitto e quindi si vedono
    return ((matrix.determinant()) < -std::numeric_limits<double>::epsilon());
}



/**
 * @brief ConflictGraph::addFaceToVertex()
 * This method is the used to insert the face f that is in conflict with the vertex v
 */
void ConflictGraph::addFaceToVertex(Dcel::Face* face, Dcel::Vertex* vertex){

    auto iter =this->f_conflict.find(vertex);
    if(iter!=f_conflict.end()){
        //se esiste gia lo aggiungo
        std::set<Dcel::Face*>* faceList = f_conflict[vertex];
        faceList->insert(face);
    }else{
        //Altrimenti lo creo per la prima volta ed inserisco
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

    auto iter =this->v_conflict.find(face);
    if(iter!=v_conflict.end()){
        //Se esiste gia lo aggiungo
        std::set<Dcel::Vertex*>* vertexList = v_conflict[face];
        vertexList->insert(vertex);
    }else{
        //altrimenti lo creo nuovo e poi aggiungo
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

        //Per ogni vertice che vede la faccia, elimino il riferimento ad essa da f_conflict
        for(std::set<Dcel::Vertex*>::iterator vit=vertexFromRemoveFace->begin(); vit!= vertexFromRemoveFace->end();++vit){
            Dcel::Vertex* currentVertex = *vit;
            f_conflict[currentVertex] -> erase(currentFace);
        }
        //Poi elimino la faccia stessa da v_conflict
        v_conflict.erase(currentFace);
    }
}

/**
 * @brief ConflictGraph::deleteVertexFromFace()
 * This method is the used to delete the vertex v from the face f, because the vertex v is not in conflict
 */
void ConflictGraph::deleteVertex(Dcel::Vertex* vertex){

    std::set<Dcel::Face*>* facesToRemoveVertex = getFacesVisibleByVertex(vertex);
    //Elimo i riferimenti al vertice dalla facce chee vede il vertice
    for(std::set<Dcel::Face*>::iterator fit = facesToRemoveVertex->begin(); fit != facesToRemoveVertex->end(); ++fit){
        Dcel::Face* currentFace=*fit;
        v_conflict[currentFace]->erase(vertex);
    }
    //elimino il vertice stesso
    f_conflict.erase(vertex);
}


/**
 * @brief ConflictGraph::getFacesVisibleByVertex()
 * This method return the faces that are in conflict with vertex
 */
std::set<Dcel::Face *>* ConflictGraph::getFacesVisibleByVertex(Dcel::Vertex *vertex){
    auto iter =this->f_conflict.find(vertex);
    if(iter!=f_conflict.end()){
        return new std::set<Dcel::Face*>(*f_conflict.at(vertex));
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
        return new std::set<Dcel::Vertex *>(*v_conflict.at(face));
    }else{
        return new std::set<Dcel::Vertex *>();
    }
}


/**
 * @brief ConflictGraph::UpdateCG()
 * This method return the vertexs that are in conflict with the face f
 */
void ConflictGraph::updateCG(Dcel::Face* faceToUpdate,std::set<Dcel::Vertex*>* setVertexForFace){
    //Scorro il set dei vertici
    for(std::set<Dcel::Vertex*>::iterator vit= setVertexForFace->begin(); vit != setVertexForFace->end(); ++vit){
        Dcel::Vertex* currentVertex = *vit;
        //Se il vertice è visibile dalla faccia allora lo aggiungo al cg
        if(isVisible(currentVertex, faceToUpdate)){
            addFaceToVertex( faceToUpdate, currentVertex );
            addVertexToFace( currentVertex, faceToUpdate );
        }
    }
}

/**
 * @brief ConflictGraph::getVertexMapToControlForTheNewFace()
 * This method is used to get the vertex that can be in conflict with the new Faces
 * @return map<Dcel::HalfEdge *, std::set<Dcel::Vertex*> *>
 */
std::map<Dcel::HalfEdge *, std::set<Dcel::Vertex*> *> ConflictGraph::getVertexMapToControlForTheNewFace(std::list<Dcel::HalfEdge *> horizon){
    std::map<Dcel::HalfEdge *, std::set<Dcel::Vertex*>*> vertexMap;

    //Scorro l'orizzonte, e per ogni half edge dell'orizzonte prendo i vertici in conflitto con la faccia dell'half edge consideranto e del suo twin e lo associo all'half edge dell'orizzonte
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


