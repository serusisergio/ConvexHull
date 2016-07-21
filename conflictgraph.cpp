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

            if(matrix.determinant() < -std::numeric_limits<double>::epsilon()){//nella
                addFaceToVertex( face, vertexS[point]);
                addVertexToFace( vertexS[point], face);
            }
        }


        /*
        std::list<Dcel::Vertex*>* stampa= f_conflict[face];
        std::list<Dcel::Vertex*>::iterator p;
        int z=0;


        for (p = stampa->begin(); p != stampa->end(); p++){
                cout << "Elemento " << z++ << ": " << *p <<" della lista della faccia "<<face->getId()<< endl;
        }
        */

    }

    /*
    for(int point=4; point<numberVertex; point++){
        cout<<"Vertex s "<<point<<"   : "<<vertexS[point]<<endl;

        std::list<Dcel::Face*>* stampa= v_conflict[vertexS[point]];
        std::list<Dcel::Face*>::iterator p;
        int z=0;
        if(stampa!=nullptr){
            for (p = stampa->begin(); p != stampa->end(); p++){
                    cout << "Elemento " << z++ << ": " << *p <<" della lista del vertice "<<point<< endl;
            }
        }


    }*/


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

    return (matrix.determinant() < -std::numeric_limits<double>::epsilon());
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
    auto iter =this->v_conflict.find(vertex);
    if(iter!=v_conflict.end()){
        std::set<Dcel::Face*>* faceList = v_conflict[vertex];
        faceList->insert(face);
    }else{
        std::set<Dcel::Face*>* faceList = new std::set<Dcel::Face*>();
        faceList->insert(face);
        v_conflict[vertex]=faceList;
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
    auto iter =this->f_conflict.find(face);
    if(iter!=f_conflict.end()){
        std::set<Dcel::Vertex*>* vertexList = f_conflict[face];
        vertexList->insert(vertex);
    }else{
        std::set<Dcel::Vertex*>* vertexList = new std::set<Dcel::Vertex*>();
        vertexList->insert(vertex);
        f_conflict[face]=vertexList;
    }
}

/**
 * @brief ConflictGraph::deleteFaceFromVertex()
 * This method is the used to delete the face f from the vertex v, because the face f is not in conflict
 */
void ConflictGraph::deleteFaceFromVertex(std::set<Dcel::Face*> *faces){

    for(std::set<Dcel::Face*>::iterator face = faces->begin(); face!= faces->end(); ++face){

        std::map<Dcel::Vertex*, std::set<Dcel::Face*>*>::iterator vertexIt =v_conflict.begin();
        for(; vertexIt != v_conflict.end(); ++vertexIt){
            if(vertexIt->second->size()>0){
                vertexIt->second->erase(*face);
            }
        }
        f_conflict.erase(*face);
    }
}

/**
 * @brief ConflictGraph::deleteVertexFromFace()
 * This method is the used to delete the vertex v from the face f, because the vertex v is not in conflict
 */
void ConflictGraph::deleteVertexFromFace(Dcel::Vertex* vertex){
    
    v_conflict.erase(vertex);

     for(std::map<Dcel::Face*, std::set<Dcel::Vertex*>*>::iterator fit = f_conflict.begin(); fit != f_conflict.end(); ++fit){
         if(fit->second != nullptr){
             fit->second->erase(vertex);
         }
 }
    
}

void ConflictGraph::deleteFaceAndVertex(std::set<Dcel::Face *>* faces, Dcel::Vertex* vertex){
    /*

    //Se la lista delle facce visibili da vertex non è vuota allora cancellale
    if(faces->size()>0){
        for(auto i : *faces){
            std::list<Dcel::Vertex *>* vertexs = f_conflict[i];
            for(auto j : *vertexs){
                std::list<Dcel::Face *>* face =v_conflict[j];
                face->erase(i);
            }
            cout<<"Faccia "<<i->getId()<<endl;

        }
    }
    cout<<"____"<<endl;
    //Elimina il vertice
    v_conflict.erase(vertex);

*/
}


/**
 * @brief ConflictGraph::getFacesVisibleByVertex()
 * This method return the faces that are in conflict with vertex
 */
std::set<Dcel::Face *>* ConflictGraph::getFacesVisibleByVertex(Dcel::Vertex *vertex){
    auto iter =this->v_conflict.find(vertex);
    if(iter!=v_conflict.end()){
        return v_conflict.at(vertex);
    }else{
        return new std::set<Dcel::Face*>();
    }
}

/**
 * @brief ConflictGraph::getVertexVisibleByFace()
 * This method return the vertexs that are in conflict with the face f
 */
std::set<Dcel::Vertex *>* ConflictGraph::getVertexVisibleByFace(Dcel::Face *face){
    auto iter =this->f_conflict.find(face);
    if(iter!=f_conflict.end()){
        return f_conflict.at(face);
    }else{
        return new std::set<Dcel::Vertex *>();
    }
}
