#include "convexhullcore.h"

ConvexHullCore::ConvexHullCore(DrawableDcel *dcel){

    this->dcel         = dcel;
    this->numberVertex = dcel->getNumberVertices();
    this->vertexS      = std::vector<Dcel::Vertex*>(numberVertex);

}

/**
 * @brief ConvexHullCore::VerifyEuleroProperty()
 * This method is executed to verify the Eurler's formula (n-ne+nf=2)
 * @return True if the dcel respects the property, False otherwhise
 */
bool ConvexHullCore::verifyEuleroProperty(){

    int numberFace   = dcel->getNumberFaces();
    int numberVertex = dcel->getNumberVertices();
    int numberEdge   = dcel->getNumberHalfEdges()/2;

    int euler        = numberVertex-numberFace+numberEdge;

    if(euler==2){
        return true;
    }else{
        return false;
    }
}

/**
 * @brief ConvexHullCore::getVertexs()
 * This method is executed to get all the vertex of the dcel
 */
void ConvexHullCore::getVertexs(){

    std::vector<Dcel::Vertex*>::iterator vectIt = vertexS.begin();
    for(Dcel::VertexIterator vit = dcel->vertexBegin(); vit != dcel->vertexEnd() && vectIt != vertexS.end(); ++vit,++vectIt){
        *vectIt = new Dcel::Vertex(**vit);
    }
}

/**
 * @brief ConvexHull::isCoplanar()
 * This method is execut to verify if the 4 points are coplanar
 * @return True if all the 4 points are coplanar, false otherwise
 * http://mathworld.wolfram.com/Coplanar.html
 * In this method is used eigen library, it can easily perform the
 * determinant of a matrix
 */
bool ConvexHullCore::isCoplanar(){

    Pointd p0= vertexS[0]->getCoordinate();
    Pointd p1= vertexS[1]->getCoordinate();
    Pointd p2= vertexS[2]->getCoordinate();
    Pointd p3= vertexS[3]->getCoordinate();

    Matrix<double,4,4> matrix;
    matrix<< p0.x(), p0.y(), p0.z(), 1,
             p1.x(), p1.y(), p1.z(), 1,
             p2.x(), p2.y(), p2.z(), 1,
             p3.x(), p3.y(), p3.z(), 1;

    double det = matrix.determinant();

    if(det > std::numeric_limits<double>::epsilon() || det < -std::numeric_limits<double>::epsilon()){
        return false; //zucchero sintattico, fa un return 0
    }else{
        return true; // fa un return 1
    }
}

/**
 * @brief ConvexHullCore::getPermutation()
 * This method is executed to execute the permutation of the vertexs
 * http://www.cplusplus.com/reference/algorithm/random_shuffle/
 */
void ConvexHullCore::executePermutation(){

    do{
       std::random_shuffle(this->vertexS.begin(), this->vertexS.end());
    }while(isCoplanar());
}


/**
 * @brief ConvexHullCore::setTetrahedron
 * This method is used to set the first 4 points to create the initial tetrahedron. In the dcel, after
 * the execution of setTetrahedron() the dcel contain the convex hull of the 4 initial points
 */
void ConvexHullCore::setTetrahedron(){
    //CREAZIONE TETRAEDRO

    //CREAZIONE TRIANGOLO
    std::vector<Dcel::Vertex*> vertexInitial  = std::vector<Dcel::Vertex*>(3);  //conterrà i vertici per formare la base
    std::vector<Dcel::HalfEdge*> heInTriangle = std::vector<Dcel::HalfEdge*>(3);//conterrà gli halfedge per formare la base

    //HalfEdge del Tetraedro
    std::vector<Dcel::HalfEdge*> heInTetrahedron = std::vector<Dcel::HalfEdge*>(9);//conterrà gli halfedge per completare il tetraedro

    //Creazione vertici della Base
    for(unsigned int i=0;i<3;i++){
        vertexInitial[i] = this->dcel->addVertex(this->vertexS[i]->getCoordinate());
        heInTriangle[i]  = this->dcel->addHalfEdge();
    }

    //AggiuntaHalfEdgeTetraedro
    for(unsigned int i=0;i<9;i++){
        heInTetrahedron[i]= this->dcel->addHalfEdge();
    }

    //Creazione faccia della base
    Dcel::Face* f1= this->dcel->addFace();

    //Ciclo per impostare i next,prev,from,to e face della Base
    for(unsigned int i=0;i<3;i++){//uso il modulo per completare il ciclo
        heInTriangle[i]->setFromVertex(vertexInitial[ i ]);
        heInTriangle[i]->setToVertex(vertexInitial[ (i+1)%3 ]);
        heInTriangle[i]->setNext(heInTriangle[ (i+1)%3 ]);
        heInTriangle[i]->setPrev(heInTriangle[ (i+2)%3 ]);
        heInTriangle[i]->setFace(f1);
        vertexInitial[i]->setIncidentHalfEdge(heInTriangle[ i ]);
        vertexInitial[ i ]->incrementCardinality();
        vertexInitial[ (i+1)%3 ]->incrementCardinality();
    }
    f1->setOuterHalfEdge(heInTriangle[0]);

    //Aggiunta quarto vertice
    Dcel::Vertex* v4 =this->dcel->addVertex(this->vertexS[3]->getCoordinate());



    //Settaggio half edge tetraedro
    int k=0;//usata per gli half edge del tetraedro
    for(unsigned int i=0;i<3;i++){//ad ogni ciclo creo una faccia F ed imposto i 3 half edge della faccia corrente (F)
        Dcel::Face* face= this->dcel->addFace();

        heInTetrahedron[k]->setFromVertex(vertexInitial[ i ]);
        heInTetrahedron[k]->setToVertex(v4);
        heInTetrahedron[k]->setNext(heInTetrahedron[ (k+1) ]);
        heInTetrahedron[k]->setPrev(heInTetrahedron[ (k+2) ]);
        heInTetrahedron[k]->setFace(face);
        heInTetrahedron[k]->setTwin(heInTetrahedron[ (k+7)%9 ]);
        v4->setIncidentHalfEdge(heInTetrahedron[k]);
        vertexInitial[ i ]->incrementCardinality();
        v4->incrementCardinality();


        face->setOuterHalfEdge(heInTetrahedron[k]);

        k++;
        heInTetrahedron[k]->setFromVertex(v4);
        heInTetrahedron[k]->setToVertex(vertexInitial[ (i+1)%3 ]);
        heInTetrahedron[k]->setNext(heInTetrahedron[ (k+1) ]);
        heInTetrahedron[k]->setPrev(heInTetrahedron[ (k-1) ]);
        heInTetrahedron[k]->setFace(face);
        heInTetrahedron[k]->setTwin(heInTetrahedron[ (k+2)%9 ]);
        vertexInitial[ (i+1)%3 ]->incrementCardinality();
        v4->incrementCardinality();

        k++;
        heInTetrahedron[k]->setFromVertex(vertexInitial[ (i+1)%3 ]);
        heInTetrahedron[k]->setToVertex(vertexInitial[ i ]);
        heInTetrahedron[k]->setNext(heInTetrahedron[ (k-2) ]);
        heInTetrahedron[k]->setPrev(heInTetrahedron[ (k-1) ]);
        heInTetrahedron[k]->setFace(face);
        heInTetrahedron[k]->setTwin(heInTriangle[i]);
        heInTriangle[i]->setTwin(heInTetrahedron[k]);
        vertexInitial[ (i+1)%3 ]->incrementCardinality();
        vertexInitial[ i ]->incrementCardinality();

        k++;
    }

}

/**
 * @brief ConvexHullCore::getHorizon()
 * This method is executed to find the horizon by a faces visible by a vertex
 * This method, return a pointer of the horizon list
 */
std::list<Dcel::HalfEdge*> ConvexHullCore::getHorizon(std::set<Dcel::Face *> *facesVisibleByVertex){
    std::set<Dcel::HalfEdge*> horizonUnordered;
    std::list<Dcel::HalfEdge*> horizonOrdered;
    std::map<Dcel::Vertex*, Dcel::HalfEdge*> hm;


    for(std::set<Dcel::Face*>::iterator fit = facesVisibleByVertex->begin(); fit != facesVisibleByVertex->end(); ++fit){
        Dcel::Face* currentFace = *fit;
        cout<<"Faccia ->"<<currentFace->getId()<<endl;

        Dcel::HalfEdge* outerHE = currentFace->getOuterHalfEdge();
        Dcel::HalfEdge* twin    = outerHE->getTwin();

        //Per ogni faccia scorro gli half edge della faccia
        for(int i=0;i<3;i++){
            Dcel::Face* faceTwin = twin->getFace();

            if(facesVisibleByVertex->count(faceTwin) == 0){//se il twin dell'HE sta in una faccia non visibile, allora HE è proprio nell'orizzonte
                horizonUnordered.insert(twin);
                cout<<"Inserimento HE nell'orizzonte ->"<<twin->getId()<<endl;
            }

            outerHE = outerHE->getNext();
            twin    = outerHE->getTwin();
        }

    }

    //Oridno la lista degli HE dell'orizzonte
    for(std::set<Dcel::HalfEdge*>::iterator heit= horizonUnordered.begin(); heit!= horizonUnordered.end() ; ++heit){
        Dcel::HalfEdge* he= *heit;
        hm[he->getFromVertex()]=he;
    }


    int count=0;
    for(std::set<Dcel::HalfEdge*>::iterator heit= horizonUnordered.begin(); heit!= horizonUnordered.end() ; ++heit){
        Dcel::HalfEdge* he;//il precedente
        if(count==0){
            he= *heit;
            horizonOrdered.push_back(*heit);
            count++;
        }
        horizonOrdered.push_back(hm[he->getToVertex()]);
        he= hm[he->getToVertex()];
    }

    horizonOrdered.pop_back();

    std::vector<Dcel::HalfEdge*> horizVertex=std::vector<Dcel::HalfEdge*>(horizonOrdered.size());
    int i=0;
    for(std::list<Dcel::HalfEdge*>::iterator heit= horizonOrdered.begin(); heit!= horizonOrdered.end() ; ++heit,i++){
        cout<<"Vertice "<<(*heit)->getId()<<endl;
        dcel->addDebugCylinder((*heit)->getFromVertex()->getCoordinate(), (*heit)->getToVertex()->getCoordinate(), 0.005, QColor(255,0,0));


    }

    return horizonOrdered;


}

/**
 * @brief ConvexHullCore::removeFacesVisibleByVertex(std::set<Dcel::Face *> *facesVisibleByVertex)
 * This method is executed to remove the face that the current point see
 */
void ConvexHullCore::removeFacesVisibleByVertex(std::set<Dcel::Face *> facesVisibleByVertex){

    std::list<Dcel::Vertex*> vertexsToRemove;


    for(std::set<Dcel::Face*>::iterator fit = facesVisibleByVertex.begin(); fit != facesVisibleByVertex.end(); ++fit){
        Dcel::Face* currentFace = *fit;
        cout<<"Rimuovendo faccia FAccia ->"<<currentFace->getId();
        Dcel::HalfEdge* outerHE= currentFace->getOuterHalfEdge();

        for(int i=0;i<3;i++){
            Dcel::Vertex* from = outerHE -> getFromVertex();
            Dcel::Vertex* to   = outerHE -> getToVertex();

            from -> decrementCardinality();
            to   -> decrementCardinality();

            dcel -> deleteHalfEdge(outerHE);

            outerHE = outerHE->getNext();
            if(from -> getCardinality() == 0){
                vertexsToRemove.push_back(from);
            }
            if(to -> getCardinality() == 0){
                vertexsToRemove.push_back(to);
            }
        }
        this -> dcel -> deleteFace(currentFace);
        cout<<"Eliminazione faccia ->"<<currentFace->getId()<<endl;

    }

    if(vertexsToRemove.size()>0){
        for(std::list<Dcel::Vertex*>::iterator heit= vertexsToRemove.begin(); heit!= vertexsToRemove.end() ; ++heit){
            dcel->deleteVertex(*heit);
            cout<<"Rimozione vertice ->"<<(*heit)->getId()<<endl;
        }
    }

}

/**
 * @brief ConvexHullCore::createNewFaces(std::list<Dcel::HalfEdge *> horizon, Dcel::Vertex *)
 * This method is executed to create the new faces using the horizon
 */
void ConvexHullCore::createNewFaces(std::list<Dcel::HalfEdge *> horizon, Dcel::Vertex* v3){
    std::vector<Dcel::HalfEdge*> heEnter = std::vector<Dcel::HalfEdge*>(horizon.size());
    std::vector<Dcel::HalfEdge*> heExit  = std::vector<Dcel::HalfEdge*>(horizon.size());

    cout<<"Creazione nuove facce"<<endl;
    int i=0;
    for(std::list<Dcel::HalfEdge*>::iterator it = horizon.begin(); it != horizon.end(); ++it){
        Dcel::HalfEdge* currentHalfEdgeHorizon = *it;
        cout<<"HalfEdgeOrizzonte ->"<<currentHalfEdgeHorizon->getId() <<endl;

        Dcel::HalfEdge* halfEdge1=dcel->addHalfEdge();
        Dcel::HalfEdge* halfEdge2=dcel->addHalfEdge();
        Dcel::HalfEdge* halfEdge3=dcel->addHalfEdge();

        Dcel::Face* currentFace = dcel->addFace();
        currentFace->setOuterHalfEdge(halfEdge1);

        Dcel::Vertex* v1 = currentHalfEdgeHorizon->getToVertex(); //attenzione all'ordine, deve essere in senso antiorario, regola mano destra
        Dcel::Vertex* v2 = currentHalfEdgeHorizon->getFromVertex();

        halfEdge1 -> setFromVertex(v1);
        halfEdge1 -> setToVertex(v2);
        halfEdge1 -> setFace(currentFace);
        halfEdge1 -> setNext(halfEdge2);
        halfEdge1 -> setPrev(halfEdge3);
        halfEdge1 -> setTwin(currentHalfEdgeHorizon);
        currentHalfEdgeHorizon -> setTwin(halfEdge2);
        v1 -> setIncidentHalfEdge(halfEdge1);
        v1 -> incrementCardinality();
        v2 -> incrementCardinality();

        halfEdge2 -> setFromVertex(v2);
        halfEdge2 -> setToVertex(v3);
        halfEdge2 -> setFace(currentFace);
        halfEdge2 -> setNext(halfEdge3);
        halfEdge2 -> setPrev(halfEdge1);
        v2 -> setIncidentHalfEdge(halfEdge2);
        v2 -> incrementCardinality();
        v3 -> incrementCardinality();

        heExit[i] = halfEdge2;

        halfEdge3 -> setFromVertex(v3);
        halfEdge3 -> setToVertex(v1);
        halfEdge3 -> setFace(currentFace);
        halfEdge3 -> setNext(halfEdge1);
        halfEdge3 -> setPrev(halfEdge2);
        v3 -> setIncidentHalfEdge(halfEdge3);
        v3 -> incrementCardinality();
        v1 -> incrementCardinality();

        heEnter[i] = halfEdge3;
        i++;
        dcel->addDebugCylinder(currentHalfEdgeHorizon->getFromVertex()->getCoordinate(), currentHalfEdgeHorizon->getToVertex()->getCoordinate(), 0.005, QColor(255,0,0));

    }

    //Settaggio twin half edge
    int dim=(heEnter.size())-1;
    for(int i=0; i <= dim ; i++){
        heEnter[(i+1)%dim] -> setTwin(heExit[i]);
        heExit[i] -> setTwin(heEnter[(i+1)%dim]);
    }
}

/**
 * @brief ConvexHullCore::findConvexHull()
 * This method is executed to find the convex hull given a set of points (contains into dcel)
 * This method, is the principal method of the class
 */
void ConvexHullCore::findConvexHull(){

    //Salva i vertici della dcel in un vector (vertexS) perchè alla dcel verra chiamato reset()
    getVertexs();

    //Calcola una permutazione random degli n punti
    executePermutation();
    
    //Pulizia della dcel, che conterrà il convex hull alla fine dell'algoritmo
    this->dcel->reset();

    //Trova 4 punti che formano il tetraedro (quindi il convex hull di questi 4 punti)
    setTetrahedron();

    //Inizializza il conflict graph con tutte le coppie visibili (Pt,f) con f faccia in dcel e t>4 (quindi con i punti successivi)
    ConflictGraph conflictGraph=ConflictGraph(this->dcel, this-> vertexS);
    conflictGraph.initializeCG();

    int count=0;
    //Ciclo principlae sei punti, dal punto 4 fino alla fine
    for(unsigned int point_i=4; point_i < vertexS.size(); point_i++){
        dcel->clearDebugCylinders();
        dcel->clearDebugSpheres();

        Dcel::Vertex* currentPoint=vertexS[point_i];
        cout<<"Current vertex->"<<currentPoint->getId()<<endl;

        std::set<Dcel::Face*>* facesVisibleByVertex=conflictGraph.getFacesVisibleByVertex(currentPoint);
        std::list<Dcel::HalfEdge*> horizon;


        //Se il punto corrente non è all'interno del convex hull, allora bisogna aggiornare il convexhull
        if(facesVisibleByVertex->size()>0){

            //Inserimento punto nella dcel
            Dcel::Vertex* currentVertex = dcel->addVertex(vertexS[point_i]->getCoordinate());
            currentVertex -> setCardinality(0);
            cout<<"Aggiunto vertice alla DCEL "<<endl;


            //Ricerca Orizzonte
            horizon = getHorizon(facesVisibleByVertex);
            dcel->addDebugSphere((currentPoint)->getCoordinate(), 0.01, QColor(255,100,0));

            //Cancellazione Facce Visibili dal punto
            removeFacesVisibleByVertex(*facesVisibleByVertex);

            //Creazione nuove facce
            //createNewFaces(horizon,currentVertex);



        }
        count++;
        if(count ==1){
           break;
        }


        conflictGraph.deleteVertexFromFace(currentPoint);
        conflictGraph.deleteFaceFromVertex(facesVisibleByVertex);
    }


    /*
    for(std::vector<Dcel::Vertex*>::iterator it = vertexS.begin(); it != vertexS.end(); ++it){
            dcel->addDebugSphere((*it)->getCoordinate(), 0.01, QColor(255,0,0));
    }
    */



}
