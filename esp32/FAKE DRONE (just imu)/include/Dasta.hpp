/*
cette classe regroupe toutes les fonctionnaités necessaires au fonctionnement du saint projet
*/

#ifndef DASTA_HPP
#define DASTA_HPP
#include "SensorPreProcessing.hpp"
#include "StateEstimate.hpp"
#include "Communication.hpp"

class Dasta
{
public:
    Dasta();
    ~Dasta(){};
    SensorPreProcessing sensors;
    StateEstimate estimator;
    Communication communication; 
};

#endif // DASTA_HPP