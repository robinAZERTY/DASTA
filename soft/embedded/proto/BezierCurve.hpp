/*
Permet de décrire un mouvement de type Bezier à partir de 4 points et d'une durée (ou avec des position et vitesse initiales et finales)
Ici, le mouvement est en 4D (3D + cap).
Pour aider à l'implémentation, voici un exemple :
https://github.com/robinAZERTY/SEAD/blob/Simulation-dev-kit/Simulation-dev%200.12/lib_validated/scenario%20construction/BezierPositionMotion/BezierPositionMotion.cpp
*/


#include "Vector.hpp"

class BezierPositionMotion
{
public:
    BezierPositionMotion();
    BezierPositionMotion(const Vector points_4D[4], const double &duration);
    BezierPositionMotion(const Vector &initialPos, const Vector &initialVel, const Vector points_4D[2], const double &duration);
    BezierPositionMotion(const Vector &initialPos, const Vector &initialVel, const Vector &finalalPos, const Vector &finalalVel, const double &duration);
    void updatePose(const double &t);
    const Vector &getPos();
    const Vector &getVel();
    const Vector &getInitialPos();
    const Vector &getInitialVel();
    const Vector &getFinalPos();
    const Vector &getFinalVel();
    
private:
    Vector A = Vector(4),B=1,C=A,D=A, state=A;
    double alpha;
};  