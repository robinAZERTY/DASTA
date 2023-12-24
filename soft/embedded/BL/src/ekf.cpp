#include "ekf.hpp"

Matrix *Ekf::tmp1;
Matrix *Ekf::tmp2;
Matrix *Ekf::Min;
Matrix Ekf::refP=Matrix();
Vector *Ekf::Vin;

Ekf::Ekf(Matrix_f2 f, Matrix_f1 h, int x_dim, int z_dim, int u_dim, Matrix_f2 Fx, Matrix_f2 Fu, Matrix_f1 H)
{
    this->x_dim = x_dim;
    this->z_dim = z_dim;
    this->u_dim = u_dim;

    this->u = new Vector(u_dim);
    this->z = new Vector(z_dim);
    this->R = new Matrix(z_dim, z_dim);
    this->R->set_eye();

    this->x = new Vector(x_dim);
    this->xtmp = new Vector(x_dim);
    this->P = new Matrix(x_dim, x_dim);
    this->P->set_eye();

    this->h = h;
    this->H = H;
    this->Q = new Matrix(u_dim, u_dim);
    this->Q->set_eye();
    this->I = new Matrix(x_dim, x_dim);
    this->I->set_eye();

    this->f = f;
    this->Fx = Fx;
    this->Fu = Fu;
    this->K = new Matrix(x_dim, z_dim);
    this->S = new Matrix(z_dim, z_dim);

    this->Fx_val = new Matrix(x_dim, x_dim);
    this->Fu_val = new Matrix(x_dim, u_dim);
    this->H_val = new Matrix(z_dim, x_dim);

    this->y = new Vector(z_dim);

    int max_dim = (x_dim > z_dim) ? x_dim : z_dim;
    max_dim = (max_dim > u_dim) ? max_dim : u_dim;

    Ekf::tmp1 = new Matrix(max_dim, max_dim);
    Ekf::tmp2 = new Matrix(max_dim, max_dim);
}

Ekf::~Ekf()
{
    delete this->u;
    delete this->z;
    delete this->R;
    delete this->x;
    delete this->P;
    delete this->Q;
    delete this->I;
    delete this->K;
    delete this->S;
    delete this->Fx_val;
    delete this->Fu_val;
    delete this->H_val;
    delete this->y;
    delete this->xtmp;
}

void Ekf::predict()
{
    // x<- f(x,u)
    Vin=x;
    f(*x, *u);

    // P<- Fx*P*Fx'
    Min=Fx_val;
    Fx(*x, *u);
    tmp1->cols = x_dim;
    tmp1->rows = x_dim;
    mul(*tmp1, *Fx_val, *P);
    Fx_val->transpose();
    mul(*P, *tmp1, *Fx_val);
    Fx_val->transpose();


    // tmp2<- Fu*Q*Fu'
    Min=Fu_val;
    Fu(*x, *u);
    tmp1->cols = u_dim;
    tmp1->rows = x_dim;
    mul(*tmp1, *Fu_val, *Q);
    tmp2->cols = x_dim;
    tmp2->rows = x_dim;
    Fu_val->transpose();
    mul(*tmp2, *tmp1, *Fu_val);
    Fu_val->transpose();

    // P<- P+tmp2
    add(*P, *P, *tmp2);
}

void Ekf::update()
{
    // y<- z-h(x)
    Vin=y;
    h(*x);
    sub(*y, *z, *y);

    // S<- H*P*H'+R
    Min=H_val;
    H(*x);
    tmp1->cols = x_dim;
    tmp1->rows = z_dim;
    mul(*tmp1, *H_val, *P);
    H_val->transpose();
    mul(*S, *tmp1, *H_val);
    add(*S, *S, *R);

    // K<- P*H'*S^-1
    tmp1->cols = z_dim;
    tmp1->rows = x_dim;
    mul(*tmp1, *P, *H_val);
    H_val->transpose();
    tmp2->cols = z_dim;
    tmp2->rows = z_dim;
    inv(*tmp2, *S);
    mul(*K, *tmp1, *tmp2);

    // x<- x+K*y
    mul(*xtmp, *K, *y);
    add(*x, *x, *xtmp);

    // P<- (I-K*H)*P
    tmp1->cols = x_dim;
    tmp1->rows = x_dim;
    mul(*tmp1, *K, *H_val);
    sub(*tmp1, *I, *tmp1);
    tmp2->cols = x_dim;
    tmp2->rows = x_dim;
    mul(*tmp2, *tmp1, *P);
    cd(*P, *tmp2);

    // P<- (P+P')/2 (symétrisation pour éviter les erreurs d'arrondi)
    refd(refP, *P);
    refP.transpose();
    add(*P, *P, refP);
    mul(*P, *P, 0.5);
}