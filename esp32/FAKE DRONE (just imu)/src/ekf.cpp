#include "ekf.hpp"

Matrix *Ekf::tmp1;
Matrix *Ekf::tmp2;
Matrix *Ekf::Min;
// Matrix Ekf::refP = Matrix();
Vector *Ekf::Vin;

Ekf::Ekf(Matrix_f2 f, Matrix_f1 h, int x_dim, int z_dim, int u_dim, Matrix_f2 Fx, Matrix_f2 Fu, Matrix_f1 H)
{
    this->x_dim = x_dim;
    this->z_dim = z_dim;
    this->u_dim = u_dim;

    this->h_val = new Vector(z_dim);

    this->u = new Vector(u_dim);
    this->utmp = new Vector(u_dim);
    this->z = new Vector(z_dim);
    this->R = new Matrix(z_dim, z_dim);
    this->R->set_eye();

    this->x = new Vector(x_dim);

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

    if (Ekf::tmp1 != nullptr)
    {
        if (max_dim > Ekf::tmp1->cols)
        {
            delete Ekf::tmp1;
            delete Ekf::tmp2;
            Ekf::tmp1 = new Matrix(max_dim, max_dim);
            Ekf::tmp2 = new Matrix(max_dim, max_dim);
        }
    }
    else
    {
        Ekf::tmp1 = new Matrix(max_dim, max_dim);
        Ekf::tmp2 = new Matrix(max_dim, max_dim);
    }
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
    delete this->utmp;
    delete this->h_val;
}

void Ekf::predict()
{
    cd(*tmp2, *x);

    // x<- f(x,u)
    Vin = x;
    f(*x, *u);

    uint16_t tmp2_size = tmp2->size;
    uint16_t tmp1_size = tmp1->size;
    tmp2->size = x_dim;
    tmp1->size = x_dim;

    if (Fx != nullptr)
    {
        Min = Fx_val;
        Fx(*x, *u);
    }
    else
        finite_diff_Fx();

    if (Fu != nullptr)
    {
        Min = Fu_val;
        Fu(*x, *u);
    }
    else
        finite_diff_Fu();

    tmp2->size = tmp2_size;
    tmp1->size = tmp1_size;

    // P<- Fx*P*Fx'
    tmp1->cols = x_dim;
    tmp1->rows = x_dim;
    mul(*tmp1, *Fx_val, *P);
    Fx_val->transpose();
    mul(*P, *tmp1, *Fx_val);
    Fx_val->transpose();

    // tmp2<- Fu*Q*Fu'

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
    Vin = h_val;
    h(*x);
    sub(*y, *z, *h_val);

    if (H != nullptr)
    {
        Min = H_val;
        H(*x);
    }
    else
    {
        uint16_t tmp2_size = tmp2->size;
        tmp2->size = z_dim;
        finite_diff_H();
        tmp2->size = tmp2_size;
    }

    // S<- H*P*H'+R
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
    mul(*tmp1, *K, *y);
    add(*x, *x, *tmp1);

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
    refd(*refP, *P);
    refP->transpose();
    add(*P, *P, *refP);
    mul(*P, *P, 0.5);
}

void Ekf::finite_diff_Fx(const uint8_t i, const data_type eps)
{
    cd(*tmp1, *tmp2);

    tmp1->data[i] += eps;

    Vin = tmp1;
    f(*tmp1, *u);

    sub(*tmp1, *tmp1, *x);
    mul(*tmp1, *tmp1, 1 / eps);

    for (uint_fast8_t j = 0; j < x_dim; j++)
        Fx_val->data[j * Fx_val->cols + i] = tmp1->data[j];
}

void Ekf::finite_diff_Fu(const uint8_t i, const data_type eps)
{
    cd(*utmp, *u);

    utmp->data[i] += eps;

    Vin = tmp1;
    f(*tmp2, *utmp);

    sub(*tmp1, *tmp1, *x);
    mul(*tmp1, *tmp1, 1 / eps);

    for (uint_fast8_t j = 0; j < x_dim; j++)
        Fu_val->data[j * Fu_val->cols + i] = tmp1->data[j];
}

void Ekf::finite_diff_H(const uint8_t i, const data_type eps)
{
    cd(*tmp1, *x);

    tmp1->data[i] += eps;

    Vin = tmp2;
    h(*tmp1);

    sub(*tmp2, *tmp2, *h_val);
    mul(*tmp2, *tmp2, 1 / eps);

    for (uint_fast8_t j = 0; j < z_dim; j++)
        H_val->data[j * H_val->cols + i] = tmp2->data[j];
}

void Ekf::finite_diff_Fx()
{
    for (uint_fast8_t i = 0; i < x_dim; i++)
        finite_diff_Fx(i);
}

void Ekf::finite_diff_Fu()
{
    for (uint_fast8_t i = 0; i < u_dim; i++)
        finite_diff_Fu(i);
}

void Ekf::finite_diff_H()
{
    for (uint_fast8_t i = 0; i < x_dim; i++)
        finite_diff_H(i);
}