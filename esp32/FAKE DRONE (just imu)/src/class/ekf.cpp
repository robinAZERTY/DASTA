#include "ekf.hpp"

Matrix *Ekf::tmp1;
Matrix *Ekf::tmp2;
Matrix *Ekf::feedM;
Vector *Ekf::feedV;

Ekf::Ekf(Matrix_f2 f, Matrix_f1 h[], uint_fast8_t x_dim, uint_fast8_t z_dim[], uint_fast8_t u_dim, Matrix_f2 Fx, Matrix_f2 Fu, Matrix_f1 H[], uint_fast8_t z_num)
{
    this->x_dim = x_dim;
    this->z_dim = z_dim;
    this->u_dim = u_dim;

    this->h_val = new Vector *[z_num];

    this->u = new Vector(u_dim);
    this->utmp = new Vector(u_dim);
    this->z = new Vector *[z_num];

    this->R = new Matrix *[z_num];

    uint_fast8_t max_z_dim = 0;
    for (uint_fast8_t i = 0; i < z_num; i++)
    {
        this->z[i] = new Vector(z_dim[i]);
        this->h_val[i] = new Vector(z_dim[i]);
        this->R[i] = new Matrix(z_dim[i], z_dim[i]);
        this->R[i]->fill(0);
        if (z_dim[i] > max_z_dim)
            max_z_dim = z_dim[i];
    }

    this->x = new Vector(x_dim);

    this->P = new Matrix(x_dim, x_dim);
    this->P->fill(0);

    this->h = h;
    this->H = H;
    this->Q = new Matrix(u_dim, u_dim);
    this->Q->fill(0);
    this->I = new Matrix(x_dim, x_dim);
    this->I->set_eye();

    this->f = f;
    this->Fx = Fx;
    this->Fu = Fu;
    this->K = new Matrix(x_dim, max_z_dim);
    this->S = new Matrix(max_z_dim, max_z_dim);
    

    this->Fx_val = new Matrix(x_dim, x_dim);
    this->Fu_val = new Matrix(x_dim, u_dim);
    this->H_val = new Matrix(max_z_dim, x_dim);

    this->y = new Vector(max_z_dim);

    int max_dim = (x_dim > max_z_dim) ? x_dim : max_z_dim;
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
    for (uint_fast8_t i = 0; i < z_number; i++)
    {
        delete this->z[i];
        delete this->h_val[i];
        delete this->R[i];
    }
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

void Ekf::compute_Fx_Fu()
{
    uint16_t tmp2_size = tmp2->size;
    uint16_t tmp1_size = tmp1->size;
    tmp2->size = x_dim;
    tmp1->size = x_dim;

    if (Fx != nullptr)
    {
        feedM = Fx_val;
        Fx(*x, *u, *c);
    }
    else
        finite_diff_Fx();

    if (Fu != nullptr)
    {
        feedM = Fu_val;
        Fu(*x, *u, *c);
    }
    else
        finite_diff_Fu();

    tmp2->size = tmp2_size;
    tmp1->size = tmp1_size;
}

void Ekf::predict()
{
    cd(*tmp2, *x);

    // x<- f(x,u)
    feedV = x;
    f(*tmp2, *u, *c);

    compute_Fx_Fu();

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

void Ekf::predictMeasurement(const uint8_t iz)
{
    feedV = h_val[iz]; // set h_val[iz] as output
    h[iz](*x, *c);
}

void Ekf::update(const uint8_t iz, const bool predictMeasurment)
{
    // y<- z-h(x)
    predictMeasurement(iz);

    y->size = z_dim[iz];
    sub(*y, *z[iz], *h_val[iz]);

    H_val->cols = x_dim;
    H_val->rows = z_dim[iz];
    
    bool use_finite_diff = true;
    if (H != nullptr)
        if (H[iz] != nullptr)
            use_finite_diff = false;

            
    if (!use_finite_diff)
    {
        feedM = H_val;
        H[iz](*x, *c);
    }
    else
    {
        uint16_t tmp2_size = tmp2->size;
        tmp2->size = z_dim[iz];
        finite_diff_H(iz);
        tmp2->size = tmp2_size;
    }

    // S<- H*P*H'+R
    tmp1->cols = x_dim;
    tmp1->rows = z_dim[iz];
    mul(*tmp1, *H_val, *P);
    H_val->transpose();
    S->cols = z_dim[iz];
    S->rows = z_dim[iz];
    mul(*S, *tmp1, *H_val);
    add(*S, *S, *R[iz]);

    // K<- P*H'*S^-1
    tmp1->cols = z_dim[iz];
    tmp1->rows = x_dim;
    mul(*tmp1, *P, *H_val);
    H_val->transpose();

    tmp2->cols = z_dim[iz];
    tmp2->rows = z_dim[iz];
    inv(*tmp2, *S);
    K->rows = x_dim;
    K->cols = z_dim[iz];
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
}

void Ekf::finite_diff_Fx(const uint8_t i, const data_type eps)
{
    cd(*tmp1, *tmp2);

    tmp1->data[i] += eps;

    feedV = tmp1;
    f(*tmp1, *u, *c);

    sub(*tmp1, *tmp1, *x);
    mul(*tmp1, *tmp1, 1 / eps);

    for (uint_fast8_t j = 0; j < x_dim; j++)
        Fx_val->data[j * Fx_val->cols + i] = tmp1->data[j];
}

void Ekf::finite_diff_Fu(const uint8_t i, const data_type eps)
{
    cd(*utmp, *u);

    utmp->data[i] += eps;

    feedV = tmp1;
    f(*tmp2, *utmp, *c);

    sub(*tmp1, *tmp1, *x);
    mul(*tmp1, *tmp1, 1 / eps);

    for (uint_fast8_t j = 0; j < x_dim; j++)
        Fu_val->data[j * Fu_val->cols + i] = tmp1->data[j];
}

void Ekf::finite_diff_H(const uint8_t iz, const uint8_t i, const data_type eps)
{
    cd(*tmp1, *x);

    tmp1->data[i] += eps;

    feedV = tmp2;
    h[iz](*tmp1, *c);

    sub(*tmp2, *tmp2, *(h_val[iz]));
    mul(*tmp2, *tmp2, 1 / eps);

    for (uint_fast8_t j = 0; j < z_dim[iz]; j++)
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

void Ekf::finite_diff_H(const uint8_t iz)
{
    for (uint_fast8_t i = 0; i < x_dim; i++)
        finite_diff_H(iz, i);
}