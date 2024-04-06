#include "Ekf.hpp"

Matrix Ekf::tmp1;
Matrix Ekf::tmp2;
Matrix Ekf::feedM;
Vector Ekf::feedV;
LdlMatrix Ekf::tmpLdl;
SymMatrix Ekf::tmpSym;
Matrix Ekf::Fx_val;
Matrix Ekf::Fu_val;
Vector Ekf::utmp;
Matrix Ekf::K;
Vector Ekf::y;

unsigned int Ekf::instance_count = 0;

uint_fast8_t Ekf::max_x_dim = 0;
uint_fast8_t Ekf::max_u_dim = 0;
uint_fast8_t Ekf::max_z_dim = 0;
uint_fast8_t Ekf::max_dim = 0;

Ekf::Ekf()
{
    instance_count++;
}

void Ekf::allocTmp(const uint_fast8_t dim)
{
    Ekf::tmp1.alloc(dim, dim);
    Ekf::tmp2.alloc(dim, dim);
}

void Ekf::alloc(uint_fast8_t x_dim, uint_fast8_t u_dim, uint_fast8_t z_num)
{
    // set the parameters
    this->x_dim = x_dim;
    this->u_dim = u_dim;
    this->z_number = z_num;

    // allocate the memory
    this->x.alloc(x_dim);
    this->P.alloc(x_dim);
    this->u.alloc(u_dim);

    this->Q.alloc(u_dim);
    this->I.alloc(x_dim);

    this->z_dim = new uint_fast8_t[z_num];
    this->h_val = new Vector[z_num];
    this->z = new Vector[z_num];
    this->R = new SymMatrix[z_num];
    this->S = new LdlMatrix[z_num];
    this->h_predicted = new bool[z_num];
    this->S_computed = new bool[z_num];
    this->H_val = new Matrix[z_num];

    if (x_dim > max_x_dim)
        this->Fx_val.alloc(x_dim, x_dim);
    if (x_dim > max_x_dim || u_dim > max_u_dim)
        this->Fu_val.alloc(max(x_dim, max_x_dim), max(u_dim, max_u_dim));
    if (u_dim > max_u_dim)
        this->utmp.alloc(u_dim);

    uint8_t max_dim_here = (x_dim > u_dim) ? x_dim : u_dim;
    if (max_dim_here > max_dim)
        allocTmp(max_dim_here);
    
    Ekf::tmpLdl.data = Ekf::tmp1.data;
    Ekf::K.data = Ekf::tmp2.data;

    if (max_dim_here > max_dim)
        max_dim = max_dim_here;

    if (x_dim > max_x_dim)
        max_x_dim = x_dim;

    if (u_dim > max_u_dim)
        max_u_dim = u_dim;

    this->I.fill(1);
}

void Ekf::alloc(uint_fast8_t z_dim, uint_fast8_t iz)
{
    this->z_dim[iz] = z_dim;
    this->z[iz].alloc(z_dim);
    this->h_val[iz].alloc(z_dim);
    this->R[iz].alloc(z_dim);
    this->S[iz].alloc(z_dim);
    this->H_val[iz].alloc(z_dim, x_dim);
    this->S_computed[iz] = false;
    this->h_predicted[iz] = false;

    this->R[iz].fill(0);

    if (z_dim > max_dim)
        allocTmp(z_dim);
    
    Ekf::tmpLdl.data = Ekf::tmp1.data;
    Ekf::K.data = Ekf::tmp2.data;

    if (z_dim > max_z_dim_here)
        max_z_dim_here = z_dim;

    if (max_z_dim_here > max_z_dim)
    {
        tmpSym.alloc(max_z_dim_here);
        y.alloc(max_z_dim_here);
        max_z_dim = max_z_dim_here;
    }

    if (z_dim > max_dim)
        max_dim = z_dim;
}

void Ekf::alloc(uint_fast8_t x_dim, uint_fast8_t u_dim, uint_fast8_t z_num, uint_fast8_t z_dim[])
{
    alloc(x_dim, u_dim, z_num);
    for (uint_fast8_t i = 0; i < z_num; i++)
        alloc(z_dim[i], i);
}

Ekf::Ekf(uint_fast8_t x_dim, uint_fast8_t u_dim, uint_fast8_t z_num)
{
    alloc(x_dim, u_dim, z_num);
}

void Ekf::setTransitionFunction(Matrix_f2 f)
{
    this->f = f;
}

Ekf::Ekf(uint_fast8_t x_dim, uint_fast8_t u_dim, Matrix_f2 f, uint_fast8_t z_num)
{
    instance_count++;
    alloc(x_dim, u_dim, z_num);
    setTransitionFunction(f);
}

void Ekf::setMeasurementFunction(Matrix_f1 h, uint_fast8_t z_dim, uint_fast8_t iz)
{
    alloc(z_dim, iz);
    if (this->h == nullptr)
    {
        this->h = new Matrix_f1[z_number];
    }

    this->h[iz] = h;
}

void Ekf::setMeasurementFunctions(Matrix_f1 h[], uint_fast8_t z_dim[])
{
    this->h = h;
    if (this->z_dim != nullptr)
        delete this->z_dim;
    this->z_dim = z_dim;
    for (uint_fast8_t i = 0; i < z_number; i++)
        alloc(z_dim[i], i);
}

Ekf::Ekf(uint_fast8_t x_dim, uint_fast8_t u_dim, Matrix_f2 f, uint_fast8_t z_dim, Matrix_f1 h)
{
    instance_count++;
    alloc(x_dim, u_dim, 1);
    setTransitionFunction(f);
    setMeasurementFunction(h, z_dim, 0);
}


void Ekf::setJacobianFunction_Fx(Matrix_f2 Fx)
{
    this->Fx = Fx;
}

void Ekf::setJacobianFunction_Fu(Matrix_f2 Fu)
{
    this->Fu = Fu;
}

void Ekf::setJacobianFunction_H(Matrix_f1 H, uint_fast8_t iz)
{
    if (this->H == nullptr)
        this->H = new Matrix_f1[z_number];

    this->H[iz] = H;
}

void Ekf::setJacobianFunctions_H(Matrix_f1 H[])
{
    this->H = H;
}

Ekf::Ekf(uint_fast8_t x_dim, uint_fast8_t u_dim, Matrix_f2 f, uint_fast8_t z_num, uint_fast8_t z_dim[], Matrix_f1 h[], Matrix_f2 Fx, Matrix_f2 Fu, Matrix_f1 H[])
{
    instance_count++;
    alloc(x_dim, u_dim, z_num, z_dim);
    setTransitionFunction(f);
    setMeasurementFunctions(h, z_dim);
    setJacobianFunction_Fx(Fx);
    setJacobianFunction_Fu(Fu);
    setJacobianFunctions_H(H);
}

Ekf::~Ekf()
{
    if (this->z != nullptr)
        delete[] this->z;
    if (this->R != nullptr)
        delete[] this->R;
    if (this->h_val != nullptr)
        delete[] this->h_val;
    if (this->S != nullptr)
        delete[] this->S;
    if (this->H_val != nullptr)
        delete[] this->H_val;
    if (this->h_predicted != nullptr)
        delete[] this->h_predicted;
    if (this->S_computed != nullptr)
        delete[] this->S_computed;
    if (this->h != nullptr)
        delete[] this->h;
    if (this->H != nullptr)
        delete[] this->H;

    if (this->z_dim != nullptr)
        delete[] this->z_dim;

    instance_count--;
    if (instance_count == 0)
    {
        Ekf::tmpLdl.data = nullptr;
        Ekf::K.data = nullptr;
    }
}

///////////////////////////////
///// protected functions /////
///////////////////////////////

void Ekf::finite_diff_Fx(const uint_fast8_t i, const data_type eps)
{
    vector::cd(tmp1, tmp2);

    tmp1.data[i] += eps;

    feedV = tmp1;
    f(tmp1, u, c);

    vector::sub(tmp1, tmp1, x);
    vector::mul(tmp1, tmp1, 1 / eps);

    for (uint_fast8_t j = 0; j < x_dim; j++)
        Fx_val.data[j * Fx_val.cols + i] = tmp1.data[j];
}

void Ekf::finite_diff_Fu(const uint_fast8_t i, const data_type eps)
{
    vector::cd(utmp, u);

    utmp.data[i] += eps;

    feedV = tmp1;
    f(tmp2, utmp, c);

    vector::sub(tmp1, tmp1, x);
    vector::mul(tmp1, tmp1, 1 / eps);

    for (uint_fast8_t j = 0; j < x_dim; j++)
        Fu_val.data[j * Fu_val.cols + i] = tmp1.data[j];
}

void Ekf::finite_diff_H(const uint_fast8_t iz, const uint_fast8_t i, const data_type eps)
{
    vector::cd(tmp1, x);

    tmp1.data[i] += eps;

    feedV = tmp2;
    h[iz](tmp1, c);

    vector::sub(tmp2, tmp2, h_val[iz]);
    vector::mul(tmp2, tmp2, 1 / eps);

    for (uint_fast8_t j = 0; j < z_dim[iz]; j++)
        H_val[iz].data[j * H_val[iz].cols + i] = tmp2.data[j];
}

void Ekf::finite_diff_Fx()
{
    for (uint_fast8_t i = 0; i < x_dim; i++)
        finite_diff_Fx(i);
}

void Ekf::finite_diff_Fu()
{
    utmp.size = u_dim;
    for (uint_fast8_t i = 0; i < u_dim; i++)
        finite_diff_Fu(i);
}

void Ekf::finite_diff_H(const uint_fast8_t iz)
{
    tmp2.size = z_dim[iz];
    for (uint_fast8_t i = 0; i < x_dim; i++)
        finite_diff_H(iz, i);
}

void Ekf::compute_Fx_Fu()
{
    uint_fast16_t tmp2_size = tmp2.size;
    uint_fast16_t tmp1_size = tmp1.size;
    tmp2.size = x_dim;
    tmp1.size = x_dim;

    Fx_val.rows = x_dim;
    Fx_val.cols = x_dim;
    Fu_val.rows = x_dim;
    Fu_val.cols = u_dim;

    if (Fx != nullptr)
    {
        feedM = Fx_val;
        Fx(x, u, c);
    }
    else
        finite_diff_Fx();

    if (Fu != nullptr)
    {
        feedM = Fu_val;
        Fu(x, u, c);
    }
    else
        finite_diff_Fu();

    tmp2.size = tmp2_size;
    tmp1.size = tmp1_size;
}

void Ekf::compute_S(const uint_fast8_t iz)
{
    bool use_finite_diff = true;
    if (H != nullptr)
        if (H[iz] != nullptr)
            use_finite_diff = false;

    if (!use_finite_diff)
    {
        feedM = H_val[iz];
        H[iz](x, c);
    }
    else
        finite_diff_H(iz);

    // S<- H*P*H'+R
    tmp1.cols = x_dim;
    tmp1.rows = z_dim[iz];
    sym_matrix::mul(tmp1, H_val[iz], P);
    H_val[iz].transpose();
    sym_matrix::mul(S[iz], tmp1, H_val[iz]);
    vector::add(S[iz], R[iz], S[iz]);
    S_computed[iz] = true;
}

///////////////////////////////
///// public functions ////////
///////////////////////////////

void Ekf::predict()
{
    vector::cd(tmp2, x);

    // x<- f(x,u)
    feedV = x;
    f(tmp2, u, c);
    compute_Fx_Fu();

    // P<- Fx*P*Fx'
    tmp1.cols = x_dim;
    tmp1.rows = x_dim;
    sym_matrix::mul(tmp1, Fx_val, P);
    Fx_val.transpose();
    sym_matrix::mul(P, tmp1, Fx_val);
    Fx_val.transpose();

    // tmpLdl<- Fu*Q*Fu'
    tmp2.cols = u_dim;
    tmp2.rows = x_dim;
    diag_matrix::mul(tmp2, Fu_val, Q);
    Fu_val.transpose();
    sym_matrix::mul(tmpLdl, tmp2, Fu_val);
    Fu_val.transpose();

    tmpLdl.size = x_dim * (x_dim + 1) / 2;
    // P<- P+tmpLdl
    vector::add(P, P, tmpLdl);

    // make all h_predicted to false
    for (uint_fast8_t i = 0; i < z_number; i++)
    {
        h_predicted[i] = false;
        S_computed[i] = false;
    }
}

void Ekf::predictMeasurement(const uint_fast8_t iz)
{
    feedV = h_val[iz]; // set h_val[iz] as output
    h[iz](x, c);
    h_predicted[iz] = true;
    S_computed[iz] = false;
}

data_type Ekf::mahalanobis(const Vector &z, const SymMatrix &zR, const uint_fast8_t iz)
{
    if (!h_predicted[iz])
        predictMeasurement(iz);

    if (!S_computed[iz])
        compute_S(iz);

    // tmpLdl <- S + R
    vector::add(tmpLdl, S[iz], zR);
    tmpLdl.order = z_dim[iz];
    tmpSym.order = z_dim[iz];

    // tmpSym <- (S+R)^-1
    ldl_matrix::inv(tmpSym, tmpLdl);

    // y <- z-h(x)
    y.size = z_dim[iz];
    vector::sub(y, z, h_val[iz]);

    tmp1.size = z_dim[iz];
    // tmp1 <- y'*cov_inv
    sym_matrix::mul(tmp1, tmpSym, y);

    // return y'*cov_inv*y
    data_type res = 0;
    vector::mul(res, y, tmp1);

    return res;
}

// String vec2str(Vector &v)
// {
//   String s = "";
//   for (int i = 0; i < v.size; i++)
//   {
//     s += String(v.data[i], 3) + " ";
//   }
//   return s;
// }

void Ekf::update(const uint_fast8_t iz)
{
    if (!h_predicted[iz])
        predictMeasurement(iz);

    // y<- z-h(x)
    y.size = z_dim[iz];
    
    vector::sub(y, z[iz], h_val[iz]);

    if (!S_computed[iz])
    {
        compute_S(iz);
    }

    // K<- P*H'*S^-1
    tmp1.cols = z_dim[iz];
    tmp1.rows = x_dim;
    sym_matrix::mul(tmp1, P, H_val[iz]);
    H_val[iz].transpose();

    tmp2.cols = z_dim[iz];
    tmp2.rows = z_dim[iz];
    tmpSym.order = z_dim[iz];
    ldl_matrix::inv(tmpSym, S[iz]);
    K.rows = x_dim;
    K.cols = z_dim[iz];
    sym_matrix::mul(K, tmp1, tmpSym);

    // x<- x+K*y
    matrix::mul(tmp1, K, y);
    vector::add(x, x, tmp1);

    // P<- (I-K*H)*P
    tmp1.cols = x_dim;
    tmp1.rows = x_dim;
    tmp1.size = x_dim * x_dim;
    matrix::mul(tmp1, K, H_val[iz]);
    diag_matrix::sub(tmp1, I, tmp1);
    tmp2.cols = x_dim;
    tmp2.rows = x_dim;

    tmpLdl.size = x_dim * (x_dim + 1) / 2;
    sym_matrix::mul(tmpLdl, tmp1, P);
    vector::cd(P, tmpLdl);
}