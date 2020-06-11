#include "RITools.h"
#include "float.h"
#include "math.h"

namespace RobotComm
{

bool IsZero6(double v)
{
    if(v<=1.0E-6 &&v>=-1.0E-6 )
    {
        return true;
    }
    return false;
}

bool IsZero(double v)
{
    if(v<=DBL_EPSILON &&v>=-DBL_EPSILON )
    {
        return true;
    }
    return false;
}


double CycDiff(double v1,double v2)
{
    double diff = v2 - v1;

    while(diff > 0)
    {
        diff -= ARM2PI;
    }
    while(diff < 0)
    {
        diff += ARM2PI;
    }

    if(diff > ARMPI)
    {
        diff = diff - ARM2PI;
    }

    if(diff <- ARMPI )
    {
        diff = diff + ARM2PI;
    }
    return (diff);
}

double CycDiffC(double v1,double * v2)
{
    //double diff = v1 - *v2;
    double diff = v1 - *v2;

    while(diff > 0)
    {
        diff -= ARM2PI;
    }
    while(diff < 0)
    {
        diff += ARM2PI;
    }

    if(diff > ARMPI)
    {
        diff = diff - ARM2PI;
    }

    if(diff <- ARMPI )
    {
        diff = diff + ARM2PI;
    }
    * v2 = v1 - diff;
    return (diff);
}
}

namespace RobotMatrix{

bool IsZero(double v)
{
    if(v<=DBL_EPSILON &&v>=-DBL_EPSILON )
    {
        return true;
    }
    return false;
}

bool VectNorm(Vect3 v)
{
    double r = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if(IsZero(r))
    {
        return false;
    }
    v[0] /= r;
    v[1] /= r;
    v[2] /= r;
    return true;
}
double MatrixDeterminant(Matrix6 arcs,int order)
{

    double t;
    Matrix6 temp;
    int i,j,k;
    double ans = 0;
    if(order==1)
    {
        return arcs[0][0];
    }
    for(i=0;i<order;i++)
    {
        for(j=0;j<order-1;j++)
        {
            for(k=0;k<order-1;k++)
            {
                temp[j][k] = arcs[j+1][(k>=i)?k+1:k];

            }
        }
        t = MatrixDeterminant(temp,order-1);
        if(i%2==0)
        {
            ans += arcs[0][i]*t;
        }
        else
        {
            ans -=  arcs[0][i]*t;
        }
    }
    return ans;
}


void MatrixCofactor(Matrix6 arcs,int order,Matrix6 ans)
{
    if(order==1)
    {
        ans[0][0] = 1;
        return;
    }
    int i,j,k,t;
    Matrix6 temp;
    for(i=0;i<order;i++)
    {
        for(j=0;j<order;j++)
        {
            for(k=0;k<order-1;k++)
            {
                for(t=0;t<order-1;t++)
                {
                    temp[k][t] = arcs[k>=i?k+1:k][t>=j?t+1:t];
                }
            }
            ans[j][i]  =  MatrixDeterminant(temp,order-1);
            if((i+j)%2 == 1)
            {
                ans[j][i] = - ans[j][i];
            }
        }
    }
}

bool MatrixInverse3(Matrix3 m,Matrix3 invm)
{
    Matrix6 a,b;
    long i,j;
    long q = 3;
    for(i = 0; i < 6 ; i ++)
    {
        for(j = 0 ; j < 6 ; j ++)
        {
            a[i][j] = 0;
            if(i == j)
            {
                a[i][j] = 1.0;
            }
            if(i < q && j < q)
            {
                a[i][j] = m[i][j];
            }
        }
    }
    if( MatrixInverse(a,b) == false)
    {
        return false;
    }
    for(i = 0; i < q ; i ++)
    {
        for(j = 0 ; j < q ; j ++)
        {
            invm[i][j] = b[i][j];
        }
    }
    return true;
}

bool MatrixInverse4(Matrix4 m,Matrix4 invm)
{
    Matrix6 a,b;
    long i,j;
    long q = 4;
    for(i = 0; i < 6 ; i ++)
    {
        for(j = 0 ; j < 6 ; j ++)
        {
            a[i][j] = 0;
            if(i == j)
            {
                a[i][j] = 1.0;
            }
            if(i < q && j < q)
            {
                a[i][j] = m[i][j];
            }
        }
    }
    if( MatrixInverse(a,b) == false)
    {
        return false;
    }
    for(i = 0; i < q ; i ++)
    {
        for(j = 0 ; j < q ; j ++)
        {
            invm[i][j] = b[i][j];
        }
    }
    return true;
}
bool MatrixInverse6(Matrix6 m,Matrix6 invm)
{
    Matrix6 a,b;
    long i,j;
    long q = 6;
    for(i = 0; i < 6 ; i ++)
    {
        for(j = 0 ; j < 6 ; j ++)
        {
            a[i][j] = 0;
            if(i == j)
            {
                a[i][j] = 1.0;
            }
            if(i < q && j < q)
            {
                a[i][j] = m[i][j];
            }
        }
    }
    if( MatrixInverse(a,b) == false)
    {
        return false;
    }
    for(i = 0; i < q ; i ++)
    {
        for(j = 0 ; j < q ; j ++)
        {
            invm[i][j] = b[i][j];
        }
    }
    return true;
}

bool MatrixInverse(Matrix6 m,Matrix6 invm)
{
    Matrix6 astar;
    int		i,j;
    double d = MatrixDeterminant(m,6);
    if(IsZero(d))
    {
        return false;
    }
    else
    {
        MatrixCofactor(m,6,astar);
        for(i=0;i<6;i++)
        {
            for(j=0;j<6;j++)
            {
                invm[i][j] = astar[i][j]/d;
            }
        }
    }
    return true;
}

bool Solve3PlaneCross(Vect4 a_abcv,Vect4 b_abcv,Vect4 c_abcv,Vect3 ret_point)
{
    Matrix3 m,invm;
    Vect3   v;
    m[0][0] = a_abcv[0];
    m[0][1] = a_abcv[1];
    m[0][2] = a_abcv[2];

    m[1][0] = b_abcv[0];
    m[1][1] = b_abcv[1];
    m[1][2] = b_abcv[2];

    m[2][0] = c_abcv[0];
    m[2][1] = c_abcv[1];
    m[2][2] = c_abcv[2];

    v[0] = a_abcv[3];
    v[1] = b_abcv[3];
    v[2] = c_abcv[3];


    if( MatrixInverse3(m,invm) == false)
    {
        return false;
    }
    MatrixVectMult(invm,v,ret_point);
    return true;
}


void RotateMatrixInverse(Matrix4 t, Matrix4 inv)
{
    inv[0][0] =  t[0][0];
    inv[0][1] =  t[1][0];
    inv[0][2] =  t[2][0];

    inv[1][0] =  t[0][1];
    inv[1][1] =  t[1][1];
    inv[1][2] =  t[2][1];

    inv[2][0] =  t[0][2];
    inv[2][1] =  t[1][2];
    inv[2][2] =  t[2][2];
}

void TranMatrixInverse(Matrix4 t, Matrix4 inv)
{
    inv[0][0] =  t[0][0];
    inv[0][1] =  t[1][0];
    inv[0][2] =  t[2][0];

    inv[1][0] =  t[0][1];
    inv[1][1] =  t[1][1];
    inv[1][2] =  t[2][1];

    inv[2][0] =  t[0][2];
    inv[2][1] =  t[1][2];
    inv[2][2] =  t[2][2];

    inv[3][0] =  0;
    inv[3][1] =  0;
    inv[3][2] =  0;
    inv[3][3] =  1;

    Vect3 p;
    p[0] = t[0][3];
    p[1] = t[1][3];
    p[2] = t[2][3];

    inv[0][3] = - (inv[0][0] * p[0] + inv[0][1] * p[1] + inv[0][2] * p[2]);
    inv[1][3] = - (inv[1][0] * p[0] + inv[1][1] * p[1] + inv[1][2] * p[2]);
    inv[2][3] = - (inv[2][0] * p[0] + inv[2][1] * p[1] + inv[2][2] * p[2]);
}

void MatrixVectMult(Matrix6  l,Vect6 r,Vect6  result)
{
    int i,j,order;
    order = DIM6;
    for(i = 0; i < order ; i ++)
    {
        result[i] = l[i][0] * r[0];
        for(j = 1; j < order ; j ++)
        {
            result[i] += l[i][j] * r[j];
        }
    }
}

void MatrixVectMult(Matrix4  l,Vect4 r,Vect4  result)
{
    int i,j,order;
    order = DIM4;
    for(i = 0; i < order ; i ++)
    {
        result[i] = l[i][0] * r[0];
        for(j = 1; j < order ; j ++)
        {
            result[i] += l[i][j] * r[j];
        }
    }
}

void MatrixVectMult(Matrix3  l,Vect3 r,Vect3  result)
{
    int i,j,order;
    order = DIM3;
    for(i = 0; i < order ; i ++)
    {
        result[i] = l[i][0] * r[0];
        for(j = 1; j < order ; j ++)
        {
            result[i] += l[i][j] * r[j];
        }
    }
}

void MatrixMult(Matrix6  l,Matrix6  r,Matrix6  result)
{
    int i,j,k,order;
    order = DIM6;
    for(i = 0; i < order ; i ++)
    {
        for(j = 0 ; j < order ; j ++)
        {
            result[i][j] = l[i][0]*r[0][j];
            for(k = 1; k < order ; k ++)
            {
                result[i][j] += l[i][k]*r[k][j];
            }
        }
    }
}

void MatrixMult(Matrix4  l,Matrix4  r,Matrix4  result)
{
    int i,j,k,order;
    order = DIM4;
    for(i = 0; i < order ; i ++)
    {
        for(j = 0 ; j < order ; j ++)
        {
            result[i][j] = l[i][0]*r[0][j];
            for(k = 1; k < order ; k ++)
            {
                result[i][j] += l[i][k]*r[k][j];
            }
        }
    }
}

void MatrixMult(Matrix3  l,Matrix3  r,Matrix3  result)
{
    int i,j,k,order;
    order = DIM3;
    for(i = 0; i < order ; i ++)
    {
        for(j = 0 ; j < order ; j ++)
        {
            result[i][j] = l[i][0]*r[0][j];
            for(k = 1; k < order ; k ++)
            {
                result[i][j] += l[i][k]*r[k][j];
            }
        }
    }
}

void MatrixCpy(Matrix6 src,Matrix6 dst)
{
    int i,j,order;
    order = DIM6;
    for(i = 0; i < order ; i ++)
    {
        for(j = 0 ; j < order ; j ++)
        {
            dst[i][j] = src[i][j];
        }
    }
}
void MatrixCpy(Matrix4 src,Matrix4 dst)
{
    int i,j,order;
    order = DIM4;
    for(i = 0; i < order ; i ++)
    {
        for(j = 0 ; j < order ; j ++)
        {
            dst[i][j] = src[i][j];
        }
    }
}

void MatrixCpy(Matrix3 src,Matrix3 dst)
{
    int i,j,order;
    order = DIM3;
    for(i = 0; i < order ; i ++)
    {
        for(j = 0 ; j < order ; j ++)
        {
            dst[i][j] = src[i][j];
        }
    }
}

void Vect3Copy(Vect3 src,Vect3 dst)
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

void Vect6Copy(Vect6 src,Vect6 dst)
{
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
    dst[3] = src[3];
    dst[4] = src[4];
    dst[5] = src[5];
}

void Rotate2Tran(Matrix3 r,Matrix4 t)
{
    int i,j;
    for(i = 0; i < 3 ; i ++)
    {
        for(j = 0; j < 3 ; j ++)
        {
            t[i][j] = r[i][j];
        }
    }
}
void Tran2Rotate(Matrix4 t,Matrix3 r)
{
    int i,j;
    for(i = 0; i < 3 ; i ++)
    {
        for(j = 0; j < 3 ; j ++)
        {
            r[i][j] = t[i][j];
        }
    }
}


void VectCrossMult(Vect3  L,Vect3  R,Vect3  Result)
{
    Result[0] = (L)[1]*(R)[2] - (L)[2]*(R)[1];
    Result[1] = -((L)[0]*(R)[2] - (L)[2]*(R)[0]);
    Result[2] = (L)[0]*(R)[1] - (L)[1]*(R)[0];
}

double VectDotMult(Vect3  L,Vect3  R)
{
    return ( L[0] * R[0] + L[1] * R[1] + L[2] * R[2]);
}


void JointMap(Matrix6 m,Vect6 base,Vect6 jointv,Vect6 mapjv)
{
    Vect6 tmp;
    tmp[0] = jointv[0] - base[0];
    tmp[1] = jointv[1] - base[1];
    tmp[2] = jointv[2] - base[2];
    tmp[3] = jointv[3] - base[3];
    tmp[4] = jointv[4] - base[4];
    tmp[5] = jointv[5] - base[5];
    for(long i = 0; i < 6 ; i ++)
    {
        mapjv[i] = 0;
        for(long j = 0; j < 6 ; j ++)
        {
            mapjv[i] += m[i][j]* tmp[j];
        }
    }
    mapjv[0] += base[0];
    mapjv[1] += base[1];
    mapjv[2] += base[2];
    mapjv[3] += base[3];
    mapjv[4] += base[4];
    mapjv[5] += base[5];
}

bool FittingPlane(long snum,double * x,double * y,double * z,double ret_abc[3])
{
    double xx = 0;
    double yy = 0;

    double xy = 0;
    double xz = 0;
    double yz = 0;

    double sx  = 0;
    double sy  = 0;
    double sz  = 0;
    long i,j;

    for( i = 0; i < snum; i ++)
    {
        sx += x[i];
        sy += y[i];
        sz += z[i];
        xx += x[i]*x[i];
        yy += y[i]*y[i];
        xy += x[i]*y[i];
        xz += x[i]*z[i];
        yz += y[i]*z[i];
    }
    Matrix3 m;
    m[0][0] = xx; m[0][1] = xy; m[0][2] = sx;
    m[1][0] = xy; m[1][1] = yy; m[1][2] = sy;
    m[2][0] = sx; m[2][1] = sy; m[2][2] = snum;

    Vect3 r;
    r[0] = xz;
    r[1] = yz;
    r[2] = sz;

    Matrix3 invm;
    Matrix6 intm;
    for( i = 0; i < 6; i ++)
    {
        for(j = 0; j < 6 ; j ++)
        {
            if(i<3 && j < 3)
            {
                intm[i][j] = m[i][j];
            }
            else
            {
                intm[i][j] = 0;
                if(i == j)
                {
                    intm[i][j] = 1;
                }
            }
        }
    }
    Matrix6 invtm;
    if(MatrixInverse(intm,invtm) == false)
    {
        return false;
    }
    for( i = 0; i < 3; i ++)
    {
        for(j = 0; j < 3 ; j ++)
        {
            invm[i][j] = invtm[i][j];
        }
    }
    Vect3 ret;
    MatrixVectMult(invm,r,ret);

    ret_abc[0] = ret[0];
    ret_abc[1] = ret[1];
    ret_abc[2] = ret[2];
    return true;

}

}

namespace RobotEuler
{
void Rx(double sinv,double cosv,Matrix3 m)
{
    for(long i = 0; i < 3 ; i ++)
    {
        for(long j = 0; j < 3 ; j ++)
        {
            if(i == j)
            {
                m[i][j] = 1;
            }
            else
            {
                m[i][j] = 0;
            }
        }
    }
    m[1][1] = cosv;
    m[1][2] = - sinv;
    m[2][1] = sinv;
    m[2][2] = cosv;
}
void Ry(double sinv,double cosv,Matrix3 m)
{
    for(long i = 0; i < 3 ; i ++)
    {
        for(long j = 0; j < 3 ; j ++)
        {
            if(i == j)
            {
                m[i][j] = 1;
            }
            else
            {
                m[i][j] = 0;
            }
        }
    }
    m[0][0] = cosv;
    m[0][2] = sinv;
    m[2][0] = -sinv;
    m[2][2] = cosv;
}

void Rz(double sinv,double cosv,Matrix3 m)
{
    for(long i = 0; i < 3 ; i ++)
    {
        for(long j = 0; j < 3 ; j ++)
        {
            if(i == j)
            {
                m[i][j] = 1;
            }
            else
            {
                m[i][j] = 0;
            }
        }
    }
    m[0][0] = cosv;
    m[0][1] = -sinv;
    m[1][0] = sinv;
    m[1][1] = cosv;
}
void Rx(double v,Matrix3 m)
{
    double sv = sin(v);
    double cv = cos(v);
    for(long i = 0; i < 3 ; i ++)
    {
        for(long j = 0; j < 3 ; j ++)
        {
            if(i == j)
            {
                m[i][j] = 1;
            }
            else
            {
                m[i][j] = 0;
            }
        }
    }
    m[1][1] = cv;
    m[1][2] = - sv;
    m[2][1] = sv;
    m[2][2] = cv;
}


void Ry(double v,Matrix3 m)
{
    double sv = sin(v);
    double cv = cos(v);
    for(long i = 0; i < 3 ; i ++)
    {
        for(long j = 0; j < 3 ; j ++)
        {
            if(i == j)
            {
                m[i][j] = 1;
            }
            else
            {
                m[i][j] = 0;
            }
        }
    }
    m[0][0] = cv;
    m[0][2] = sv;
    m[2][0] = -sv;
    m[2][2] = cv;
}


void Rz(double v,Matrix3 m)
{
    double sv = sin(v);
    double cv = cos(v);
    for(long i = 0; i < 3 ; i ++)
    {
        for(long j = 0; j < 3 ; j ++)
        {
            if(i == j)
            {
                m[i][j] = 1;
            }
            else
            {
                m[i][j] = 0;
            }
        }
    }
    m[0][0] = cv;
    m[0][1] = -sv;
    m[1][0] = sv;
    m[1][1] = cv;
}

bool RX4N(double ang,Matrix4 norm_rx)
{
    double sv = sin(ang);
    double cv = cos(ang);

    for(long i = 0; i < 4 ; i ++)
    {
        for(long j = 0; j < 4 ; j ++)
        {
            if(i == j)
            {
                norm_rx[i][j] = 1;
            }
            else
            {
                norm_rx[i][j] = 0;
            }
        }
    }
    norm_rx[1][1] = cv;
    norm_rx[1][2] = - sv;
    norm_rx[2][1] = sv;
    norm_rx[2][2] = cv;
    return true;
}

bool RY4N(double ang,Matrix4 norm_ry)
{
    double sv = sin(ang);
    double cv = cos(ang);
    for(long i = 0; i < 4 ; i ++)
    {
        for(long j = 0; j < 4; j ++)
        {
            if(i == j)
            {
                norm_ry[i][j] = 1;
            }
            else
            {
                norm_ry[i][j] = 0;
            }
        }
    }
    norm_ry[0][0] = cv;
    norm_ry[0][2] = sv;
    norm_ry[2][0] = -sv;
    norm_ry[2][2] = cv;
    return true;
}

bool RZ3N(double ang,Matrix3 norm_rz)
{
    double sv = sin(ang);
    double cv = cos(ang);
    for(long i = 0; i < 3 ; i ++)
    {
        for(long j = 0; j < 3 ; j ++)
        {
            if(i == j)
            {
                norm_rz[i][j] = 1;
            }
            else
            {
                norm_rz[i][j] = 0;
            }
        }
    }
    norm_rz[0][0] = cv;
    norm_rz[0][1] = -sv;
    norm_rz[1][0] = sv;
    norm_rz[1][1] = cv;
    return true;
}

bool RZ4N(double ang,Matrix4 norm_rz)
{
    double sv = sin(ang);
    double cv = cos(ang);
    for(long i = 0; i < 4 ; i ++)
    {
        for(long j = 0; j < 4 ; j ++)
        {
            if(i == j)
            {
                norm_rz[i][j] = 1;
            }
            else
            {
                norm_rz[i][j] = 0;
            }
        }
    }
    norm_rz[0][0] = cv;
    norm_rz[0][1] = -sv;
    norm_rz[1][0] = sv;
    norm_rz[1][1] = cv;
    return true;
}

bool IsZero(double v)
{
    if(v<=1E-5 &&v>=-1E-5 )
    {
        return true;
    }
    return false;
}


bool RPY2Matrix(Vect3 rpy,Matrix3 m)
{
    double sin_r, cos_r, sin_p, cos_p, sin_y, cos_y;
    sin_r = sin(rpy[0]);
    cos_r = cos(rpy[0]);
    sin_p = sin(rpy[1]);
    cos_p = cos(rpy[1]);
    sin_y = sin(rpy[2]);
    cos_y = cos(rpy[2]);
    Matrix3 mz;
    Matrix3 my;
    Matrix3 mx;
    Matrix3 mzy;
    Rz(sin_r,cos_r,mz);
    Ry(sin_p,cos_p,my);
    Rx(sin_y,cos_y,mx);
    RobotMatrix::MatrixMult(mz,my,mzy);
    RobotMatrix::MatrixMult(mzy,mx,m);
    return true;
}

bool RPY2Matrix(Vect3 rpy,Matrix4 m)
{
    double sin_r, cos_r, sin_p, cos_p, sin_y, cos_y;
    sin_r = sin(rpy[0]);
    cos_r = cos(rpy[0]);
    sin_p = sin(rpy[1]);
    cos_p = cos(rpy[1]);
    sin_y = sin(rpy[2]);
    cos_y = cos(rpy[2]);
    Matrix3 mz;
    Matrix3 my;
    Matrix3 mx;
    Matrix3 mzy;
    Matrix3 mzyx;

    Rz(sin_r,cos_r,mz);
    Ry(sin_p,cos_p,my);
    Rx(sin_y,cos_y,mx);
    RobotMatrix::MatrixMult(mz,my,mzy);
    RobotMatrix::MatrixMult(mzy,mx,mzyx);
    for(long i = 0 ; i < 3 ; i ++)
    {
        for(long j = 0; j < 3 ; j ++)
        {
            m[i][j] = mzyx[i][j];
        }
    }
    return true;

}

bool RPY2Matrix(double sin_r,double cos_r,double sin_p,double cos_p,double sin_y,double cos_y,Matrix3 m)
{
    Matrix3 mz;
    Matrix3 my;
    Matrix3 mx;
    Matrix3 mzy;
    Rz(sin_r,cos_r,mz);
    Ry(sin_p,cos_p,my);
    Rx(sin_y,cos_y,mx);
    RobotMatrix::MatrixMult(mz,my,mzy);
    RobotMatrix::MatrixMult(mzy,mx,m);
    return true;
}
bool RPY2Matrix(double sin_r,double cos_r,double sin_p,double cos_p,double sin_y,double cos_y,Matrix4 m)
{
    Matrix3 mz;
    Matrix3 my;
    Matrix3 mx;
    Matrix3 mzy;
    Matrix3 mzyx;

    Rz(sin_r,cos_r,mz);
    Ry(sin_p,cos_p,my);
    Rx(sin_y,cos_y,mx);
    RobotMatrix::MatrixMult(mz,my,mzy);
    RobotMatrix::MatrixMult(mzy,mx,mzyx);
    for(long i = 0 ; i < 3 ; i ++)
    {
        for(long j = 0; j < 3 ; j ++)
        {
            m[i][j] = mzyx[i][j];
        }
    }
    return true;
}




bool Fix2Matrix(unsigned char order[3],double sin_alf,double cos_alf,double sin_beta,double cos_beta,double sin_gama,double cos_gama,Matrix3 m)
{
    Matrix3 m1;
    Matrix3 m2;
    Matrix3 m3;
    Matrix3 m12;
    if(    !(order[0] == 'x' || order[0] == 'y' || order[0] == 'z' || order[0] == 'X' || order[0] == 'Y' || order[0] == 'Z')
           || !(order[1] == 'x' || order[1] == 'y' || order[1] == 'z' || order[1] == 'X' || order[1] == 'Y' || order[1] == 'Z')
           || !(order[2] == 'x' || order[2] == 'y' || order[2] == 'z' || order[2] == 'X' || order[2] == 'Y' || order[2] == 'Z')
           )
    {
        return false;
    }

    if(order[0] == 'x' || order[0] == 'X')
    {
        Rx(sin_alf,cos_alf,m1);
    }
    else
    {
        if(order[0] == 'y' || order[0] == 'Y')
        {
            Ry(sin_alf,cos_alf,m1);
        }
        else
        {
            if(order[0] == 'z' || order[0] == 'Z')
            {
                Rz(sin_alf,cos_alf,m1);
            }
        }
    }

    if(order[1] == 'x' || order[1] == 'X')
    {
        Rx(sin_beta,cos_beta,m2);
    }
    else
    {
        if(order[1] == 'y' || order[1] == 'Y')
        {
            Ry(sin_beta,cos_beta,m2);
        }
        else
        {
            if(order[1] == 'z' || order[1] == 'Z')
            {
                Rz(sin_beta,cos_beta,m2);
            }
        }
    }

    if(order[2] == 'x' || order[2] == 'X')
    {
        Rx(sin_gama,cos_gama,m3);
    }
    else
    {
        if(order[2] == 'y' || order[2] == 'Y')
        {
            Ry(sin_gama,cos_gama,m3);
        }
        else
        {
            if(order[2] == 'z' || order[2] == 'Z')
            {
                Rz(sin_gama,cos_gama,m3);
            }
        }
    }
    RobotMatrix::MatrixMult(m1,m2,m12);
    RobotMatrix::MatrixMult(m12,m3,m);
    return true;
}

bool Fix2Matrix(unsigned char order[3],double alf,double beta,double gama,Matrix3 m)
{
    Matrix3 m1;
    Matrix3 m2;
    Matrix3 m3;
    Matrix3 m12;
    if(    !(order[0] == 'x' || order[0] == 'y' || order[0] == 'z' || order[0] == 'X' || order[0] == 'Y' || order[0] == 'Z')
           || !(order[1] == 'x' || order[1] == 'y' || order[1] == 'z' || order[1] == 'X' || order[1] == 'Y' || order[1] == 'Z')
           || !(order[2] == 'x' || order[2] == 'y' || order[2] == 'z' || order[2] == 'X' || order[2] == 'Y' || order[2] == 'Z')
           )
    {
        return false;
    }

    if(order[0] == 'x' || order[0] == 'X')
    {
        Rx(alf,m1);
    }
    else
    {
        if(order[0] == 'y' || order[0] == 'Y')
        {
            Ry(alf,m1);
        }
        else
        {
            if(order[0] == 'z' || order[0] == 'Z')
            {
                Rz(alf,m1);
            }
        }
    }



    if(order[1] == 'x' || order[1] == 'X')
    {
        Rx(beta,m2);
    }
    else
    {
        if(order[1] == 'y' || order[1] == 'Y')
        {
            Ry(beta,m2);
        }
        else
        {
            if(order[1] == 'z' || order[1] == 'Z')
            {
                Rz(beta,m2);
            }
        }
    }

    if(order[2] == 'x' || order[2] == 'X')
    {
        Rx(gama,m3);
    }
    else
    {
        if(order[2] == 'y' || order[2] == 'Y')
        {
            Ry(gama,m3);
        }
        else
        {
            if(order[2] == 'z' || order[2] == 'Z')
            {
                Rz(gama,m3);
            }
        }
    }
    RobotMatrix::MatrixMult(m1,m2,m12);
    RobotMatrix::MatrixMult(m12,m3,m);
    return true;
}


bool Fix2Matrix(unsigned char order[3],double sin_alf,double cos_alf,double sin_beta,double cos_beta,double sin_gama,double cos_gama,Matrix4 m)
{
    Matrix3 m1;
    Matrix3 m2;
    Matrix3 m3;
    Matrix3 m12;
    Matrix3 m123;
    if(    !(order[0] == 'x' || order[0] == 'y' || order[0] == 'z' || order[0] == 'X' || order[0] == 'Y' || order[0] == 'Z')
           || !(order[1] == 'x' || order[1] == 'y' || order[1] == 'z' || order[1] == 'X' || order[1] == 'Y' || order[1] == 'Z')
           || !(order[2] == 'x' || order[2] == 'y' || order[2] == 'z' || order[2] == 'X' || order[2] == 'Y' || order[2] == 'Z')
           )
    {
        return false;
    }

    if(order[0] == 'x' || order[0] == 'X')
    {
        Rx(sin_alf,cos_alf,m1);
    }
    else
    {
        if(order[0] == 'y' || order[0] == 'Y')
        {
            Ry(sin_alf,cos_alf,m1);
        }
        else
        {
            if(order[0] == 'z' || order[0] == 'Z')
            {
                Rz(sin_alf,cos_alf,m1);
            }
        }
    }

    if(order[1] == 'x' || order[1] == 'X')
    {
        Rx(sin_beta,cos_beta,m2);
    }
    else
    {
        if(order[1] == 'y' || order[1] == 'Y')
        {
            Ry(sin_beta,cos_beta,m2);
        }
        else
        {
            if(order[1] == 'z' || order[1] == 'Z')
            {
                Rz(sin_beta,cos_beta,m2);
            }
        }
    }

    if(order[2] == 'x' || order[2] == 'X')
    {
        Rx(sin_gama,cos_gama,m3);
    }
    else
    {
        if(order[2] == 'y' || order[2] == 'Y')
        {
            Ry(sin_gama,cos_gama,m3);
        }
        else
        {
            if(order[2] == 'z' || order[2] == 'Z')
            {
                Rz(sin_gama,cos_gama,m3);
            }
        }
    }
    RobotMatrix::MatrixMult(m1,m2,m12);
    RobotMatrix::MatrixMult(m12,m3,m123);
    for(long i = 0 ; i < 3 ; i ++)
    {
        for(long j = 0; j < 3 ; j ++)
        {
            m[i][j] = m123[i][j];
        }
    }
    return true;
}

bool Fix2Matrix(unsigned char order[3],double alf,double beta,double gama,Matrix4 m)
{
    Matrix3 m1;
    Matrix3 m2;
    Matrix3 m3;
    Matrix3 m12;
    Matrix3 m123;
    if(    !(order[0] == 'x' || order[0] == 'y' || order[0] == 'z' || order[0] == 'X' || order[0] == 'Y' || order[0] == 'Z')
           || !(order[1] == 'x' || order[1] == 'y' || order[1] == 'z' || order[1] == 'X' || order[1] == 'Y' || order[1] == 'Z')
           || !(order[2] == 'x' || order[2] == 'y' || order[2] == 'z' || order[2] == 'X' || order[2] == 'Y' || order[2] == 'Z')
           )
    {
        return false;
    }

    if(order[1] == 'x' || order[1] == 'X')
    {
        Rx(beta,m2);
    }
    else
    {
        if(order[1] == 'y' || order[1] == 'Y')
        {
            Ry(beta,m2);
        }
        else
        {
            if(order[1] == 'z' || order[1] == 'Z')
            {
                Rz(beta,m2);
            }
        }
    }

    if(order[2] == 'x' || order[2] == 'X')
    {
        Rx(gama,m3);
    }
    else
    {
        if(order[2] == 'y' || order[2] == 'Y')
        {
            Ry(gama,m3);
        }
        else
        {
            if(order[2] == 'z' || order[2] == 'Z')
            {
                Rz(gama,m3);
            }
        }
    }
    RobotMatrix::MatrixMult(m1,m2,m12);
    RobotMatrix::MatrixMult(m12,m3,m123);
    for(long i = 0 ; i < 3 ; i ++)
    {
        for(long j = 0; j < 3 ; j ++)
        {
            m[i][j] = m123[i][j];
        }
    }
    return true;
}
/*/
    /*/

bool Euler2Matrix(unsigned char order[3],double sin_alf,double cos_alf,double sin_beta,double cos_beta,double sin_gama,double cos_gama,Matrix3 m)
{
    Matrix3 m1;
    Matrix3 m2;
    Matrix3 m3;
    Matrix3 m21;
    if(    !(order[0] == 'x' || order[0] == 'y' || order[0] == 'z' || order[0] == 'X' || order[0] == 'Y' || order[0] == 'Z')
           || !(order[1] == 'x' || order[1] == 'y' || order[1] == 'z' || order[1] == 'X' || order[1] == 'Y' || order[1] == 'Z')
           || !(order[2] == 'x' || order[2] == 'y' || order[2] == 'z' || order[2] == 'X' || order[2] == 'Y' || order[2] == 'Z')
           )
    {
        return false;
    }

    if(order[0] == 'x' || order[0] == 'X')
    {
        Rx(sin_alf,cos_alf,m1);
    }
    else
    {
        if(order[0] == 'y' || order[0] == 'Y')
        {
            Ry(sin_alf,cos_alf,m1);
        }
        else
        {
            if(order[0] == 'z' || order[0] == 'Z')
            {
                Rz(sin_alf,cos_alf,m1);
            }
        }
    }

    if(order[1] == 'x' || order[1] == 'X')
    {
        Rx(sin_beta,cos_beta,m2);
    }
    else
    {
        if(order[1] == 'y' || order[1] == 'Y')
        {
            Ry(sin_beta,cos_beta,m2);
        }
        else
        {
            if(order[1] == 'z' || order[1] == 'Z')
            {
                Rz(sin_beta,cos_beta,m2);
            }
        }
    }

    if(order[2] == 'x' || order[2] == 'X')
    {
        Rx(sin_gama,cos_gama,m3);
    }
    else
    {
        if(order[2] == 'y' || order[2] == 'Y')
        {
            Ry(sin_gama,cos_gama,m3);
        }
        else
        {
            if(order[2] == 'z' || order[2] == 'Z')
            {
                Rz(sin_gama,cos_gama,m3);
            }
        }
    }
    RobotMatrix::MatrixMult(m2,m1,m21);
    RobotMatrix::MatrixMult(m3,m21,m);
    return true;
}

bool Euler2Matrix(unsigned char order[3],double alf,double beta,double gama,Matrix3 m)
{
    Matrix3 m1;
    Matrix3 m2;
    Matrix3 m3;
    Matrix3 m21;
    if(    !(order[0] == 'x' || order[0] == 'y' || order[0] == 'z' || order[0] == 'X' || order[0] == 'Y' || order[0] == 'Z')
           || !(order[1] == 'x' || order[1] == 'y' || order[1] == 'z' || order[1] == 'X' || order[1] == 'Y' || order[1] == 'Z')
           || !(order[2] == 'x' || order[2] == 'y' || order[2] == 'z' || order[2] == 'X' || order[2] == 'Y' || order[2] == 'Z')
           )
    {
        return false;
    }

    if(order[0] == 'x' || order[0] == 'X')
    {
        Rx(alf,m1);
    }
    else
    {
        if(order[0] == 'y' || order[0] == 'Y')
        {
            Ry(alf,m1);
        }
        else
        {
            if(order[0] == 'z' || order[0] == 'Z')
            {
                Rz(alf,m1);
            }
        }
    }



    if(order[1] == 'x' || order[1] == 'X')
    {
        Rx(beta,m2);
    }
    else
    {
        if(order[1] == 'y' || order[1] == 'Y')
        {
            Ry(beta,m2);
        }
        else
        {
            if(order[1] == 'z' || order[1] == 'Z')
            {
                Rz(beta,m2);
            }
        }
    }

    if(order[2] == 'x' || order[2] == 'X')
    {
        Rx(gama,m3);
    }
    else
    {
        if(order[2] == 'y' || order[2] == 'Y')
        {
            Ry(gama,m3);
        }
        else
        {
            if(order[2] == 'z' || order[2] == 'Z')
            {
                Rz(gama,m3);
            }
        }
    }
    RobotMatrix::MatrixMult(m2,m1,m21);
    RobotMatrix::MatrixMult(m3,m21,m);
    return true;
}


bool Euler2Matrix(unsigned char order[3],double sin_alf,double cos_alf,double sin_beta,double cos_beta,double sin_gama,double cos_gama,Matrix4 m)
{
    Matrix3 m1;
    Matrix3 m2;
    Matrix3 m3;
    Matrix3 m21;
    Matrix3 m321;
    if(    !(order[0] == 'x' || order[0] == 'y' || order[0] == 'z' || order[0] == 'X' || order[0] == 'Y' || order[0] == 'Z')
           || !(order[1] == 'x' || order[1] == 'y' || order[1] == 'z' || order[1] == 'X' || order[1] == 'Y' || order[1] == 'Z')
           || !(order[2] == 'x' || order[2] == 'y' || order[2] == 'z' || order[2] == 'X' || order[2] == 'Y' || order[2] == 'Z')
           )
    {
        return false;
    }

    if(order[0] == 'x' || order[0] == 'X')
    {
        Rx(sin_alf,cos_alf,m1);
    }
    else
    {
        if(order[0] == 'y' || order[0] == 'Y')
        {
            Ry(sin_alf,cos_alf,m1);
        }
        else
        {
            if(order[0] == 'z' || order[0] == 'Z')
            {
                Rz(sin_alf,cos_alf,m1);
            }
        }
    }

    if(order[1] == 'x' || order[1] == 'X')
    {
        Rx(sin_beta,cos_beta,m2);
    }
    else
    {
        if(order[1] == 'y' || order[1] == 'Y')
        {
            Ry(sin_beta,cos_beta,m2);
        }
        else
        {
            if(order[1] == 'z' || order[1] == 'Z')
            {
                Rz(sin_beta,cos_beta,m2);
            }
        }
    }

    if(order[2] == 'x' || order[2] == 'X')
    {
        Rx(sin_gama,cos_gama,m3);
    }
    else
    {
        if(order[2] == 'y' || order[2] == 'Y')
        {
            Ry(sin_gama,cos_gama,m3);
        }
        else
        {
            if(order[2] == 'z' || order[2] == 'Z')
            {
                Rz(sin_gama,cos_gama,m3);
            }
        }
    }
    RobotMatrix::MatrixMult(m2,m1,m21);
    RobotMatrix::MatrixMult(m3,m21,m321);


    for(long i = 0 ; i < 3 ; i ++)
    {
        for(long j = 0; j < 3 ; j ++)
        {
            m[i][j] = m321[i][j];
        }
    }
    return true;
}

bool Euler2Matrix(unsigned char order[3],double alf,double beta,double gama,Matrix4 m)
{
    Matrix3 m1;
    Matrix3 m2;
    Matrix3 m3;
    Matrix3 m21;
    Matrix3 m321;
    if(    !(order[0] == 'x' || order[0] == 'y' || order[0] == 'z' || order[0] == 'X' || order[0] == 'Y' || order[0] == 'Z')
           || !(order[1] == 'x' || order[1] == 'y' || order[1] == 'z' || order[1] == 'X' || order[1] == 'Y' || order[1] == 'Z')
           || !(order[2] == 'x' || order[2] == 'y' || order[2] == 'z' || order[2] == 'X' || order[2] == 'Y' || order[2] == 'Z')
           )
    {
        return false;
    }

    if(order[1] == 'x' || order[1] == 'X')
    {
        Rx(beta,m2);
    }
    else
    {
        if(order[1] == 'y' || order[1] == 'Y')
        {
            Ry(beta,m2);
        }
        else
        {
            if(order[1] == 'z' || order[1] == 'Z')
            {
                Rz(beta,m2);
            }
        }
    }

    if(order[2] == 'x' || order[2] == 'X')
    {
        Rx(gama,m3);
    }
    else
    {
        if(order[2] == 'y' || order[2] == 'Y')
        {
            Ry(gama,m3);
        }
        else
        {
            if(order[2] == 'z' || order[2] == 'Z')
            {
                Rz(gama,m3);
            }
        }
    }
    RobotMatrix::MatrixMult(m2,m1,m21);
    RobotMatrix::MatrixMult(m3,m21,m321);
    for(long i = 0 ; i < 3 ; i ++)
    {
        for(long j = 0; j < 3 ; j ++)
        {
            m[i][j] = m321[i][j];
        }
    }
    return true;
}


bool Matrix2RPY4(Matrix4 m,Vect3 rpy,long * degen_type)
{
    double r = sqrt(m[0][0]*m[0][0] + m[1][0] * m[1][0]);
    rpy[1] = atan2(-m[2][0],r);
    if(IsZero(r))
    {
        * degen_type = 1;
        rpy[0] = 0;
        if(rpy[1] > 0 )
        {
            rpy[2] =  atan2(m[0][1],m[1][1]);
        }
        else
        {
            rpy[2] =  -atan2(m[0][1],m[1][1]);
        }

        return false;
    }
    else
    {
        *degen_type = 0;
        rpy[0] = atan2(m[1][0],m[0][0]);
        rpy[2] = atan2(m[2][1],m[2][2]);
        return true;

    }
}

bool Matrix2RPY(Matrix3 m,Vect3 rpy,long * degen_type)
{
    double r = sqrt(m[0][0]*m[0][0] + m[1][0] * m[1][0]);
    rpy[1] = atan2(-m[2][0],r);
    if(IsZero(r))
    {
        * degen_type = 1;
        rpy[0] = 0;
        if(rpy[1] > 0 )
        {
            rpy[2] =  atan2(m[0][1],m[1][1]);
        }
        else
        {
            rpy[2] =  -atan2(m[0][1],m[1][1]);
        }

        return false;
    }
    else
    {
        *degen_type = 0;
        rpy[0] = atan2(m[1][0],m[0][0]);
        rpy[2] = atan2(m[2][1],m[2][2]);
        return true;

    }
}
bool Matrix2EulerZYX(Matrix3 m,Vect3 eular)
{
    double r = sqrt(m[0][0]*m[0][0] + m[1][0] * m[1][0]);
    eular[1] = atan2(-m[2][0],r);
    if(IsZero(r))
    {
        eular[0] = 0;
        eular[2] = atan2(m[0][1],m[1][1]);
        return false;
    }
    else
    {
        eular[0] = atan2(m[1][0],m[0][0]);
        eular[2] = atan2(m[2][1],m[2][2]);
        return true;

    }
}

bool Matrix2EulerZYZ(Matrix3 m,Vect3 eular)
{
    eular[1] = atan2(sqrt(m[2][0] * m[2][0] + m[2][1] * m[2][1] ),m[2][2]);
    double bsin = sin(eular[1]);
    if(IsZero(bsin))
    {
        eular[0] = 0;
        eular[2] = atan2(m[0][1],-m[0][0]);
        return false;
    }
    eular[0] = atan2(m[1][2]/bsin,m[0][2]/bsin);
    eular[2] = atan2(m[2][1]/bsin,-m[2][0]/bsin);
    return true;
}


void GetMinDiffRPYGroup(Vect3 in_rpy1,Vect3 in_rpy2,Vect3 out_rpy1,Vect3 out_rpy2)
{
    Vect3 rpy1[2];
    Vect3 rpy2[2];
    {//��������Ƕȵ����RPY����
        {
            rpy1[0][0] = in_rpy1[0];
            rpy1[0][1] = in_rpy1[1];
            rpy1[0][2] = in_rpy1[2];
        }
        {
            if(in_rpy1[0] > 0)
            {
                rpy1[1][0] = in_rpy1[0] - ARMPI;
            }
            else
            {
                rpy1[1][0] = in_rpy1[0] + ARMPI;
            }
            if(in_rpy1[1] > 0)
            {
                rpy1[1][1] = ARMPI - in_rpy1[1];
            }
            else
            {
                rpy1[1][1] = - ARMPI - in_rpy1[1];
            }
            if(in_rpy1[2] > 0)
            {
                rpy1[1][2] = in_rpy1[2] - ARMPI;
            }
            else
            {
                rpy1[1][2] = in_rpy1[2] + ARMPI;
            }

        }
        {
            rpy2[0][0] = in_rpy2[0];
            rpy2[0][1] = in_rpy2[1];
            rpy2[0][2] = in_rpy2[2];
        }
        {
            if(in_rpy2[0] > 0)
            {
                rpy2[1][0] = in_rpy2[0] - ARMPI;
            }
            else
            {
                rpy2[1][0] = in_rpy2[0] + ARMPI;
            }
            if(in_rpy2[1] > 0)
            {
                rpy2[1][1] = ARMPI - in_rpy2[1];
            }
            else
            {
                rpy2[1][1] = - ARMPI - in_rpy2[1];
            }
            if(in_rpy2[2] > 0)
            {
                rpy2[1][2] = in_rpy2[2] - ARMPI;
            }
            else
            {
                rpy2[1][2] = in_rpy2[2] + ARMPI;
            }
        }
    }

    long i,j,mini,minj;
    double minv = 10000;
    double d;
    for(i = 0; i < 2 ; i ++)
    {
        for(j = 0 ; j < 2 ; j ++)
        {
            d = fabs(RobotComm::CycDiff(rpy1[i][0],rpy2[j][0]));
            d += fabs(RobotComm::CycDiff(rpy1[i][1],rpy2[j][1]));
            d += fabs(RobotComm::CycDiff(rpy1[i][2],rpy2[j][2]));

            if(d < minv)
            {
                d = minv;
                mini = i;
                minj = j;
            }
        }
    }


    out_rpy1[0] = rpy1[mini][0];
    out_rpy1[1] = rpy1[mini][1];
    out_rpy1[2] = rpy1[mini][2];

    out_rpy2[0] = rpy2[minj][0];
    out_rpy2[1] = rpy2[minj][1];
    out_rpy2[2] = rpy2[minj][2];
}


void RPYMove(Vect3 rpy_in_start,Vect3 rpy_in_end,Vect3 rpy_out_start,Vect3 rpy_out_end,Matrix3 inversmove)
{


    Vect3 in1,in2,ino1,ino2;

    ino1[0] = rpy_in_start[0];
    ino1[1] = rpy_in_start[1];
    ino1[2] = rpy_in_start[2];

    ino2[0] = rpy_in_end[0];
    ino2[1] = rpy_in_end[1];
    ino2[2] = rpy_in_end[2];
    GetMinDiffRPYGroup(rpy_in_start,rpy_in_end,in1,in2);

    RobotComm::CycDiffC(in1[0],&in2[0]);
    RobotComm::CycDiffC(in1[1],&in2[1]);
    RobotComm::CycDiffC(in1[2],&in2[2]);

    double rmid = ( in1[1] + in2[1] ) / 2;

    Matrix3 drot;
    int i,j;
    for( i = 0; i < 3 ; i ++)
    {
        for(j = 0; j < 3 ; j ++)
        {
            drot[i][j] = 0;
            inversmove[i][j] = 0;
        }
        drot[i][i] = 1;
        inversmove[i][i] = 1;
    }

    double sv = sin( - rmid);
    double cv = cos( - rmid);

    drot[0][0] = cv;
    drot[0][2] = sv;
    drot[2][0] = -sv;
    drot[2][2] = cv;

    inversmove[0][0] = cv;
    inversmove[0][2] = -sv;
    inversmove[2][0] = sv;
    inversmove[2][2] = cv;

    Matrix3  in1m,in2m,tin1m,tin2m;
    RPY2Matrix(in1,in1m);
    RPY2Matrix(in2,in2m);

    RobotMatrix::MatrixMult(drot,in1m,tin1m);
    RobotMatrix::MatrixMult(drot,in2m,tin2m);

    Vect3 outrpy1,outrpy2;
    long degen;
    Matrix2RPY(tin1m,outrpy1,&degen);
    Matrix2RPY(tin2m,outrpy2,&degen);
    GetMinDiffRPYGroup(outrpy1,outrpy2,rpy_out_start,rpy_out_end);
    RobotComm::CycDiffC(rpy_out_start[0],&rpy_out_end[0]);
    RobotComm::CycDiffC(rpy_out_start[1],&rpy_out_end[1]);
    RobotComm::CycDiffC(rpy_out_start[2],&rpy_out_end[2]);
}

void RPYRMove(Vect3 rpy,Matrix3 inversmove)
{
    Matrix3 inm,rm;
    RPY2Matrix(rpy,inm);
    RobotMatrix::MatrixMult(inversmove,inm,rm);
    long degen;
    Matrix2RPY(rm,rpy,&degen);
}
}


namespace RobotCommFunc
{
void JointTran(JointDsrpt *jd,Matrix4 *t)
{
    double alf,a,d,theta;
    alf				= jd->alf_pre;
    a				= jd->a_pre;
    d				= jd->d;
    theta			= jd->theta;
    jd->sin_theta	= sin(theta);
    jd->cos_theta	= cos(theta);
    jd->sin_alf		= sin(alf);
    jd->cos_alf		= cos(alf);

    double sa		= jd->sin_alf;
    double ca		= jd->cos_alf;
    double st		= sin(theta);
    double ct		= cos(theta);
    (*t)[0][0] = ct;
    (*t)[0][1] = -st;
    (*t)[0][2] = 0;
    (*t)[0][3] = a;
    (*t)[1][0] = st*ca;
    (*t)[1][1] = ct*ca;
    (*t)[1][2] = -sa;
    (*t)[1][3] = -sa*d;
    (*t)[2][0] = st*sa;
    (*t)[2][1] = ct*sa;
    (*t)[2][2] = ca;
    (*t)[2][3] = ca*d;
    (*t)[3][0] = 0;
    (*t)[3][1] = 0;
    (*t)[3][2] = 0;
    (*t)[3][3] = 1;
}

void UpdateJointTran(JointDsrpt *jd,Matrix4 *t)
{
    jd->sin_theta	= sin(jd->theta);
    jd->cos_theta	= cos(jd->theta);
    double st		= jd->sin_theta;
    double ct		= jd->cos_theta;
    double ca		= jd->cos_alf;
    double sa		= jd->sin_alf;
    (*t)[0][0] = ct;
    (*t)[0][1] = -st;
    (*t)[1][0] = st*ca;
    (*t)[1][1] = ct*ca;
    (*t)[2][0] = st*sa;
    (*t)[2][1] = ct*sa;
}
}
