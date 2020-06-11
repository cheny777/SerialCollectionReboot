#ifndef RITOOLS_H
#define RITOOLS_H



#include "RDef.h"

namespace RobotComm
{
bool IsZero(double v);
double CycDiff(double v1,double v2);
double CycDiffC(double v1,double * v2);
}

namespace RobotMatrix{
bool VectNorm(Vect3 v);
void Vect3Copy(Vect3 src,Vect3 dst);
void Vect6Copy(Vect6 src,Vect6 dst);
void MatrixCpy(Matrix6 src,Matrix6 dst);
void MatrixCpy(Matrix4 src,Matrix4 dst);
void MatrixCpy(Matrix3 src,Matrix3 dst);
void MatrixMult(Matrix6  l,Matrix6  r,Matrix6  result);
void MatrixMult(Matrix4  l,Matrix4  r,Matrix4  result);
void MatrixMult(Matrix3  l,Matrix3  r,Matrix3  result);
void MatrixVectMult(Matrix6  l,Vect6 r,Vect6  result);
void MatrixVectMult(Matrix4  l,Vect4 r,Vect4  result);
void MatrixVectMult(Matrix3  l,Vect3 r,Vect3  result);
void TranMatrixInverse(Matrix4 t, Matrix4 inv);
void RotateMatrixInverse(Matrix4 t, Matrix4 inv);
void Rotate2Tran(Matrix3 r,Matrix4 t);
void Tran2Rotate(Matrix4 t,Matrix3 r);
double MatrixDeterminant(Matrix6 arcs,int order);
void MatrixCofactor(Matrix6 arcs,int order,Matrix6 ans);
bool MatrixInverse(Matrix6 m,Matrix6 invm);
bool MatrixInverse3(Matrix3 m,Matrix3 invm);
bool MatrixInverse4(Matrix4 m,Matrix4 invm);
bool MatrixInverse6(Matrix6 m,Matrix6 invm);
void VectCrossMult(Vect3  L,Vect3  R,Vect3  Result);
double VectDotMult(Vect3  L,Vect3  R);
void JointMap(Matrix6 m,Vect6 base,Vect6 jointv,Vect6 mapjv);
bool FittingPlane(long snum,double * x,double * y,double * z,double ret_abc[3]);
bool Solve3PlaneCross(Vect4 a_abcv,Vect4 b_abcv,Vect4 c_abcv,Vect3 ret_point);
}



namespace RobotEuler
{
bool Euler2Matrix(unsigned char order[3],double alf,double beta,double gama,Matrix3 m);
bool Euler2Matrix(unsigned char order[3],double sin_alf,double cos_alf,double sin_beta,double cos_beta,double sin_gama,double cos_gama,Matrix3 m);
bool Euler2Matrix(unsigned char order[3],double alf,double beta,double gama,Matrix4 m);
bool Euler2Matrix(unsigned char order[3],double sin_alf,double cos_alf,double sin_beta,double cos_beta,double sin_gama,double cos_gama,Matrix4 m);
bool Fix2Matrix(unsigned char order[3],double alf,double beta,double gama,Matrix3 m);
bool Fix2Matrix(unsigned char order[3],double sin_alf,double cos_alf,double sin_beta,double cos_beta,double sin_gama,double cos_gama,Matrix3 m);
bool Fix2Matrix(unsigned char order[3],double alf,double beta,double gama,Matrix4 m);
bool Fix2Matrix(unsigned char order[3],double sin_alf,double cos_alf,double sin_beta,double cos_beta,double sin_gama,double cos_gama,Matrix4 m);

bool RPY2Matrix(double sin_r,double cos_r,double sin_p,double cos_p,double sin_y,double cos_y,Matrix3 m);
bool RPY2Matrix(double sin_r,double cos_r,double sin_p,double cos_p,double sin_y,double cos_y,Matrix4 m);

bool RPY2Matrix(Vect3 rpy,Matrix3 m);
bool RPY2Matrix(Vect3 rpy,Matrix4 m);

bool Matrix2RPY(Matrix3 m,Vect3 rpy,long * degen_type);
bool Matrix2RPY4(Matrix4 m,Vect3 rpy,long * degen_type);

bool Matrix2EulerZYZ(Matrix3 m,Vect3 eular);
bool Matrix2EulerZYX(Matrix3 m,Vect3 eular);

void GetMinDiffRPYGroup(Vect3 in_rpy1,Vect3 in_rpy2,Vect3 out_rpy1,Vect3 out_rpy2);


void RPYMove(Vect3 rpy_in_start,Vect3 rpy_in_end,Vect3 rpy_out_start,Vect3 rpy_out_end,Matrix3 inversmove);
void RPYRMove(Vect3 rpy,Matrix3 inversmove);

bool RX4N(double ang,Matrix4 norm_rx);
bool RY4N(double ang,Matrix4 norm_ry);
bool RZ4N(double ang,Matrix4 norm_rz);
bool RZ3N(double ang,Matrix3 norm_rz);
void Rx(double v,Matrix3 m);
void Ry(double v,Matrix3 m);
void Rz(double v,Matrix3 m);

}

namespace RobotCommFunc
{
void JointTran(JointDsrpt *jd,Matrix4 *t);
void UpdateJointTran(JointDsrpt *jd,Matrix4 *t);
}


namespace ROBOTM= RobotMatrix;

namespace ROBOTE= RobotEuler;
namespace ROBOTCF= RobotCommFunc;
namespace ROBOTC= RobotComm;



#endif // RITOOLS_H
