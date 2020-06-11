#ifndef RDEF_H
#define RDEF_H


#define ARMPI     3.1415926535897932384626433832795
#define ARM2PI    6.283185307179586476925286766559
#define ARMPI_HALF 1.5707963267948966192313216916398
#define ARMR2D    57.295779513082320876798154814105
#define ARMD2R    0.01745329251994329576923690768489
#define ARMRperS2DperMIN 3437.7467707849392526078892888463

#define DIM3 3
#define DIM4 4
#define DIM6 6
#define DIM8 8

typedef double Matrix6[DIM6][DIM6];
typedef double Matrix4[DIM4][DIM4];
typedef double Matrix3[DIM3][DIM3];

typedef double Vect7[7];
typedef double Vect6[DIM6];
typedef double Vect4[DIM4];
typedef double Vect3[DIM3];

typedef double Quaternion[DIM4];
typedef double GesturePV[DIM4];

typedef struct
{
    double	alf_pre;
    double	a_pre;
    double	d;
    double	theta;
    double sin_theta;
    double cos_theta;
    double sin_alf;
    double cos_alf;
    double range_l;
    double range_h;
}JointDsrpt;

typedef struct
{
    double	mass;
    Vect3	mass_core_pos;
    Matrix3 inertial_tensor;
}linkInertia;


#endif // RDEF_H
