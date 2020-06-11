#ifndef KINE_H
#define KINE_H


#include "RITools.h"

#include "stdio.h"
#include "stdlib.h"


class CKine
{
public:
    CKine();
    virtual ~CKine();
    //	bool Solve(Matrix4 wrist_pos_ges,Vect6 ret_j);
    //   long SolveJ123(Matrix4 wrist_pos_ges,Vect3 j123,bool * degen_tag,long start_end_tag,Vect6 j_ref);
    //   long SolveInverKinematics_J123(Vect3 tgtt,Vect3 candidates[32],bool * degen_tag);
    //  long SolveInverKinematics_J456(Vect3 j_123,Matrix4 drct,Vect3 results[2],bool * degen_tag);
    bool SetLink(int linknum,double alf,double a,double d,double theta,double range_l,double range_h);
    bool SetTool(Vect3 xyz,Vect3 abc);
    bool SetTool(Matrix4 toolgespos);
    void CalKine(Vect6 theta);
    void GetTcpPos(Vect3 xyz);
    void GetTcpGes(Matrix3 m);
    void GetTcpPosGes(Matrix4 m);
    void GetRobotFlangePosGes(Matrix4 m);
    bool GetTcpGes(Vect3 rpy,long * degen);
    void GetWristPos(Vect3 xyz);
    bool GetWristGes(Vect3 rpy,long * degen);
    void SetRefJoint(Vect6 j);
    bool OnBaseJointPath(long serial,Vect3 pos);
protected:
    JointDsrpt	arm_jd[8];
    Matrix4		arm_tr[8];
    Matrix4		arm_rb[8];

    Vect3 j123_solve_para[7];
    bool j123_solve_paratag;

    Vect3 j123_ref;
    Vect3 j456_ref;
    bool  j123_last_is_degen;

    Vect6 refvel;
    Vect6 refacc;
};
#endif // KINE_H
