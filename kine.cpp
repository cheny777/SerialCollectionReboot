
#include "Kine.h"

using namespace ROBOTC;
using namespace ROBOTCF;
using namespace ROBOTE;
using namespace ROBOTM;

#include "stdio.h"
#include "math.h"


CKine::CKine()
{
    for(long i = 0; i < 8 ; i ++)
    {
        arm_jd[i].a_pre = 0;
        arm_jd[i].alf_pre = 0;
        arm_jd[i].d = 0;
        arm_jd[i].theta = 0;
        arm_jd[i].sin_theta = 0;
        arm_jd[i].cos_theta = 1;
        JointTran(&arm_jd[i],&arm_tr[i]);
    }
    j123_solve_paratag = false;
    j123_last_is_degen = false;

    refvel[0] = 0;
    refvel[1] = 0;
    refvel[2] = 0;
    refvel[3] = 0;
    refvel[4] = 0;
    refvel[5] = 0;


    refacc[0] = 0;
    refacc[1] = 0;
    refacc[2] = 0;
    refacc[3] = 0;
    refacc[4] = 0;
    refacc[5] = 0;

}

void CKine::SetRefJoint(Vect6 j)
{
    j123_ref[0] = j[0];
    j123_ref[1] = j[1];
    j123_ref[2] = j[2];


    j456_ref[0] = j[3];
    j456_ref[1] = j[4];
    j456_ref[2] = j[5];


    refvel[0] = 0;
    refvel[1] = 0;
    refvel[2] = 0;
    refvel[3] = 0;
    refvel[4] = 0;
    refvel[5] = 0;

}

CKine::~CKine()
{

}

void  CKine::GetWristPos(Vect3 xyz)
{
    xyz[0] = arm_rb[3][0][3];
    xyz[1] = arm_rb[3][1][3];
    xyz[2] = arm_rb[3][2][3];
}

void CKine::GetTcpGes(Matrix3 m)
{
    for(long i = 0; i < 3 ; i ++)
    {
        for(long j = 0; j < 3 ; j ++)
        {
            m[i][j] =  arm_rb[6][i][j];
        }
    }
}


void CKine::GetRobotFlangePosGes(Matrix4 m)
{
    for(long i = 0; i < 3 ; i ++)
    {
        for(long j = 0; j < 4 ; j ++)
        {
            m[i][j] =  arm_rb[5][i][j];
        }
    }
    m[3][0] = 0;
    m[3][1] = 0;
    m[3][2] = 0;
    m[3][3] = 1.0;
}

void CKine::GetTcpPosGes(Matrix4 m)
{
    for(long i = 0; i < 3 ; i ++)
    {
        for(long j = 0; j < 4 ; j ++)
        {
            m[i][j] =  arm_rb[6][i][j];
        }
    }
    m[3][0] = 0;
    m[3][1] = 0;
    m[3][2] = 0;
    m[3][3] = 1.0;
}

void CKine::GetTcpPos(Vect3 xyz)
{
    xyz[0] = arm_rb[6][0][3];
    xyz[1] = arm_rb[6][1][3];
    xyz[2] = arm_rb[6][2][3];
}


bool CKine::GetTcpGes(Vect3 rpy,long * degen)
{
    Matrix3 tmp;
    this->GetTcpGes(tmp);
    return Matrix2RPY(tmp,rpy,degen);
}

bool CKine::GetWristGes(Vect3 rpy,long * degen)
{
    Matrix3 tmp;

    for(long i = 0; i < 3 ; i ++)
    {
        for(long j = 0; j < 3 ; j ++)
        {
            tmp[i][j] =  arm_rb[5][i][j];
        }
    }
    return Matrix2RPY(tmp,rpy,degen);
}

bool CKine::OnBaseJointPath(long serial,Vect3 pos)
{
    if(serial <= 0 || serial > 7)
    {
        return false;
    }
    long ser = serial-1;
    Vect3 tmp;
    if(ser < 0)
    {
        return false;
    }

    tmp[0] = arm_rb[ser][0][0] * pos[0] + arm_rb[ser][0][1] * pos[1] + arm_rb[ser][0][2] * pos[2] + arm_rb[ser][0][3];
    tmp[1] = arm_rb[ser][1][0] * pos[0] + arm_rb[ser][1][1] * pos[1] + arm_rb[ser][1][2] * pos[2] + arm_rb[ser][1][3];
    tmp[2] = arm_rb[ser][2][0] * pos[0] + arm_rb[ser][2][1] * pos[1] + arm_rb[ser][2][2] * pos[2] + arm_rb[ser][2][3];
    pos[0] = tmp[0];
    pos[1] = tmp[1];
    pos[2] = tmp[2];
    return true;
}

void CKine::CalKine(Vect6 theta)
{
    int i;

    for( i = 0; i < 8 ; i ++)
    {
        if(i<6)
        {
            arm_jd[i].theta = theta[i];
            UpdateJointTran(&arm_jd[i],&arm_tr[i]);
        }

    }
    MatrixCpy(arm_tr[0],arm_rb[0]);
    MatrixMult(arm_rb[0],arm_tr[1],arm_rb[1]);
    MatrixMult(arm_rb[1],arm_tr[2],arm_rb[2]);
    MatrixMult(arm_rb[2],arm_tr[3],arm_rb[3]);
    MatrixMult(arm_rb[3],arm_tr[4],arm_rb[4]);
    MatrixMult(arm_rb[4],arm_tr[5],arm_rb[5]);
    MatrixMult(arm_rb[5],arm_tr[6],arm_rb[6]);
    MatrixMult(arm_rb[6],arm_tr[7],arm_rb[7]);
}


bool CKine::SetLink(int linknum,double alf,double a,double d,double theta,double range_l,double range_h)
{
    if(linknum<0 || linknum > 5)
    {
        return false;
    }
    if(range_h <= range_l)
    {
        return false;
    }
    arm_jd[linknum].a_pre	= a;
    arm_jd[linknum].alf_pre = alf;
    arm_jd[linknum].d		= d;
    arm_jd[linknum].theta	= theta;
    arm_jd[linknum].range_l = range_l;
    arm_jd[linknum].range_h = range_h;
    JointTran(&arm_jd[linknum],&arm_tr[linknum]);
    return true;
}


bool CKine::SetTool(Vect3 xyz,Vect3 abc)
{
    Matrix3 toolges;
    unsigned char order[3];
    order[0] = 'Z';
    order[1] = 'Y';
    order[2] = 'Z';
    Euler2Matrix(order,abc[0],abc[1],abc[2],toolges);
    Rotate2Tran(toolges,arm_tr[6]);
    arm_tr[6][3][0] = 0;
    arm_tr[6][3][1] = 0;
    arm_tr[6][3][2] = 0;
    arm_tr[6][3][3] = 1;
    arm_tr[6][0][3] = xyz[0];
    arm_tr[6][1][3] = xyz[1];
    arm_tr[6][2][3] = xyz[2];
    return true;
}



bool CKine::SetTool(Matrix4 toolgespos)
{
    long i,j;
    for(i = 0; i < 3 ; i ++)
    {
        for(j = 0; j < 4 ; j ++)
        {
            arm_tr[6][i][j] = toolgespos[i][j];
        }
    }
    arm_tr[6][3][0] = 0;
    arm_tr[6][3][1] = 0;
    arm_tr[6][3][2] = 0;
    arm_tr[6][3][3] = 1;
    return true;
}
