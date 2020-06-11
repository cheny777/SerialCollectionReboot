#include "robotcfg.h"

#include "stdlib.h"
#include "string.h"
#include "stdio.h"


static CRobotCFG * ins_robot_cfg = NULL;

CRobotCFG * CRobotCFG::GetCFGIns()
{
    if(ins_robot_cfg == NULL)
    {
        ins_robot_cfg = new CRobotCFG;
    }
    return ins_robot_cfg;
}

void CRobotCFG::dmemcpy(double * t,double * s,long cnt)
{
    for(long i = 0; i < cnt ; i ++)
    {
        t[i] = s[i];
    }
}


CRobotCFG::CRobotCFG()
{
    long i = 0;
    for(i = 0; i < MAX_CFG_ITEM_SIZE ; i ++)
    {
        m_value_valid_tags[i] = false;
    }
    value_base = (unsigned char *)malloc(10240);

    long pos = 2;
    m_dh = (JointDsrpt  *)(&value_base[pos]);
    pos += sizeof(JointDsrpt)*6;
    m_inert = (linkInertia  *)(&value_base[pos]);
    pos += sizeof(linkInertia)*6;
    m_max_vel = (double * )(&value_base[pos]);
    pos += sizeof(double)*6;
    m_max_acc = (double * )(&value_base[pos]);
    pos += sizeof(double)*6;
    m_max_jerk = (double * )(&value_base[pos]);
    pos += sizeof(double)*6;
    m_max_torque = (double * )(&value_base[pos]);
    pos += sizeof(double)*6;
    m_joint_link =  (double * )(&value_base[pos]);
    pos += sizeof(double)*36;
    m_joint_base = (double * )(&value_base[pos]);
    pos += sizeof(double)*6;
    m_torque_current_foctor = (double * )(&value_base[pos]);
    pos += sizeof(double)*6;
    m_output_filter_para  = (double * )(&value_base[pos]);
    pos += sizeof(double)*4;
    m_pos_ges_para = (double * )(&value_base[pos]);
    pos += sizeof(double)*3;
    crc_end_pos = pos;
    crc_1 = (&value_base[pos]);
    pos += sizeof(unsigned char)*1;
    crc_2 = (&value_base[pos]);
}

CRobotCFG::~CRobotCFG()
{

}


bool CRobotCFG:: InGetJD( JointDsrpt * jd)
{
    if(jd == NULL)
    {
        return false;
    }
    if(m_value_valid_tags[0] == false)
    {
        return false;
    }
    memcpy(jd,m_dh,sizeof(JointDsrpt) * 6);
    return true;
}

bool CRobotCFG:: InGetInert( linkInertia * inert)
{
    if(inert == NULL)
    {
        return false;
    }
    if(m_value_valid_tags[1] == false)
    {
        return false;
    }
    memcpy(inert,m_inert,sizeof(linkInertia) * 6);
    return true;
}
bool CRobotCFG:: InGetMaxKineFeatures( Vect6 mv,Vect6 ma,Vect6 mj,Vect6 mt)
{
    if(m_value_valid_tags[2] == false)
    {
        return false;
    }
    if(m_value_valid_tags[3] == false)
    {
        return false;
    }
    if(m_value_valid_tags[4] == false)
    {
        return false;
    }
    if(m_value_valid_tags[5] == false)
    {
        return false;
    }
    dmemcpy(mv,m_max_vel, 6);
    dmemcpy(ma,m_max_acc, 6);
    dmemcpy(mj,m_max_jerk, 6);
    dmemcpy(mt,m_max_torque,6);
    return true;
}

bool CRobotCFG:: InGetLinkSetting( Matrix6 ll,Vect6 lb)
{
    if(m_value_valid_tags[6] == false)
    {
        return false;
    }
    if(m_value_valid_tags[7] == false)
    {
        return false;
    }
    long i,j;
    for(i = 0; i < 6 ; i ++)
    {
        for(j = 0; j < 6 ; j ++)
        {
            ll[i][j] = m_joint_link[i * 6 + j];
        }
    };
    dmemcpy(lb,m_joint_base,6);
    return true;
}
bool CRobotCFG:: InGetTCF(Vect6 tc)
{
    if(m_value_valid_tags[8] == false)
    {
        return false;
    }
    dmemcpy(tc,m_torque_current_foctor,6);
    return true;
}
bool CRobotCFG:: InGetOFilter(Vect4 of)
{
    if(m_value_valid_tags[9] == false)
    {
        return false;
    }
    dmemcpy(of,m_output_filter_para,4);
    return true;

}

bool CRobotCFG:: InGetPGPara(Vect3 pg)
{
    if(m_value_valid_tags[10] == false)
    {
        return false;
    }
    dmemcpy(pg,m_pos_ges_para,3);
    return true;
}


//////////////////////////



bool CRobotCFG:: InSetJD( JointDsrpt * jd)
{
    if(jd == NULL)
    {
        return false;
    }
    memcpy(m_dh,jd,sizeof(JointDsrpt) * 6);
    m_value_valid_tags[0] = true;
    return true;
}

bool CRobotCFG:: InSetInert( linkInertia * inert)
{
    if(inert == NULL)
    {
        return false;
    }

    memcpy(m_inert,inert,sizeof(linkInertia) * 6);

    m_value_valid_tags[1] = true;
    return true;
}
bool CRobotCFG:: InSetMaxKineFeatures( Vect6 mv,Vect6 ma,Vect6 mj,Vect6 mt)
{

    dmemcpy(m_max_vel,mv,6);
    dmemcpy(m_max_acc,ma,6);
    dmemcpy(m_max_jerk,mj,6);
    dmemcpy(m_max_torque,mt, 6);

    m_value_valid_tags[2] = true;
    m_value_valid_tags[3] = true;
    m_value_valid_tags[4] = true;
    m_value_valid_tags[5] = true;
    return true;
}

bool CRobotCFG:: InSetLinkSetting( Matrix6 ll,Vect6 lb)
{
    long i,j;
    for(i = 0; i < 6 ; i ++)
    {
        for(j = 0; j < 6 ; j ++)
        {
            m_joint_link[i * 6 + j] = ll[i][j];
        }
    };
    dmemcpy(m_joint_base,lb,6);
    m_value_valid_tags[6] = true;
    m_value_valid_tags[7] = true;
    return true;
}
bool CRobotCFG:: InSetTCF(Vect6 tc)
{
    dmemcpy(m_torque_current_foctor,tc,6);

    m_value_valid_tags[8] = true;
    return true;
}
bool CRobotCFG:: InSetOFilter(Vect4 of)
{
    dmemcpy(m_output_filter_para,of,4);
    m_value_valid_tags[9] = true;
    return true;

}

bool CRobotCFG:: InSetPGPara(Vect3 pg)
{
    dmemcpy(m_pos_ges_para,pg,3);
    m_value_valid_tags[10] = true;
    return true;
}
///////////////////////
bool CRobotCFG:: InLoad(char * path)
{
    long i;
    if(path == NULL)
    {
        return false;
    }
    FILE * fp = fopen(path,"rb");
    if(fp == NULL)
    {
        return false;
    }
    for(i = 0; i < crc_end_pos + 2 ; i ++)
    {
        fread(&value_base[i],1,1,fp);
    }
    fclose(fp);


    if( value_base[0] !=  ROB_CFG_VERSION1)
    {
        return false;
    }
    if( value_base[1] !=  ROB_CFG_VERSION2)
    {
        return false;
    }

    unsigned long v1 = 0;
    unsigned long v2 = 0;

    for(i = 0; i < crc_end_pos; i ++)
    {
        v1 += value_base[i];
        v2 += value_base[i]*7;
    }

    unsigned char cv1 = ( unsigned char)v1;
    unsigned char cv2 = ( unsigned char)v2;

    if(* crc_1 != cv1)
    {
        return false;
    }
    if(* crc_2 != cv2)
    {
        return false;
    }

    for(i = 0; i < MAX_CFG_ITEM_SIZE ; i ++)
    {
        m_value_valid_tags[i] = true;
    }
    return true;
}

bool CRobotCFG:: InSave(char * path)
{
    long i;
    for(i = 0; i < MAX_CFG_ITEM_SIZE ; i ++)
    {
        if(m_value_valid_tags[i] == false)
        {
            return false;
        }
    }
    if(path == NULL)
    {
        return false;
    }
    FILE * fp = fopen(path,"wb");
    if(fp == NULL)
    {
        return false;
    }

    value_base[0] =  ROB_CFG_VERSION1;
    value_base[1] =  ROB_CFG_VERSION2;

    unsigned long v1 = 0;
    unsigned long v2 = 0;

    for(i = 0; i < crc_end_pos; i ++)
    {
        v1 += value_base[i];
        v2 += value_base[i]*7;
    }
    * crc_1 = ( unsigned char)v1;
    * crc_2 = ( unsigned char)v2;

    for(i = 0; i < crc_end_pos + 2 ; i ++)
    {
        fwrite(&value_base[i],1,1,fp);
    }
    fclose(fp);
    return true;
}




bool CRobotCFG:: OnGetJD( JointDsrpt * jd)
{
    CRobotCFG * t = GetCFGIns();
    return t->InGetJD(jd);
}

bool CRobotCFG:: OnGetInert( linkInertia * inert)
{
    CRobotCFG * t = GetCFGIns();
    return t->InGetInert(inert);
}
bool CRobotCFG:: OnGetMaxKineFeatures( Vect6 mv,Vect6 ma,Vect6 mj,Vect6 mt)
{
    CRobotCFG * t = GetCFGIns();
    return t->InGetMaxKineFeatures( mv,ma,mj,mt);
}
bool CRobotCFG:: OnGetLinkSetting( Matrix6 ll,Vect6 lb)
{
    CRobotCFG * t = GetCFGIns();
    return t->InGetLinkSetting(  ll, lb);
}
bool CRobotCFG:: OnGetTCF(Vect6 tc)
{
    CRobotCFG * t = GetCFGIns();
    return t->InGetTCF(tc);
}
bool CRobotCFG:: OnGetOFilter(Vect4 of)
{
    CRobotCFG * t = GetCFGIns();
    return t->InGetOFilter(of);
}
bool CRobotCFG:: OnGetPGPara(Vect3 pg)
{
    CRobotCFG * t = GetCFGIns();
    return t->InGetPGPara(pg);
}
bool CRobotCFG:: OnSetJD( JointDsrpt * jd)
{
    CRobotCFG * t = GetCFGIns();
    return t->InSetJD( jd);
}
bool CRobotCFG:: OnSetInert( linkInertia * inert)
{
    CRobotCFG * t = GetCFGIns();
    return t->InSetInert( inert);
}
bool CRobotCFG:: OnSetMaxKineFeatures( Vect6 mv,Vect6 ma,Vect6 mj,Vect6 mt)
{
    CRobotCFG * t = GetCFGIns();
    return t->InSetMaxKineFeatures( mv,ma,mj,mt);
}
bool CRobotCFG:: OnSetLinkSetting( Matrix6 ll,Vect6 lb)
{
    CRobotCFG * t = GetCFGIns();
    return t->InSetLinkSetting( ll,lb);
}
bool CRobotCFG:: OnSetTCF(Vect6 tc)
{
    CRobotCFG * t = GetCFGIns();
    return t->InSetTCF(tc);
}
bool CRobotCFG:: OnSetOFilter(Vect4 of)
{
    CRobotCFG * t = GetCFGIns();
    return t->InSetOFilter(of);
}
bool CRobotCFG:: OnSetPGPara(Vect3 pg)
{
    CRobotCFG * t = GetCFGIns();
    return t->InSetPGPara(pg);
}
bool CRobotCFG:: OnLoad(char * path)
{
    CRobotCFG * t = GetCFGIns();
    return t->InLoad2(path);
}

bool CRobotCFG:: OnLoad2(char * path)
{
    CRobotCFG * t = GetCFGIns();
    return t->InLoad2(path);
}
bool CRobotCFG:: OnSave(char * path)
{
    CRobotCFG * t = GetCFGIns();
    return t->InSave2(path);
}

bool CRobotCFG:: OnSave2(char * path)
{
    CRobotCFG * t = GetCFGIns();
    return t->InSave2(path);
}


///////////////////////
bool CRobotCFG:: InLoad2(char * path)
{
    long i;

    if(path == NULL)
    {
        return false;
    }
    FILE * fp = fopen(path,"r");
    if(fp == NULL)
    {
        return false;
    }

    long j;
    long v1,v2;


    fscanf(fp,"%d\n",&v1);
    fscanf(fp,"%d\n",&v2);
    if(v1 != ROB_CFG_VERSION1 || v2 != ROB_CFG_VERSION2)
    {
        fclose(fp);
        return false;
    }
    for( i = 0; i < 6 ; i ++)
    {
        fscanf(fp,"%lf\n",&m_dh[i].alf_pre);
        fscanf(fp,"%lf\n",&m_dh[i].a_pre);
        fscanf(fp,"%lf\n",&m_dh[i].d);
        fscanf(fp,"%lf\n",&m_dh[i].theta);
        fscanf(fp,"%lf\n",&m_dh[i].range_h);
        fscanf(fp,"%lf\n",&m_dh[i].range_l);
    }

    for( i = 0; i < 6 ; i ++)
    {
        fscanf(fp,"%lf\n",&m_inert[i].mass);
        fscanf(fp,"%lf\n",&m_inert[i].mass_core_pos[0]);
        fscanf(fp,"%lf\n",&m_inert[i].mass_core_pos[1]);
        fscanf(fp,"%lf\n",&m_inert[i].mass_core_pos[2]);


        for(j = 0; j < 3 ;j ++)
        {
            fscanf(fp,"%lf\n",&m_inert[i].inertial_tensor[j][0]);
            fscanf(fp,"%lf\n",&m_inert[i].inertial_tensor[j][1]);
            fscanf(fp,"%lf\n",&m_inert[i].inertial_tensor[j][2]);
        }

    }

    for(i = 0; i < 6 ; i ++)
    {
        fscanf(fp,"%lf\n",&m_max_vel[i]);
    }

    for(i = 0; i < 6 ; i ++)
    {
        fscanf(fp,"%lf\n",&m_max_acc[i]);
    }


    for(i = 0; i < 6 ; i ++)
    {
        fscanf(fp,"%lf\n",&m_max_jerk[i]);
    }
    for(i = 0; i < 6 ; i ++)
    {
        fscanf(fp,"%lf\n",&m_max_torque[i]);
    }

    for(i = 0; i < 6 ; i ++)
    {
        for(j = 0; j < 6 ; j ++)
        {
            fscanf(fp,"%lf\n",&m_joint_link[i*6 + j]);
        }
    }

    for(i = 0; i < 6 ; i ++)
    {
        fscanf(fp,"%lf\n",&m_joint_base[i]);
    }

    for(i = 0; i < 6 ; i ++)
    {
        fscanf(fp,"%lf\n",&m_torque_current_foctor[i]);
    }

    for(i = 0; i < 4 ; i ++)
    {
        fscanf(fp,"%lf\n",&m_output_filter_para[i]);
    }
    for(i = 0; i < 3 ; i ++)
    {
        fscanf(fp,"%lf\n",&m_pos_ges_para[i]);
    }


    fclose(fp);

    for(i = 0; i < MAX_CFG_ITEM_SIZE ; i ++)
    {
        m_value_valid_tags[i] = true;
    }
    return true;
}

bool CRobotCFG:: InSave2(char * path)
{
    long i,j;
    for(i = 0; i < MAX_CFG_ITEM_SIZE ; i ++)
    {
        if(m_value_valid_tags[i] == false)
        {
            return false;
        }
    }
    if(path == NULL)
    {
        return false;
    }
    FILE * fp = fopen(path,"w");
    if(fp == NULL)
    {
        return false;
    }

    fprintf(fp,"%d\n",ROB_CFG_VERSION1);
    fprintf(fp,"%d\n",ROB_CFG_VERSION2);
    for( i = 0; i < 6 ; i ++)
    {
        fprintf(fp,"%lf\n",m_dh[i].alf_pre);
        fprintf(fp,"%lf\n",m_dh[i].a_pre);
        fprintf(fp,"%lf\n",m_dh[i].d);
        fprintf(fp,"%lf\n",m_dh[i].theta);
        fprintf(fp,"%lf\n",m_dh[i].range_h);
        fprintf(fp,"%lf\n",m_dh[i].range_l);
    }

    for( i = 0; i < 6 ; i ++)
    {
        fprintf(fp,"%lf\n",m_inert[i].mass);
        fprintf(fp,"%lf\n",m_inert[i].mass_core_pos[0]);
        fprintf(fp,"%lf\n",m_inert[i].mass_core_pos[1]);
        fprintf(fp,"%lf\n",m_inert[i].mass_core_pos[2]);


        for(j = 0; j < 3 ;j ++)
        {
            fprintf(fp,"%lf\n",m_inert[i].inertial_tensor[j][0]);
            fprintf(fp,"%lf\n",m_inert[i].inertial_tensor[j][1]);
            fprintf(fp,"%lf\n",m_inert[i].inertial_tensor[j][2]);
        }

    }

    for(i = 0; i < 6 ; i ++)
    {
        fprintf(fp,"%lf\n",m_max_vel[i]);
    }

    for(i = 0; i < 6 ; i ++)
    {
        fprintf(fp,"%lf\n",m_max_acc[i]);
    }


    for(i = 0; i < 6 ; i ++)
    {
        fprintf(fp,"%lf\n",m_max_jerk[i]);
    }
    for(i = 0; i < 6 ; i ++)
    {
        fprintf(fp,"%lf\n",m_max_torque[i]);
    }

    for(i = 0; i < 6 ; i ++)
    {
        for(j = 0; j < 6 ; j ++)
        {
            fprintf(fp,"%lf\n",m_joint_link[i*6 + j]);
        }
    }

    for(i = 0; i < 6 ; i ++)
    {
        fprintf(fp,"%lf\n",m_joint_base[i]);
    }

    for(i = 0; i < 6 ; i ++)
    {
        fprintf(fp,"%lf\n",m_torque_current_foctor[i]);
    }

    for(i = 0; i < 4 ; i ++)
    {
        fprintf(fp,"%lf\n",m_output_filter_para[i]);
    }
    for(i = 0; i < 3 ; i ++)
    {
        fprintf(fp,"%lf\n",m_pos_ges_para[i]);
    }

    fclose(fp);
    return true;
}


