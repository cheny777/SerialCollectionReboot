#ifndef ROBOTCFG_H
#define ROBOTCFG_H


#include "RDef.h"

#define  MAX_CFG_ITEM_SIZE 11
#define  ROB_CFG_VERSION1  6
#define  ROB_CFG_VERSION2  1


class CRobotCFG
{
public:
    static bool OnGetJD( JointDsrpt * jd);
    static bool OnGetInert( linkInertia * inert);
    static bool OnGetMaxKineFeatures( Vect6 mv,Vect6 ma,Vect6 mj,Vect6 mt);
    static bool OnGetLinkSetting( Matrix6 ll,Vect6 lb);
    static bool OnGetTCF(Vect6 tc);
    static bool OnGetOFilter(Vect4 of);
    static bool OnGetPGPara(Vect3 pg);

    static bool OnSetJD( JointDsrpt * jd);
    static bool OnSetInert( linkInertia * inert);
    static bool OnSetMaxKineFeatures( Vect6 mv,Vect6 ma,Vect6 mj,Vect6 mt);
    static bool OnSetLinkSetting( Matrix6 ll,Vect6 lb);
    static bool OnSetTCF(Vect6 tc);
    static bool OnSetOFilter(Vect4 of);
    static bool OnSetPGPara(Vect3 pg);

    static bool OnLoad(char * path);
    static bool OnLoad2(char * path);
    static bool OnSave(char * path);
    static bool OnSave2(char * path);

protected:
    CRobotCFG();
    virtual ~CRobotCFG();

    static CRobotCFG * GetCFGIns();

protected:
    JointDsrpt  * m_dh;
    linkInertia * m_inert;
    double      * m_max_vel;
    double      * m_max_acc;
    double      * m_max_jerk;
    double      * m_max_torque;
    double      * m_joint_link;
    double      * m_joint_base;
    double      * m_torque_current_foctor;
    double      * m_output_filter_para;
    double      * m_pos_ges_para;
    unsigned char * crc_1;
    unsigned char * crc_2;

    bool        m_value_valid_tags[MAX_CFG_ITEM_SIZE];
    long crc_end_pos;

    unsigned char * value_base;

protected:

    bool InGetJD( JointDsrpt * jd);
    bool InGetInert( linkInertia * inert);
    bool InGetMaxKineFeatures( Vect6 mv,Vect6 ma,Vect6 mj,Vect6 mt);
    bool InGetLinkSetting( Matrix6 ll,Vect6 lb);
    bool InGetTCF(Vect6 tc);
    bool InGetOFilter(Vect4 of);
    bool InGetPGPara(Vect3 pg);
    bool InSetJD( JointDsrpt * jd);
    bool InSetInert( linkInertia * inert);
    bool InSetMaxKineFeatures( Vect6 mv,Vect6 ma,Vect6 mj,Vect6 mt);
    bool InSetLinkSetting( Matrix6 ll,Vect6 lb);
    bool InSetTCF(Vect6 tc);
    bool InSetOFilter(Vect4 of);
    bool InSetPGPara(Vect3 pg);
    bool InLoad(char * path);
    bool InSave(char * path);
    void dmemcpy(double * t,double * s,long cnt);
    bool InLoad2(char * path);
    bool InSave2(char * path);

};

#endif // ROBOTCFG_H
