#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include "drawline.h"
#include <QList>
#include "lowpassfilter2p.h"
#include <QList>
#include "kine.h"

typedef struct robootJog
{
    double jog[6];
} robootJog;

#define  CI_ADDR_JP0			30128
#define  CI_ADDR_JP1			30178
#define  CI_ADDR_JP2			30228
#define  CI_ADDR_JP3			30278
#define  CI_ADDR_JP4			30328
#define  CI_ADDR_JP5			30378



#define  CI_ADDR_JP0_CMP_REAL	78676
#define  CI_ADDR_JP1_CMP_REAL	78686
#define  CI_ADDR_JP2_CMP_REAL	78696
#define  CI_ADDR_JP3_CMP_REAL	78706
#define  CI_ADDR_JP4_CMP_REAL	78716
#define  CI_ADDR_JP5_CMP_REAL	78726

#define  CI_ADDR_SYS_STATE		50292
#define  CI_ADDR_SYS_ERR		2041


/*串口采集程序*/
namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();

private:
    Ui::Widget *ui;

    QSerialPort *serial;

    QString strTemporary;

    DrawLine *mdrawLine;

    /*字符串拆分*/
    QList<double> StrSplit(QString str);

    /*滤波器钥匙*/
    int m_nfilter;

    /*巴特沃斯低通滤波器*/
    math::LowPassFilter2p *lowFilter;


    /*time key*/
    int tmieID;

    /*储存关节容器*/
    QList<robootJog> jogList;
    /*储存百分表*/
    QList<double> dialgageList;

    CKine  m_kine;
private slots:
    void slotSerialMessgae();
    void slotopenSerial();
    void slotreadSerial();
    void slotstopSerial();

    void slotconnect();
    void slotStartCollection();
    void slotstopCollection();

    void slotCalculate();

protected:
    void timerEvent(QTimerEvent *);



};

#endif // WIDGET_H
