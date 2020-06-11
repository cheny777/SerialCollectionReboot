#include "widget.h"
#include "ui_widget.h"
#include <QDebug>
#include <QHBoxLayout>
#include <QString>
#include "ciprotal.h"
#include "smoothing.h"
#include "robotcfg.h"
#include "fileportal.h"
Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);

    this->resize(1200,500);
    //查找可用的串口
    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        QSerialPort serial;
        serial.setPort(info);
        if(serial.open(QIODevice::ReadWrite))
        {
            ui->comboBox->addItem(serial.portName());
            serial.close();
        }
    }
    /*设置波特率*/
    QStringList strbaud;
    strbaud<<"9600"<<"19200"<<"38400"<<"57600"<<"115200";
    for(int i =0;i<strbaud.size();i++)
    {
        ui->comboBox_2->addItem(strbaud[i]);
    }
    ui->comboBox_2->setCurrentIndex(4);

    ui->comboBox_3->addItem(QString::number(8));

    ui->comboBox_4->addItem(QString::number(0));

    ui->comboBox_5->addItem(QString::number(0));
    ui->comboBox_5->addItem(QString::number(1));
    ui->comboBox_5->addItem(QString::number(2));

    for(int i =0 ;i<6;i++)
    {
        ui->comboBox_6->addItem(QString(65+i));
    }
    /*QSerialPortInfo：提供系统中存在的串口的信息
    接下来需要创建一个QSerialPort的对象，对串口的名称、
    波特率、数据位、校验位、停止位等参数进行设置，然后才进行串口读写操作。
    */
    connect(ui->pushButton_2,SIGNAL(clicked(bool)),this,SLOT(slotstopSerial()));
    connect(ui->pushButton,SIGNAL(clicked(bool)),this,SLOT(slotopenSerial()));
    connect(ui->pushButton_4,SIGNAL(clicked(bool)),this,SLOT(slotCalculate()));



    ui->textEdit->setReadOnly(true);

    mdrawLine = new DrawLine(ui->widget);
    mdrawLine->setMinimumHeight(300);
    QHBoxLayout *hoxlayout = new QHBoxLayout(ui->widget);
    hoxlayout->addWidget(mdrawLine);
    hoxlayout->setSpacing(0);
    hoxlayout->setMargin(0);

    QStringList FilterList;
    FilterList<<"低通"<<"均值"<<"低通加均值"<<"不滤波";
    for(int i = 0 ;i<FilterList.size();i++)
    {
        ui->comboBox_7->addItem(FilterList[i]);
    }

    /*设置滤波参数*/
    int FilterPoints = 3;
    int FilterRange = 10;
    m_nfilter = pulse_filter_create(FilterPoints, FilterRange, 1);


    /*巴特沃斯滤波*/
    lowFilter = new math::LowPassFilter2p(0.01,1);
}

Widget::~Widget()
{
    delete ui;
}

QList<double> Widget::StrSplit(QString str)
{
    QString nums;
    QList<double> doublelist;
    for(int i =0;i<str.size();i++)
    {

        if(str[i] == 'A'||str[i] == 'B'||str[i] == 'C'||str[i] == 'D'||
                str[i] == 'E'||str[i] == 'F'||str[i] == 'G'||
                str[i] == 'H'||str[i] == 'I')
        {
            bool bbl;
            double nn = nums.toDouble(&bbl);
            if(bbl)
            {
                doublelist.push_back(nn);
            }
            nums.clear();
        }
        else
        {
            nums+=str[i];
        }
    }
    return doublelist;
}

void Widget::slotSerialMessgae()
{

}

void Widget::slotopenSerial()
{
    ui->textEdit->clear();

    serial = new QSerialPort();

    //设置串口名
    serial->setPortName(ui->comboBox->currentText());
    //打开串口
    serial->open(QIODevice::ReadWrite);
    //设置波特率
    serial->setBaudRate(ui->comboBox_2->currentText().toInt());
    //设置数据位数
    switch(ui->comboBox_3->currentText().toInt())
    {
    case 8: serial->setDataBits(QSerialPort::Data8); break;
    default: break;
    }
    //设置奇偶校验
    switch(ui->comboBox_4->currentText().toInt())
    {
    case 0: serial->setParity(QSerialPort::NoParity); break;
    default: break;
    }
    //设置停止位
    switch(ui->comboBox_5->currentIndex())
    {
    case 1: serial->setStopBits(QSerialPort::OneStop); break;
    case 2: serial->setStopBits(QSerialPort::TwoStop); break;
    default: break;
    }
    //设置流控制
    serial->setFlowControl(QSerialPort::NoFlowControl);
    //disconnect(serial,SIGNAL(readyRead()),SLOT(slotreadSerial()));
    connect(serial,SIGNAL(readyRead()),SLOT(slotreadSerial()));

}

void Widget::slotreadSerial()
{
    QByteArray buf;
    buf = serial->readAll();
    if(!buf.isEmpty())
    {
        double numdl = QString(buf).toDouble();
        ui->textEdit->append(QString(buf));
        switch (ui->comboBox_7->currentIndex()) {
        case 0:
            mdrawLine->setData(FilterDitong2(numdl));
            break;
        case 1:
            mdrawLine->setData(LowPassFilter_Average(numdl));
            break;
        case 2:
            mdrawLine->setData(LowPassFilter_Average(FilterDitong2(numdl)));
            break;
        case 3:
            mdrawLine->setData(numdl);
            break;
        default:
            break;
        }
        dialgageList.push_back(numdl);

        /*读取机器人数据*/
        double v[30];
        unsigned long c;
        long cnt = OnGetCiValue(0,v,&c,14);
        if(cnt == 14)
        {
            robootJog ro;
            for(int i =0;i<6;i++)
            {
                ro.jog[i] = v[i];
            }
            jogList.push_back(ro);
        }
    }
    buf.clear();
}

void Widget::slotstopSerial()
{
    //关闭串口
    serial->clear();
    serial->close();
    serial->deleteLater();

}

void Widget::slotconnect()
{
    char ipchar[4];
    ipchar[0] = 192;
    ipchar[1] = 168;
    ipchar[2] = 1;
    ipchar[3] = 158;
    /*连接控制器*/
    if(OnLinkMachine(0,ipchar[0],ipchar[1],ipchar[2],ipchar[3])== false)
    {
        qDebug()<<"失败";
    }
    else
    {
        qDebug()<<"成功";
        long   addr[14];

        long reg_cnt = 0;
        addr[reg_cnt+0] = CI_ADDR_JP0;
        addr[reg_cnt+1] = CI_ADDR_JP1;
        addr[reg_cnt+2] = CI_ADDR_JP2;
        addr[reg_cnt+3] = CI_ADDR_JP3;
        addr[reg_cnt+4] = CI_ADDR_JP4;
        addr[reg_cnt+5] = CI_ADDR_JP5;


        addr[reg_cnt+6] = CI_ADDR_JP0_CMP_REAL;
        addr[reg_cnt+7] = CI_ADDR_JP1_CMP_REAL;
        addr[reg_cnt+8] = CI_ADDR_JP2_CMP_REAL;
        addr[reg_cnt+9] = CI_ADDR_JP3_CMP_REAL;
        addr[reg_cnt+10] = CI_ADDR_JP4_CMP_REAL;
        addr[reg_cnt+11] = CI_ADDR_JP5_CMP_REAL;
        addr[reg_cnt+12] = CI_ADDR_SYS_STATE;
        addr[reg_cnt+13] = CI_ADDR_SYS_ERR;

        if( OnRegCiValue(0,addr,14,0.01,0.01) == false)
        {
            qDebug()<<"error";
            return;
        }

        char * path = "./Temp/RCFG.rcfg";
        bool ret = RecvFile(ipchar[0],ipchar[1],ipchar[2],ipchar[3],path,"/home/Lynuc/RI/RCFG.rcfg");
        if(ret == false)
        {
            qDebug()<<"同步失败";
            return ;
        }
        if(CRobotCFG::OnLoad2(path) == false)
        {
            qDebug()<<"同步失败";
            return ;
        }
        JointDsrpt m_dh_para[6];
        if( CRobotCFG::OnGetJD(m_dh_para) == false)
        {
            qDebug()<<"同步失败";
            return ;
        }
        long i;
        for(  i = 0; i < 6 ; i ++)
        {
            if(m_kine.SetLink(i, m_dh_para[i].alf_pre,
                              m_dh_para[i].a_pre,
                              m_dh_para[i].d,
                              m_dh_para[i].theta,
                              m_dh_para[i].range_l,
                              m_dh_para[i].range_h) == false )
            {
                qDebug()<<"同步失败";
                return ;
            }
        }
    }
}

void Widget::slotStartCollection()
{
    tmieID = startTimer(10);
}

void Widget::slotstopCollection()
{
    killTimer(tmieID);
}

void Widget::slotCalculate()
{
    /*计算数据*/





}

void Widget::timerEvent(QTimerEvent *)
{
    double v[30];
    unsigned long c;
    /* 读取机器人关节，坐标，工具 CI数据 */
    long cnt = OnGetCiValue(0,v,&c,14);

    //******显示机器人状态信息***********//
    if(cnt == 14)
    {
        robootJog ro;
        for(int i =0;i<6;i++)
        {
            ro.jog[i] = v[i];
        }
    }
}
