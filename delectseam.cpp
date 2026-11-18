#include "delectseam.h"
#include "ui_delectseam.h"

DelectSeam::DelectSeam(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::DelectSeam)
{
    ui->setupUi(this);

    connect(ui->delect_seam, &QPushButton::clicked, this, &DelectSeam::delectSeam);
    connect(ui->delect_arc, &QPushButton::clicked, this, &DelectSeam::delectArc);
}

DelectSeam::~DelectSeam()
{
    delete ui;
}

//焊缝删除
void DelectSeam::delectSeam()
{

    if (ui->seams->text().isEmpty()) {

        done(-1);
    } else {
        int number=10000+ui->seams->text().toInt()-1;
        done(number);
    }

}

//圆弧删除
void DelectSeam::delectArc()
{
    if (ui->arcs->text().isEmpty()) {

        done(-1);
    } else {
        int number=20000+ui->arcs->text().toInt()-1;
        done(number);
    }
}

//添加图片
void DelectSeam::addpic(const QString &picpath)
{
    QPixmap originalPixmap(picpath);
    ui->label_pic->clear();
    ui->label_pic->setPixmap(originalPixmap);
    ui->label_pic->setScaledContents(true);
    ui->label_pic->update();
}