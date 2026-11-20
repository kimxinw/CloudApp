#include "delectseam.h"
#include "ui_delectseam.h"

#include <QListWidgetItem>

DelectSeam::DelectSeam(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::DelectSeam)
{
    ui->setupUi(this);
    setSeamSelectionVisible(false);

    connect(ui->delect_arc, &QPushButton::clicked, this, &DelectSeam::delectArc);
    connect(ui->select_all, &QPushButton::clicked, this, [this]() {
        setAllSeamsChecked(true);
    });
    connect(ui->clear_all, &QPushButton::clicked, this, [this]() {
        setAllSeamsChecked(false);
    });
    connect(ui->confirm_button, &QPushButton::clicked, this, &QDialog::accept);
    connect(ui->cancel_button, &QPushButton::clicked, this, &QDialog::reject);
}

DelectSeam::~DelectSeam()
{
    delete ui;
}

void DelectSeam::setSeamItems(const QStringList &items)
{
    ui->seam_list->clear();
    setSeamSelectionVisible(!items.isEmpty());

    for (int index = 0; index < items.size(); ++index) {
        auto *item = new QListWidgetItem(items[index], ui->seam_list);
        item->setData(Qt::UserRole, index);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Checked);
    }
}

QVector<int> DelectSeam::selectedSeamIndices() const
{
    QVector<int> indices;
    indices.reserve(ui->seam_list->count());

    for (int row = 0; row < ui->seam_list->count(); ++row) {
        const QListWidgetItem *item = ui->seam_list->item(row);
        if (item->checkState() == Qt::Checked) {
            indices.push_back(item->data(Qt::UserRole).toInt());
        }
    }

    return indices;
}

void DelectSeam::setArcDeletionVisible(bool visible)
{
    ui->arc_controls->setVisible(visible);
}

void DelectSeam::setSeamSelectionVisible(bool visible)
{
    ui->selection_title->setVisible(visible);
    ui->selection_hint->setVisible(visible);
    ui->seam_list->setVisible(visible);
    ui->select_all->setVisible(visible);
    ui->clear_all->setVisible(visible);
}

void DelectSeam::setAllSeamsChecked(bool checked)
{
    const Qt::CheckState state = checked ? Qt::Checked : Qt::Unchecked;
    for (int row = 0; row < ui->seam_list->count(); ++row) {
        ui->seam_list->item(row)->setCheckState(state);
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
