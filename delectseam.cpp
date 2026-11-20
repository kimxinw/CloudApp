#include "delectseam.h"
#include "ui_delectseam.h"

#include <QEvent>
#include <QListWidgetItem>
#include <QMouseEvent>
#include <QPainter>
#include <QSignalBlocker>

#include <algorithm>

DelectSeam::DelectSeam(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::DelectSeam)
{
    ui->setupUi(this);
    setSeamSelectionVisible(false);
    ui->label_pic->installEventFilter(this);

    connect(ui->delect_arc, &QPushButton::clicked, this, &DelectSeam::delectArc);
    connect(ui->select_all, &QPushButton::clicked, this, [this]() {
        setAllSeamsChecked(true);
    });
    connect(ui->clear_all, &QPushButton::clicked, this, [this]() {
        setAllSeamsChecked(false);
    });
    connect(ui->confirm_button, &QPushButton::clicked, this, &QDialog::accept);
    connect(ui->cancel_button, &QPushButton::clicked, this, &QDialog::reject);
    connect(ui->seam_list, &QListWidget::itemChanged, this, [this](QListWidgetItem *) {
        refreshPreview();
    });
}

DelectSeam::~DelectSeam()
{
    delete ui;
}

void DelectSeam::setSeamItems(const QStringList &items)
{
    const QSignalBlocker blocker(ui->seam_list);
    ui->seam_list->clear();
    setSeamSelectionVisible(!items.isEmpty());

    for (int index = 0; index < items.size(); ++index) {
        auto *item = new QListWidgetItem(items[index], ui->seam_list);
        item->setData(Qt::UserRole, index);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Checked);
    }

    refreshPreview();
}

void DelectSeam::setSeamHitLines(const QVector<QLineF> &lines)
{
    seamHitLines_ = lines;
    ui->label_pic->setCursor(
        seamHitLines_.isEmpty() ? Qt::ArrowCursor : Qt::PointingHandCursor);
    refreshPreview();
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
    const QSignalBlocker blocker(ui->seam_list);
    const Qt::CheckState state = checked ? Qt::Checked : Qt::Unchecked;
    for (int row = 0; row < ui->seam_list->count(); ++row) {
        ui->seam_list->item(row)->setCheckState(state);
    }
    refreshPreview();
}

bool DelectSeam::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == ui->label_pic && !seamHitLines_.isEmpty() &&
        event->type() == QEvent::MouseButtonRelease) {
        auto *mouseEvent = static_cast<QMouseEvent *>(event);
        if (mouseEvent->button() == Qt::LeftButton) {
            toggleSeamAt(mouseEvent->position());
            return true;
        }
    }

    return QDialog::eventFilter(watched, event);
}

double DelectSeam::pointToSegmentDistance(const QPointF &point, const QLineF &line)
{
    const double dx = line.dx();
    const double dy = line.dy();
    const double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared < 1e-9) {
        return QLineF(point, line.p1()).length();
    }

    double t = ((point.x() - line.x1()) * dx +
                (point.y() - line.y1()) * dy) /
               lengthSquared;
    t = std::clamp(t, 0.0, 1.0);

    const QPointF nearestPoint(line.x1() + t * dx, line.y1() + t * dy);
    return QLineF(point, nearestPoint).length();
}

void DelectSeam::toggleSeamAt(const QPointF &labelPosition)
{
    if (originalPixmap_.isNull() || seamHitLines_.isEmpty()) {
        return;
    }

    const QRect contentRect = ui->label_pic->contentsRect();
    if (!contentRect.contains(labelPosition.toPoint()) ||
        contentRect.width() <= 0 || contentRect.height() <= 0) {
        return;
    }

    const double scaleX = static_cast<double>(contentRect.width()) /
                          originalPixmap_.width();
    const double scaleY = static_cast<double>(contentRect.height()) /
                          originalPixmap_.height();

    int nearestIndex = -1;
    double nearestDistance = 12.0;
    const int lineCount = std::min(
        static_cast<int>(seamHitLines_.size()),
        ui->seam_list->count());

    for (int index = 0; index < lineCount; ++index) {
        const QLineF &imageLine = seamHitLines_[index];
        const QLineF labelLine(
            QPointF(contentRect.left() + imageLine.x1() * scaleX,
                    contentRect.top() + imageLine.y1() * scaleY),
            QPointF(contentRect.left() + imageLine.x2() * scaleX,
                    contentRect.top() + imageLine.y2() * scaleY));

        const double distance = pointToSegmentDistance(labelPosition, labelLine);
        if (distance < nearestDistance) {
            nearestDistance = distance;
            nearestIndex = index;
        }
    }

    if (nearestIndex < 0) {
        return;
    }

    QListWidgetItem *item = ui->seam_list->item(nearestIndex);
    item->setCheckState(
        item->checkState() == Qt::Checked ? Qt::Unchecked : Qt::Checked);
}

void DelectSeam::refreshPreview()
{
    if (originalPixmap_.isNull()) {
        return;
    }

    QPixmap preview = originalPixmap_;
    QPainter painter(&preview);
    painter.setRenderHint(QPainter::Antialiasing);

    const int lineCount = std::min(
        static_cast<int>(seamHitLines_.size()),
        ui->seam_list->count());

    for (int index = 0; index < lineCount; ++index) {
        const bool selected =
            ui->seam_list->item(index)->checkState() == Qt::Checked;

        QPen linePen(selected ? QColor(0, 190, 0) : QColor(150, 150, 150));
        linePen.setWidth(7);
        linePen.setCapStyle(Qt::RoundCap);
        painter.setPen(linePen);
        painter.drawLine(seamHitLines_[index]);

        const QPointF center = seamHitLines_[index].center();
        const QRectF numberRect(center.x() - 11.0, center.y() - 11.0, 22.0, 22.0);
        painter.fillRect(numberRect, Qt::white);
        painter.setPen(selected ? QColor(0, 120, 0) : QColor(90, 90, 90));
        painter.drawText(numberRect, Qt::AlignCenter, QString::number(index + 1));
    }

    painter.end();
    ui->label_pic->setPixmap(preview);
    ui->label_pic->update();
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
    originalPixmap_ = QPixmap(picpath);
    ui->label_pic->clear();
    ui->label_pic->setScaledContents(true);
    refreshPreview();
}
