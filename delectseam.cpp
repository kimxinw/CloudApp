#include "delectseam.h"
#include "ui_delectseam.h"

#include <QEvent>
#include <QListWidgetItem>
#include <QMouseEvent>
#include <QPainter>
#include <QSignalBlocker>

#include <algorithm>

namespace {
constexpr int TaskIndexRole = Qt::UserRole;
constexpr int TaskTypeRole = Qt::UserRole + 1;
constexpr double FlatSeamHitDistance = 12.0;
constexpr double VerticalSeamHitDistance = 14.0;
}

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
        item->setData(TaskIndexRole, index);
        item->setData(TaskTypeRole, static_cast<int>(TaskType::FlatSeam));
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Checked);
    }

    refreshPreview();
}

void DelectSeam::setSeamHitLines(const QVector<QLineF> &lines)
{
    seamHitLines_ = lines;
    ui->label_pic->setCursor(
        seamHitLines_.isEmpty() && verticalSeamHitPoints_.isEmpty()
            ? Qt::ArrowCursor
            : Qt::PointingHandCursor);
    refreshPreview();
}

void DelectSeam::setVerticalSeamItems(const QStringList &items)
{
    const QSignalBlocker blocker(ui->seam_list);

    for (int row = ui->seam_list->count() - 1; row >= 0; --row) {
        const QListWidgetItem *item = ui->seam_list->item(row);
        if (item->data(TaskTypeRole).toInt()
            == static_cast<int>(TaskType::VerticalSeam)) {
            delete ui->seam_list->takeItem(row);
        }
    }

    for (int index = 0; index < items.size(); ++index) {
        auto *item = new QListWidgetItem(items[index], ui->seam_list);
        item->setData(TaskIndexRole, index);
        item->setData(TaskTypeRole, static_cast<int>(TaskType::VerticalSeam));
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Checked);
    }

    setSeamSelectionVisible(ui->seam_list->count() > 0);
    refreshPreview();
}

void DelectSeam::setVerticalSeamHitPoints(const QVector<QPointF> &points)
{
    verticalSeamHitPoints_ = points;
    ui->label_pic->setCursor(
        seamHitLines_.isEmpty() && verticalSeamHitPoints_.isEmpty()
            ? Qt::ArrowCursor
            : Qt::PointingHandCursor);
    refreshPreview();
}

QVector<int> DelectSeam::selectedIndices(TaskType type) const
{
    QVector<int> indices;
    indices.reserve(ui->seam_list->count());

    for (int row = 0; row < ui->seam_list->count(); ++row) {
        const QListWidgetItem *item = ui->seam_list->item(row);
        if (item->checkState() == Qt::Checked
            && item->data(TaskTypeRole).toInt() == static_cast<int>(type)) {
            indices.push_back(item->data(TaskIndexRole).toInt());
        }
    }

    return indices;
}

QVector<int> DelectSeam::selectedSeamIndices() const
{
    return selectedIndices(TaskType::FlatSeam);
}

QVector<int> DelectSeam::selectedVerticalSeamIndices() const
{
    return selectedIndices(TaskType::VerticalSeam);
}

QListWidgetItem *DelectSeam::taskItem(TaskType type, int index) const
{
    for (int row = 0; row < ui->seam_list->count(); ++row) {
        QListWidgetItem *item = ui->seam_list->item(row);
        if (item->data(TaskTypeRole).toInt() == static_cast<int>(type)
            && item->data(TaskIndexRole).toInt() == index) {
            return item;
        }
    }

    return nullptr;
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
    if (watched == ui->label_pic
        && (!seamHitLines_.isEmpty() || !verticalSeamHitPoints_.isEmpty()) &&
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
    if (originalPixmap_.isNull()
        || (seamHitLines_.isEmpty() && verticalSeamHitPoints_.isEmpty())) {
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

    int nearestVerticalIndex = -1;
    double nearestVerticalDistance = VerticalSeamHitDistance;
    for (int index = 0; index < verticalSeamHitPoints_.size(); ++index) {
        const QPointF &imagePoint = verticalSeamHitPoints_[index];
        const QPointF labelPoint(contentRect.left() + imagePoint.x() * scaleX,
                                 contentRect.top() + imagePoint.y() * scaleY);
        const double distance = QLineF(labelPosition, labelPoint).length();
        if (distance < nearestVerticalDistance) {
            nearestVerticalDistance = distance;
            nearestVerticalIndex = index;
        }
    }

    if (nearestVerticalIndex >= 0) {
        if (QListWidgetItem *item = taskItem(TaskType::VerticalSeam,
                                             nearestVerticalIndex)) {
            item->setCheckState(
                item->checkState() == Qt::Checked ? Qt::Unchecked : Qt::Checked);
        }
        return;
    }

    int nearestIndex = -1;
    double nearestDistance = FlatSeamHitDistance;
    const int lineCount = static_cast<int>(seamHitLines_.size());

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

    if (QListWidgetItem *item = taskItem(TaskType::FlatSeam, nearestIndex)) {
        item->setCheckState(
            item->checkState() == Qt::Checked ? Qt::Unchecked : Qt::Checked);
    }
}

void DelectSeam::refreshPreview()
{
    if (originalPixmap_.isNull()) {
        return;
    }

    QPixmap preview = originalPixmap_;
    QPainter painter(&preview);
    painter.setRenderHint(QPainter::Antialiasing);

    const int lineCount = static_cast<int>(seamHitLines_.size());

    for (int index = 0; index < lineCount; ++index) {
        const QListWidgetItem *item = taskItem(TaskType::FlatSeam, index);
        if (!item) {
            continue;
        }
        const bool selected = item->checkState() == Qt::Checked;

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

    for (int index = 0; index < verticalSeamHitPoints_.size(); ++index) {
        const QListWidgetItem *item = taskItem(TaskType::VerticalSeam, index);
        if (!item) {
            continue;
        }

        const bool selected = item->checkState() == Qt::Checked;
        const QPointF center = verticalSeamHitPoints_[index];
        const QColor markerColor = selected ? QColor(180, 0, 210)
                                            : QColor(150, 150, 150);

        painter.setPen(QPen(Qt::white, 3));
        painter.setBrush(markerColor);
        painter.drawEllipse(center, 10.0, 10.0);
        painter.setPen(QPen(markerColor.darker(150), 2));
        painter.setBrush(Qt::NoBrush);
        painter.drawEllipse(center, 10.0, 10.0);

        const QRectF numberRect(center.x() + 9.0, center.y() - 20.0, 34.0, 20.0);
        painter.fillRect(numberRect, Qt::white);
        painter.setPen(markerColor.darker(150));
        painter.drawText(numberRect, Qt::AlignCenter,
                         QString("V%1").arg(index + 1));
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
