#ifndef DELECTSEAM_H
#define DELECTSEAM_H

#include <QDialog>
#include <QLineF>
#include <QPixmap>
#include <QStringList>
#include <QVector>

class QListWidgetItem;

namespace Ui {
class DelectSeam;
}

class DelectSeam : public QDialog
{
    Q_OBJECT

public:
    explicit DelectSeam(QWidget *parent = nullptr);
    ~DelectSeam();

    void setSeamItems(const QStringList &items);
    void setSeamHitLines(const QVector<QLineF> &lines);
    void setVerticalSeamItems(const QStringList &items);
    void setVerticalSeamHitPoints(const QVector<QPointF> &points);
    QVector<int> selectedSeamIndices() const;
    QVector<int> selectedVerticalSeamIndices() const;
    void setArcDeletionVisible(bool visible);
    void delectArc();
    void addpic(const QString &picpath);

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;

private:
    enum class TaskType
    {
        FlatSeam,
        VerticalSeam
    };

    static double pointToSegmentDistance(const QPointF &point, const QLineF &line);
    QVector<int> selectedIndices(TaskType type) const;
    QListWidgetItem *taskItem(TaskType type, int index) const;
    void toggleSeamAt(const QPointF &labelPosition);
    void refreshPreview();
    void setSeamSelectionVisible(bool visible);
    void setAllSeamsChecked(bool checked);

    Ui::DelectSeam *ui;
    QVector<QLineF> seamHitLines_;
    QVector<QPointF> verticalSeamHitPoints_;
    QPixmap originalPixmap_;
};

#endif // DELECTSEAM_H
