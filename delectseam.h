#ifndef DELECTSEAM_H
#define DELECTSEAM_H

#include <QDialog>
#include <QLineF>
#include <QPixmap>
#include <QStringList>
#include <QVector>

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
    QVector<int> selectedSeamIndices() const;
    void setArcDeletionVisible(bool visible);
    void delectArc();
    void addpic(const QString &picpath);

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;

private:
    static double pointToSegmentDistance(const QPointF &point, const QLineF &line);
    void toggleSeamAt(const QPointF &labelPosition);
    void refreshPreview();
    void setSeamSelectionVisible(bool visible);
    void setAllSeamsChecked(bool checked);

    Ui::DelectSeam *ui;
    QVector<QLineF> seamHitLines_;
    QPixmap originalPixmap_;
};

#endif // DELECTSEAM_H
