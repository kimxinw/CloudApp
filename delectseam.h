#ifndef DELECTSEAM_H
#define DELECTSEAM_H

#include <QDialog>
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
    QVector<int> selectedSeamIndices() const;
    void setArcDeletionVisible(bool visible);
    void delectArc();
    void addpic(const QString &picpath);

private:
    void setSeamSelectionVisible(bool visible);
    void setAllSeamsChecked(bool checked);

    Ui::DelectSeam *ui;
};

#endif // DELECTSEAM_H
