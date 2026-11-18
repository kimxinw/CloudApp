#ifndef DELECTSEAM_H
#define DELECTSEAM_H

#include <QDialog>

namespace Ui {
class DelectSeam;
}

class DelectSeam : public QDialog
{
    Q_OBJECT

public:
    explicit DelectSeam(QWidget *parent = nullptr);
    ~DelectSeam();
    void delectSeam();
    void delectArc();
    void addpic(const QString &picpath);

private:
    Ui::DelectSeam *ui;
};

#endif // DELECTSEAM_H
