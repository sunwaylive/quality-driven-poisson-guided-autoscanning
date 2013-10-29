/********************************************************************************
** Form generated from reading UI file 'camera_para.ui'
**
** Created: Mon Oct 28 22:33:12 2013
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CAMERA_PARA_H
#define UI_CAMERA_PARA_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFrame>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QTableView>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_camera_paras
{
public:
    QPushButton *pushButton_scan;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QDoubleSpinBox *horizon_dist;
    QDoubleSpinBox *vertical_dist;
    QDoubleSpinBox *max_dist;
    QGroupBox *groupBox;
    QLabel *label_6;
    QDoubleSpinBox *dist_to_model;
    QPushButton *pushButton_initial_scan;
    QLabel *label_4;
    QLabel *label_5;
    QPushButton *pushButton_merge;
    QFrame *line;
    QFrame *line_2;
    QTableView *tableView_scan_candidates;
    QTableView *tableView_scan_results;
    QPushButton *pushButton_reset;
    QPushButton *pushButton_compute_nbv;
    QCheckBox *checkBox_show_init_cameras;

    void setupUi(QWidget *camera_paras)
    {
        if (camera_paras->objectName().isEmpty())
            camera_paras->setObjectName(QString::fromUtf8("camera_paras"));
        camera_paras->resize(265, 704);
        pushButton_scan = new QPushButton(camera_paras);
        pushButton_scan->setObjectName(QString::fromUtf8("pushButton_scan"));
        pushButton_scan->setGeometry(QRect(30, 431, 201, 31));
        label = new QLabel(camera_paras);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(31, 25, 111, 16));
        label_2 = new QLabel(camera_paras);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(31, 51, 111, 16));
        label_3 = new QLabel(camera_paras);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(31, 78, 111, 16));
        horizon_dist = new QDoubleSpinBox(camera_paras);
        horizon_dist->setObjectName(QString::fromUtf8("horizon_dist"));
        horizon_dist->setGeometry(QRect(160, 25, 62, 22));
        horizon_dist->setDecimals(10);
        horizon_dist->setMaximum(5);
        horizon_dist->setSingleStep(0.1);
        horizon_dist->setValue(1);
        vertical_dist = new QDoubleSpinBox(camera_paras);
        vertical_dist->setObjectName(QString::fromUtf8("vertical_dist"));
        vertical_dist->setGeometry(QRect(160, 51, 62, 22));
        vertical_dist->setDecimals(10);
        vertical_dist->setMaximum(5);
        vertical_dist->setSingleStep(0.1);
        vertical_dist->setValue(0.6);
        max_dist = new QDoubleSpinBox(camera_paras);
        max_dist->setObjectName(QString::fromUtf8("max_dist"));
        max_dist->setGeometry(QRect(160, 78, 62, 22));
        max_dist->setDecimals(10);
        max_dist->setMaximum(5);
        max_dist->setSingleStep(0.1);
        max_dist->setValue(1.5);
        groupBox = new QGroupBox(camera_paras);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(10, 10, 241, 141));
        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(21, 95, 111, 16));
        dist_to_model = new QDoubleSpinBox(groupBox);
        dist_to_model->setObjectName(QString::fromUtf8("dist_to_model"));
        dist_to_model->setGeometry(QRect(150, 95, 62, 22));
        dist_to_model->setDecimals(10);
        dist_to_model->setMaximum(5);
        dist_to_model->setSingleStep(0.1);
        dist_to_model->setValue(1);
        pushButton_initial_scan = new QPushButton(camera_paras);
        pushButton_initial_scan->setObjectName(QString::fromUtf8("pushButton_initial_scan"));
        pushButton_initial_scan->setGeometry(QRect(30, 161, 101, 31));
        label_4 = new QLabel(camera_paras);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(20, 241, 211, 21));
        label_5 = new QLabel(camera_paras);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(20, 471, 211, 21));
        pushButton_merge = new QPushButton(camera_paras);
        pushButton_merge->setObjectName(QString::fromUtf8("pushButton_merge"));
        pushButton_merge->setGeometry(QRect(40, 671, 71, 31));
        line = new QFrame(camera_paras);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(-10, 461, 311, 20));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        line_2 = new QFrame(camera_paras);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(-20, 191, 311, 20));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        tableView_scan_candidates = new QTableView(camera_paras);
        tableView_scan_candidates->setObjectName(QString::fromUtf8("tableView_scan_candidates"));
        tableView_scan_candidates->setGeometry(QRect(10, 266, 251, 161));
        tableView_scan_results = new QTableView(camera_paras);
        tableView_scan_results->setObjectName(QString::fromUtf8("tableView_scan_results"));
        tableView_scan_results->setGeometry(QRect(10, 491, 251, 171));
        pushButton_reset = new QPushButton(camera_paras);
        pushButton_reset->setObjectName(QString::fromUtf8("pushButton_reset"));
        pushButton_reset->setGeometry(QRect(160, 671, 71, 31));
        pushButton_compute_nbv = new QPushButton(camera_paras);
        pushButton_compute_nbv->setObjectName(QString::fromUtf8("pushButton_compute_nbv"));
        pushButton_compute_nbv->setGeometry(QRect(30, 211, 201, 31));
        checkBox_show_init_cameras = new QCheckBox(camera_paras);
        checkBox_show_init_cameras->setObjectName(QString::fromUtf8("checkBox_show_init_cameras"));
        checkBox_show_init_cameras->setGeometry(QRect(133, 166, 91, 21));
        groupBox->raise();
        pushButton_scan->raise();
        label->raise();
        label_2->raise();
        label_3->raise();
        horizon_dist->raise();
        vertical_dist->raise();
        max_dist->raise();
        pushButton_initial_scan->raise();
        label_4->raise();
        label_5->raise();
        pushButton_merge->raise();
        line->raise();
        line_2->raise();
        tableView_scan_candidates->raise();
        tableView_scan_results->raise();
        pushButton_reset->raise();
        pushButton_compute_nbv->raise();
        checkBox_show_init_cameras->raise();

        retranslateUi(camera_paras);

        QMetaObject::connectSlotsByName(camera_paras);
    } // setupUi

    void retranslateUi(QWidget *camera_paras)
    {
        camera_paras->setWindowTitle(QApplication::translate("camera_paras", "camera paras", 0, QApplication::UnicodeUTF8));
        pushButton_scan->setText(QApplication::translate("camera_paras", "Scan", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("camera_paras", "Horizontal Range :", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("camera_paras", "Vertical Range :", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("camera_paras", "Max Distance :", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("camera_paras", "Camera Parameters", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("camera_paras", "Distance To model:", 0, QApplication::UnicodeUTF8));
        pushButton_initial_scan->setText(QApplication::translate("camera_paras", "Initial Scan", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("camera_paras", "Choose scan candidates:", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("camera_paras", "Merge scan results with Init-scan:", 0, QApplication::UnicodeUTF8));
        pushButton_merge->setText(QApplication::translate("camera_paras", "Merge", 0, QApplication::UnicodeUTF8));
        pushButton_reset->setText(QApplication::translate("camera_paras", "Reset", 0, QApplication::UnicodeUTF8));
        pushButton_compute_nbv->setText(QApplication::translate("camera_paras", "Compute Next Best View ", 0, QApplication::UnicodeUTF8));
        checkBox_show_init_cameras->setText(QApplication::translate("camera_paras", "show cameras", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class camera_paras: public Ui_camera_paras {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CAMERA_PARA_H
