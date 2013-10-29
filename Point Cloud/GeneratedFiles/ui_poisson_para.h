/********************************************************************************
** Form generated from reading UI file 'poisson_para.ui'
**
** Created: Mon Oct 28 22:33:12 2013
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_POISSON_PARA_H
#define UI_POISSON_PARA_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QTabWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_poisson_paras
{
public:
    QTabWidget *tabWidget;
    QWidget *tab;
    QCheckBox *checkBox_show_slices_Z;
    QDoubleSpinBox *poisson_depth;
    QCheckBox *checkBox_show_slices;
    QLabel *label;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_2;
    QPushButton *pushButton_label_iso_points;
    QPushButton *pushButton_label_smooth;
    QPushButton *pushButton_clear_label;
    QPushButton *pushButton_label_boundary_points;
    QPushButton *pushButton_compute_view_candidates;
    QPushButton *pushButton_view_candidates_clustering;
    QLabel *label_2;
    QDoubleSpinBox *radius;
    QCheckBox *checkBox_show_slices_Y;
    QCheckBox *checkBox_show_slices_X;
    QLabel *label_3;
    QPushButton *pushButton_slice;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_poisson_leafs;
    QPushButton *pushButton_poisson_nodes;
    QPushButton *pushButton_poisson_MC;
    QPushButton *pushButton_remove_non_iso_points;
    QDoubleSpinBox *poisson_ISO_interval;
    QCheckBox *checkBox_show_slices_transparent;
    QWidget *tab_2;
    QCheckBox *checkBox_use_confidence1;
    QCheckBox *checkBox_use_confidence2;
    QCheckBox *checkBox_use_confidence3;
    QCheckBox *checkBox_use_confidence4;
    QPushButton *pushButton_compute_confidence_original;
    QPushButton *pushButton_compute_confidence_samples;

    void setupUi(QWidget *poisson_paras)
    {
        if (poisson_paras->objectName().isEmpty())
            poisson_paras->setObjectName(QString::fromUtf8("poisson_paras"));
        poisson_paras->resize(248, 754);
        tabWidget = new QTabWidget(poisson_paras);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(0, 0, 241, 731));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        checkBox_show_slices_Z = new QCheckBox(tab);
        checkBox_show_slices_Z->setObjectName(QString::fromUtf8("checkBox_show_slices_Z"));
        checkBox_show_slices_Z->setGeometry(QRect(150, 570, 91, 16));
        poisson_depth = new QDoubleSpinBox(tab);
        poisson_depth->setObjectName(QString::fromUtf8("poisson_depth"));
        poisson_depth->setGeometry(QRect(120, 50, 101, 20));
        poisson_depth->setDecimals(5);
        poisson_depth->setMinimum(1e-05);
        poisson_depth->setMaximum(1000);
        poisson_depth->setSingleStep(0.01);
        poisson_depth->setValue(1);
        checkBox_show_slices = new QCheckBox(tab);
        checkBox_show_slices->setObjectName(QString::fromUtf8("checkBox_show_slices"));
        checkBox_show_slices->setGeometry(QRect(0, 540, 91, 16));
        label = new QLabel(tab);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(12, 17, 102, 26));
        groupBox_2 = new QGroupBox(tab);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        groupBox_2->setGeometry(QRect(-10, 280, 241, 221));
        verticalLayout_2 = new QVBoxLayout(groupBox_2);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        pushButton_label_iso_points = new QPushButton(groupBox_2);
        pushButton_label_iso_points->setObjectName(QString::fromUtf8("pushButton_label_iso_points"));

        verticalLayout_2->addWidget(pushButton_label_iso_points);

        pushButton_label_smooth = new QPushButton(groupBox_2);
        pushButton_label_smooth->setObjectName(QString::fromUtf8("pushButton_label_smooth"));

        verticalLayout_2->addWidget(pushButton_label_smooth);

        pushButton_clear_label = new QPushButton(groupBox_2);
        pushButton_clear_label->setObjectName(QString::fromUtf8("pushButton_clear_label"));

        verticalLayout_2->addWidget(pushButton_clear_label);

        pushButton_label_boundary_points = new QPushButton(groupBox_2);
        pushButton_label_boundary_points->setObjectName(QString::fromUtf8("pushButton_label_boundary_points"));

        verticalLayout_2->addWidget(pushButton_label_boundary_points);

        pushButton_compute_view_candidates = new QPushButton(groupBox_2);
        pushButton_compute_view_candidates->setObjectName(QString::fromUtf8("pushButton_compute_view_candidates"));

        verticalLayout_2->addWidget(pushButton_compute_view_candidates);

        pushButton_view_candidates_clustering = new QPushButton(groupBox_2);
        pushButton_view_candidates_clustering->setObjectName(QString::fromUtf8("pushButton_view_candidates_clustering"));

        verticalLayout_2->addWidget(pushButton_view_candidates_clustering);

        label_2 = new QLabel(tab);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(12, 47, 102, 26));
        radius = new QDoubleSpinBox(tab);
        radius->setObjectName(QString::fromUtf8("radius"));
        radius->setGeometry(QRect(120, 20, 101, 20));
        radius->setDecimals(5);
        radius->setMinimum(1e-05);
        radius->setMaximum(1000);
        radius->setSingleStep(0.01);
        radius->setValue(1);
        checkBox_show_slices_Y = new QCheckBox(tab);
        checkBox_show_slices_Y->setObjectName(QString::fromUtf8("checkBox_show_slices_Y"));
        checkBox_show_slices_Y->setGeometry(QRect(70, 570, 91, 16));
        checkBox_show_slices_X = new QCheckBox(tab);
        checkBox_show_slices_X->setObjectName(QString::fromUtf8("checkBox_show_slices_X"));
        checkBox_show_slices_X->setGeometry(QRect(0, 570, 91, 16));
        label_3 = new QLabel(tab);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(12, 77, 102, 26));
        pushButton_slice = new QPushButton(tab);
        pushButton_slice->setObjectName(QString::fromUtf8("pushButton_slice"));
        pushButton_slice->setGeometry(QRect(0, 510, 211, 21));
        layoutWidget = new QWidget(tab);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(0, 110, 231, 161));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        pushButton_poisson_leafs = new QPushButton(layoutWidget);
        pushButton_poisson_leafs->setObjectName(QString::fromUtf8("pushButton_poisson_leafs"));

        verticalLayout->addWidget(pushButton_poisson_leafs);

        pushButton_poisson_nodes = new QPushButton(layoutWidget);
        pushButton_poisson_nodes->setObjectName(QString::fromUtf8("pushButton_poisson_nodes"));

        verticalLayout->addWidget(pushButton_poisson_nodes);

        pushButton_poisson_MC = new QPushButton(layoutWidget);
        pushButton_poisson_MC->setObjectName(QString::fromUtf8("pushButton_poisson_MC"));

        verticalLayout->addWidget(pushButton_poisson_MC);

        pushButton_remove_non_iso_points = new QPushButton(layoutWidget);
        pushButton_remove_non_iso_points->setObjectName(QString::fromUtf8("pushButton_remove_non_iso_points"));

        verticalLayout->addWidget(pushButton_remove_non_iso_points);

        poisson_ISO_interval = new QDoubleSpinBox(tab);
        poisson_ISO_interval->setObjectName(QString::fromUtf8("poisson_ISO_interval"));
        poisson_ISO_interval->setGeometry(QRect(120, 80, 101, 20));
        poisson_ISO_interval->setDecimals(5);
        poisson_ISO_interval->setMinimum(1e-05);
        poisson_ISO_interval->setMaximum(1000);
        poisson_ISO_interval->setSingleStep(0.01);
        poisson_ISO_interval->setValue(1);
        checkBox_show_slices_transparent = new QCheckBox(tab);
        checkBox_show_slices_transparent->setObjectName(QString::fromUtf8("checkBox_show_slices_transparent"));
        checkBox_show_slices_transparent->setGeometry(QRect(100, 540, 131, 16));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        checkBox_use_confidence1 = new QCheckBox(tab_2);
        checkBox_use_confidence1->setObjectName(QString::fromUtf8("checkBox_use_confidence1"));
        checkBox_use_confidence1->setGeometry(QRect(10, 20, 201, 16));
        checkBox_use_confidence2 = new QCheckBox(tab_2);
        checkBox_use_confidence2->setObjectName(QString::fromUtf8("checkBox_use_confidence2"));
        checkBox_use_confidence2->setGeometry(QRect(10, 50, 201, 16));
        checkBox_use_confidence3 = new QCheckBox(tab_2);
        checkBox_use_confidence3->setObjectName(QString::fromUtf8("checkBox_use_confidence3"));
        checkBox_use_confidence3->setGeometry(QRect(10, 80, 231, 16));
        checkBox_use_confidence4 = new QCheckBox(tab_2);
        checkBox_use_confidence4->setObjectName(QString::fromUtf8("checkBox_use_confidence4"));
        checkBox_use_confidence4->setGeometry(QRect(10, 110, 231, 16));
        pushButton_compute_confidence_original = new QPushButton(tab_2);
        pushButton_compute_confidence_original->setObjectName(QString::fromUtf8("pushButton_compute_confidence_original"));
        pushButton_compute_confidence_original->setGeometry(QRect(10, 150, 201, 31));
        pushButton_compute_confidence_samples = new QPushButton(tab_2);
        pushButton_compute_confidence_samples->setObjectName(QString::fromUtf8("pushButton_compute_confidence_samples"));
        pushButton_compute_confidence_samples->setGeometry(QRect(10, 200, 201, 31));
        tabWidget->addTab(tab_2, QString());

        retranslateUi(poisson_paras);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(poisson_paras);
    } // setupUi

    void retranslateUi(QWidget *poisson_paras)
    {
        poisson_paras->setWindowTitle(QApplication::translate("poisson_paras", "Poisson paras", 0, QApplication::UnicodeUTF8));
        checkBox_show_slices_Z->setText(QApplication::translate("poisson_paras", "Show Z", 0, QApplication::UnicodeUTF8));
        checkBox_show_slices->setText(QApplication::translate("poisson_paras", "Show Slices", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("poisson_paras", "Grid Radius", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("poisson_paras", "GroupBox", 0, QApplication::UnicodeUTF8));
        pushButton_label_iso_points->setText(QApplication::translate("poisson_paras", "Label ISO points", 0, QApplication::UnicodeUTF8));
        pushButton_label_smooth->setText(QApplication::translate("poisson_paras", "Label Smooth", 0, QApplication::UnicodeUTF8));
        pushButton_clear_label->setText(QApplication::translate("poisson_paras", "Clear Label", 0, QApplication::UnicodeUTF8));
        pushButton_label_boundary_points->setText(QApplication::translate("poisson_paras", "Label Boundary Points", 0, QApplication::UnicodeUTF8));
        pushButton_compute_view_candidates->setText(QApplication::translate("poisson_paras", "Compute View Candidates", 0, QApplication::UnicodeUTF8));
        pushButton_view_candidates_clustering->setText(QApplication::translate("poisson_paras", "View Candidates Clustering", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("poisson_paras", "Octree Depth", 0, QApplication::UnicodeUTF8));
        checkBox_show_slices_Y->setText(QApplication::translate("poisson_paras", "Show Y", 0, QApplication::UnicodeUTF8));
        checkBox_show_slices_X->setText(QApplication::translate("poisson_paras", "Show X", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("poisson_paras", "ISO Interal", 0, QApplication::UnicodeUTF8));
        pushButton_slice->setText(QApplication::translate("poisson_paras", "Slice", 0, QApplication::UnicodeUTF8));
        pushButton_poisson_leafs->setText(QApplication::translate("poisson_paras", "Run Poisson on original", 0, QApplication::UnicodeUTF8));
        pushButton_poisson_nodes->setText(QApplication::translate("poisson_paras", "Run Poisson on samples", 0, QApplication::UnicodeUTF8));
        pushButton_poisson_MC->setText(QApplication::translate("poisson_paras", "Extract MC iso points on original", 0, QApplication::UnicodeUTF8));
        pushButton_remove_non_iso_points->setText(QApplication::translate("poisson_paras", "Extract MC iso points on sample", 0, QApplication::UnicodeUTF8));
        checkBox_show_slices_transparent->setText(QApplication::translate("poisson_paras", "Transparent Slices", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("poisson_paras", "Tab 1", 0, QApplication::UnicodeUTF8));
        checkBox_use_confidence1->setText(QApplication::translate("poisson_paras", "use confidence 1 (density)", 0, QApplication::UnicodeUTF8));
        checkBox_use_confidence2->setText(QApplication::translate("poisson_paras", "use confidence 2 (curvature)", 0, QApplication::UnicodeUTF8));
        checkBox_use_confidence3->setText(QApplication::translate("poisson_paras", "use confidence 3 (proj error)", 0, QApplication::UnicodeUTF8));
        checkBox_use_confidence4->setText(QApplication::translate("poisson_paras", "use confidence 4 (eigen)", 0, QApplication::UnicodeUTF8));
        pushButton_compute_confidence_original->setText(QApplication::translate("poisson_paras", "Compute Confidence on Original", 0, QApplication::UnicodeUTF8));
        pushButton_compute_confidence_samples->setText(QApplication::translate("poisson_paras", "Compute Confidence on Samples", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("poisson_paras", "Tab 2", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class poisson_paras: public Ui_poisson_paras {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_POISSON_PARA_H
