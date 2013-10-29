/****************************************************************************
** Meta object code from reading C++ file 'dlg_poisson_para.h'
**
** Created: Mon Oct 28 20:52:14 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../UI/dlg_poisson_para.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'dlg_poisson_para.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PoissonParaDlg[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      26,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      21,   15,   16,   15, 0x08,
      40,   35,   15,   15, 0x08,
      64,   35,   15,   15, 0x08,
      88,   35,   15,   15, 0x08,
     118,   15,   15,   15, 0x08,
     146,   15,   15,   15, 0x08,
     174,   15,   15,   15, 0x08,
     199,   15,   15,   15, 0x08,
     210,   15,   15,   15, 0x08,
     231,   15,   15,   15, 0x08,
     248,   15,   15,   15, 0x08,
     262,   15,   15,   15, 0x08,
     284,   15,   15,   15, 0x08,
     308,   15,   15,   15, 0x08,
     335,   15,   15,   15, 0x08,
     348,   35,   15,   15, 0x08,
     365,   35,   15,   15, 0x08,
     393,   35,   15,   15, 0x08,
     411,   35,   15,   15, 0x08,
     429,   35,   15,   15, 0x08,
     447,   35,   15,   15, 0x08,
     468,   35,   15,   15, 0x08,
     489,   35,   15,   15, 0x08,
     510,   35,   15,   15, 0x08,
     531,   15,   15,   15, 0x08,
     559,   15,   15,   15, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_PoissonParaDlg[] = {
    "PoissonParaDlg\0\0bool\0initWidgets()\0"
    "_val\0getRadiusValues(double)\0"
    "getPoissonDepth(double)\0"
    "getPoissonIsoInterval(double)\0"
    "runPoissonAndExtractLeafs()\0"
    "runPoissonAndExtractNodes()\0"
    "runPoissonAndExtractMC()\0runSlice()\0"
    "removeNonIsoPoints()\0labelIsoPoints()\0"
    "labelSmooth()\0labelBoundaryPoints()\0"
    "computeViewCandidates()\0"
    "viewCandidatesClustering()\0clearLabel()\0"
    "showSlices(bool)\0showSlicesTransparent(bool)\0"
    "showSlicesX(bool)\0showSlicesY(bool)\0"
    "showSlicesZ(bool)\0useConfidence1(bool)\0"
    "useConfidence2(bool)\0useConfidence3(bool)\0"
    "useConfidence4(bool)\0computeOriginalConfidence()\0"
    "computeSamplesConfidence()\0"
};

void PoissonParaDlg::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PoissonParaDlg *_t = static_cast<PoissonParaDlg *>(_o);
        switch (_id) {
        case 0: { bool _r = _t->initWidgets();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 1: _t->getRadiusValues((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 2: _t->getPoissonDepth((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 3: _t->getPoissonIsoInterval((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 4: _t->runPoissonAndExtractLeafs(); break;
        case 5: _t->runPoissonAndExtractNodes(); break;
        case 6: _t->runPoissonAndExtractMC(); break;
        case 7: _t->runSlice(); break;
        case 8: _t->removeNonIsoPoints(); break;
        case 9: _t->labelIsoPoints(); break;
        case 10: _t->labelSmooth(); break;
        case 11: _t->labelBoundaryPoints(); break;
        case 12: _t->computeViewCandidates(); break;
        case 13: _t->viewCandidatesClustering(); break;
        case 14: _t->clearLabel(); break;
        case 15: _t->showSlices((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 16: _t->showSlicesTransparent((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 17: _t->showSlicesX((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 18: _t->showSlicesY((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 19: _t->showSlicesZ((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 20: _t->useConfidence1((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 21: _t->useConfidence2((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 22: _t->useConfidence3((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 23: _t->useConfidence4((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 24: _t->computeOriginalConfidence(); break;
        case 25: _t->computeSamplesConfidence(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData PoissonParaDlg::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PoissonParaDlg::staticMetaObject = {
    { &QFrame::staticMetaObject, qt_meta_stringdata_PoissonParaDlg,
      qt_meta_data_PoissonParaDlg, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PoissonParaDlg::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PoissonParaDlg::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PoissonParaDlg::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PoissonParaDlg))
        return static_cast<void*>(const_cast< PoissonParaDlg*>(this));
    return QFrame::qt_metacast(_clname);
}

int PoissonParaDlg::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QFrame::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 26)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 26;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
