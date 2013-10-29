/****************************************************************************
** Meta object code from reading C++ file 'dlg_camera_para.h'
**
** Created: Mon Oct 28 21:35:51 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../UI/dlg_camera_para.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'dlg_camera_para.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_CameraParaDlg[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      20,   14,   15,   14, 0x08,
      34,   14,   14,   14, 0x08,
      48,   14,   14,   14, 0x08,
      62,   14,   14,   14, 0x08,
      90,   82,   14,   14, 0x08,
     112,   14,   14,   14, 0x08,
     142,   14,   14,   14, 0x08,
     177,  171,   14,   14, 0x08,
     218,  171,   14,   14, 0x08,
     255,   14,   14,   14, 0x08,
     291,  286,   14,   14, 0x08,
     320,  286,   14,   14, 0x08,
     350,  286,   14,   14, 0x08,
     375,  286,   14,   14, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_CameraParaDlg[] = {
    "CameraParaDlg\0\0bool\0initWidgets()\0"
    "virtualScan()\0initialScan()\0"
    "NBVCandidatesScan()\0is_show\0"
    "showInitCameras(bool)\0"
    "updateTableViewNBVCandidate()\0"
    "updateTabelViewScanResults()\0index\0"
    "showSelectedScannCandidates(QModelIndex)\0"
    "showSelectedScannedMesh(QModelIndex)\0"
    "mergeScannedMeshWithOriginal()\0_val\0"
    "getCameraHorizonDist(double)\0"
    "getCameraVerticalDist(double)\0"
    "getCameraMaxDist(double)\0"
    "getCameraDistToModel(double)\0"
};

void CameraParaDlg::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        CameraParaDlg *_t = static_cast<CameraParaDlg *>(_o);
        switch (_id) {
        case 0: { bool _r = _t->initWidgets();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 1: _t->virtualScan(); break;
        case 2: _t->initialScan(); break;
        case 3: _t->NBVCandidatesScan(); break;
        case 4: _t->showInitCameras((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->updateTableViewNBVCandidate(); break;
        case 6: _t->updateTabelViewScanResults(); break;
        case 7: _t->showSelectedScannCandidates((*reinterpret_cast< QModelIndex(*)>(_a[1]))); break;
        case 8: _t->showSelectedScannedMesh((*reinterpret_cast< QModelIndex(*)>(_a[1]))); break;
        case 9: _t->mergeScannedMeshWithOriginal(); break;
        case 10: _t->getCameraHorizonDist((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 11: _t->getCameraVerticalDist((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 12: _t->getCameraMaxDist((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 13: _t->getCameraDistToModel((*reinterpret_cast< double(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData CameraParaDlg::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject CameraParaDlg::staticMetaObject = {
    { &QFrame::staticMetaObject, qt_meta_stringdata_CameraParaDlg,
      qt_meta_data_CameraParaDlg, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &CameraParaDlg::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *CameraParaDlg::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *CameraParaDlg::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_CameraParaDlg))
        return static_cast<void*>(const_cast< CameraParaDlg*>(this));
    return QFrame::qt_metacast(_clname);
}

int CameraParaDlg::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QFrame::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 14)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 14;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
