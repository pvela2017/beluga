/****************************************************************************
** Meta object code from reading C++ file 'GLWindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../include/GLWindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'GLWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_GLWindow_t {
    QByteArrayData data[36];
    char stringdata0[393];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_GLWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_GLWindow_t qt_meta_stringdata_GLWindow = {
    {
QT_MOC_LITERAL(0, 0, 8), // "GLWindow"
QT_MOC_LITERAL(1, 9, 16), // "updateUsedGPUMem"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 10), // "toggleMesh"
QT_MOC_LITERAL(4, 38, 10), // "toggleBBox"
QT_MOC_LITERAL(5, 49, 13), // "toggleVectors"
QT_MOC_LITERAL(6, 63, 9), // "toggleVDB"
QT_MOC_LITERAL(7, 73, 10), // "toggleGrid"
QT_MOC_LITERAL(8, 84, 10), // "toggleCrop"
QT_MOC_LITERAL(9, 95, 15), // "setVectorLength"
QT_MOC_LITERAL(10, 111, 6), // "_delta"
QT_MOC_LITERAL(11, 118, 6), // "setLOD"
QT_MOC_LITERAL(12, 125, 4), // "_lod"
QT_MOC_LITERAL(13, 130, 12), // "setScanSpeed"
QT_MOC_LITERAL(14, 143, 6), // "_speed"
QT_MOC_LITERAL(15, 150, 13), // "setScanXWidth"
QT_MOC_LITERAL(16, 164, 2), // "_w"
QT_MOC_LITERAL(17, 167, 6), // "_index"
QT_MOC_LITERAL(18, 174, 14), // "setScanYHeight"
QT_MOC_LITERAL(19, 189, 2), // "_h"
QT_MOC_LITERAL(20, 192, 13), // "setScanZDepth"
QT_MOC_LITERAL(21, 206, 2), // "_d"
QT_MOC_LITERAL(22, 209, 11), // "toggleLevel"
QT_MOC_LITERAL(23, 221, 6), // "_level"
QT_MOC_LITERAL(24, 228, 12), // "toggleLevel0"
QT_MOC_LITERAL(25, 241, 12), // "toggleLevel1"
QT_MOC_LITERAL(26, 254, 12), // "toggleLevel2"
QT_MOC_LITERAL(27, 267, 12), // "toggleLevel3"
QT_MOC_LITERAL(28, 280, 7), // "runScan"
QT_MOC_LITERAL(29, 288, 5), // "_axis"
QT_MOC_LITERAL(30, 294, 9), // "pauseScan"
QT_MOC_LITERAL(31, 304, 10), // "resumeScan"
QT_MOC_LITERAL(32, 315, 8), // "stopScan"
QT_MOC_LITERAL(33, 324, 17), // "resetCameraVolume"
QT_MOC_LITERAL(34, 342, 20), // "resetVolumeTransform"
QT_MOC_LITERAL(35, 363, 29) // "resetCameraAndVolumeTransform"

    },
    "GLWindow\0updateUsedGPUMem\0\0toggleMesh\0"
    "toggleBBox\0toggleVectors\0toggleVDB\0"
    "toggleGrid\0toggleCrop\0setVectorLength\0"
    "_delta\0setLOD\0_lod\0setScanSpeed\0_speed\0"
    "setScanXWidth\0_w\0_index\0setScanYHeight\0"
    "_h\0setScanZDepth\0_d\0toggleLevel\0_level\0"
    "toggleLevel0\0toggleLevel1\0toggleLevel2\0"
    "toggleLevel3\0runScan\0_axis\0pauseScan\0"
    "resumeScan\0stopScan\0resetCameraVolume\0"
    "resetVolumeTransform\0resetCameraAndVolumeTransform"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_GLWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      25,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,  139,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,  140,    2, 0x0a /* Public */,
       4,    0,  141,    2, 0x0a /* Public */,
       5,    0,  142,    2, 0x0a /* Public */,
       6,    0,  143,    2, 0x0a /* Public */,
       7,    0,  144,    2, 0x0a /* Public */,
       8,    0,  145,    2, 0x0a /* Public */,
       9,    1,  146,    2, 0x0a /* Public */,
      11,    1,  149,    2, 0x0a /* Public */,
      13,    1,  152,    2, 0x0a /* Public */,
      15,    2,  155,    2, 0x0a /* Public */,
      18,    2,  160,    2, 0x0a /* Public */,
      20,    2,  165,    2, 0x0a /* Public */,
      22,    1,  170,    2, 0x0a /* Public */,
      24,    0,  173,    2, 0x0a /* Public */,
      25,    0,  174,    2, 0x0a /* Public */,
      26,    0,  175,    2, 0x0a /* Public */,
      27,    0,  176,    2, 0x0a /* Public */,
      28,    1,  177,    2, 0x0a /* Public */,
      30,    0,  180,    2, 0x0a /* Public */,
      31,    0,  181,    2, 0x0a /* Public */,
      32,    0,  182,    2, 0x0a /* Public */,
      33,    0,  183,    2, 0x0a /* Public */,
      34,    0,  184,    2, 0x0a /* Public */,
      35,    0,  185,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Int,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Float,   10,
    QMetaType::Void, QMetaType::Float,   12,
    QMetaType::Void, QMetaType::Float,   14,
    QMetaType::Void, QMetaType::Float, QMetaType::Int,   16,   17,
    QMetaType::Void, QMetaType::Float, QMetaType::Int,   19,   17,
    QMetaType::Void, QMetaType::Float, QMetaType::Int,   21,   17,
    QMetaType::Void, QMetaType::Int,   23,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   29,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void GLWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<GLWindow *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->updateUsedGPUMem(); break;
        case 1: { int _r = _t->toggleMesh();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 2: _t->toggleBBox(); break;
        case 3: _t->toggleVectors(); break;
        case 4: _t->toggleVDB(); break;
        case 5: _t->toggleGrid(); break;
        case 6: _t->toggleCrop(); break;
        case 7: _t->setVectorLength((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 8: _t->setLOD((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 9: _t->setScanSpeed((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 10: _t->setScanXWidth((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 11: _t->setScanYHeight((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 12: _t->setScanZDepth((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 13: _t->toggleLevel((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->toggleLevel0(); break;
        case 15: _t->toggleLevel1(); break;
        case 16: _t->toggleLevel2(); break;
        case 17: _t->toggleLevel3(); break;
        case 18: _t->runScan((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 19: _t->pauseScan(); break;
        case 20: _t->resumeScan(); break;
        case 21: _t->stopScan(); break;
        case 22: _t->resetCameraVolume(); break;
        case 23: _t->resetVolumeTransform(); break;
        case 24: _t->resetCameraAndVolumeTransform(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (GLWindow::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&GLWindow::updateUsedGPUMem)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject GLWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QGLWidget::staticMetaObject>(),
    qt_meta_stringdata_GLWindow.data,
    qt_meta_data_GLWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *GLWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *GLWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_GLWindow.stringdata0))
        return static_cast<void*>(this);
    return QGLWidget::qt_metacast(_clname);
}

int GLWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 25)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 25;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 25)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 25;
    }
    return _id;
}

// SIGNAL 0
void GLWindow::updateUsedGPUMem()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
