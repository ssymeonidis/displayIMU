/****************************************************************************
** Meta object code from reading C++ file 'pixmap.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "pixmap.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pixmap.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_pixmap_t {
    QByteArrayData data[8];
    char stringdata0[75];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_pixmap_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_pixmap_t qt_meta_stringdata_pixmap = {
    {
QT_MOC_LITERAL(0, 0, 6), // "pixmap"
QT_MOC_LITERAL(1, 7, 10), // "paintEvent"
QT_MOC_LITERAL(2, 18, 0), // ""
QT_MOC_LITERAL(3, 19, 12), // "updateParams"
QT_MOC_LITERAL(4, 32, 12), // "updateIndex1"
QT_MOC_LITERAL(5, 45, 5), // "value"
QT_MOC_LITERAL(6, 51, 12), // "updateIndex2"
QT_MOC_LITERAL(7, 64, 10) // "updatePlay"

    },
    "pixmap\0paintEvent\0\0updateParams\0"
    "updateIndex1\0value\0updateIndex2\0"
    "updatePlay"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_pixmap[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x0a /* Public */,
       3,    0,   40,    2, 0x0a /* Public */,
       4,    1,   41,    2, 0x0a /* Public */,
       6,    0,   44,    2, 0x0a /* Public */,
       7,    0,   45,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void pixmap::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        pixmap *_t = static_cast<pixmap *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->paintEvent(); break;
        case 1: _t->updateParams(); break;
        case 2: _t->updateIndex1((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->updateIndex2(); break;
        case 4: _t->updatePlay(); break;
        default: ;
        }
    }
}

const QMetaObject pixmap::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_pixmap.data,
      qt_meta_data_pixmap,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *pixmap::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *pixmap::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_pixmap.stringdata0))
        return static_cast<void*>(const_cast< pixmap*>(this));
    return QWidget::qt_metacast(_clname);
}

int pixmap::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
