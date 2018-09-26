/****************************************************************************
** Meta object code from reading C++ file 'windowGUI.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "windowGUI.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'windowGUI.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_windowGUI_t {
    QByteArrayData data[10];
    char stringdata0[145];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_windowGUI_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_windowGUI_t qt_meta_stringdata_windowGUI = {
    {
QT_MOC_LITERAL(0, 0, 9), // "windowGUI"
QT_MOC_LITERAL(1, 10, 11), // "config_read"
QT_MOC_LITERAL(2, 22, 0), // ""
QT_MOC_LITERAL(3, 23, 12), // "config_write"
QT_MOC_LITERAL(4, 36, 10), // "calib_read"
QT_MOC_LITERAL(5, 47, 11), // "calib_write"
QT_MOC_LITERAL(6, 59, 21), // "on_configOpen_clicked"
QT_MOC_LITERAL(7, 81, 21), // "on_configSave_clicked"
QT_MOC_LITERAL(8, 103, 20), // "on_calibOpen_clicked"
QT_MOC_LITERAL(9, 124, 20) // "on_calibSave_clicked"

    },
    "windowGUI\0config_read\0\0config_write\0"
    "calib_read\0calib_write\0on_configOpen_clicked\0"
    "on_configSave_clicked\0on_calibOpen_clicked\0"
    "on_calibSave_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_windowGUI[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   54,    2, 0x08 /* Private */,
       3,    0,   55,    2, 0x08 /* Private */,
       4,    0,   56,    2, 0x08 /* Private */,
       5,    0,   57,    2, 0x08 /* Private */,
       6,    0,   58,    2, 0x08 /* Private */,
       7,    0,   59,    2, 0x08 /* Private */,
       8,    0,   60,    2, 0x08 /* Private */,
       9,    0,   61,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void windowGUI::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        windowGUI *_t = static_cast<windowGUI *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->config_read(); break;
        case 1: _t->config_write(); break;
        case 2: _t->calib_read(); break;
        case 3: _t->calib_write(); break;
        case 4: _t->on_configOpen_clicked(); break;
        case 5: _t->on_configSave_clicked(); break;
        case 6: _t->on_calibOpen_clicked(); break;
        case 7: _t->on_calibSave_clicked(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject windowGUI::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_windowGUI.data,
      qt_meta_data_windowGUI,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *windowGUI::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *windowGUI::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_windowGUI.stringdata0))
        return static_cast<void*>(const_cast< windowGUI*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int windowGUI::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
