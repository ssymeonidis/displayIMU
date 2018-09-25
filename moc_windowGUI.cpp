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
struct qt_meta_stringdata_WindowGUI_t {
    QByteArrayData data[15];
    char stringdata0[205];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_WindowGUI_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_WindowGUI_t qt_meta_stringdata_WindowGUI = {
    {
QT_MOC_LITERAL(0, 0, 9), // "WindowGUI"
QT_MOC_LITERAL(1, 10, 11), // "updateDebug"
QT_MOC_LITERAL(2, 22, 0), // ""
QT_MOC_LITERAL(3, 23, 15), // "updateCheckAccl"
QT_MOC_LITERAL(4, 39, 14), // "updateCheckMag"
QT_MOC_LITERAL(5, 54, 15), // "updateCheckGyro"
QT_MOC_LITERAL(6, 70, 14), // "updateCheckIMU"
QT_MOC_LITERAL(7, 85, 14), // "updateStateIMU"
QT_MOC_LITERAL(8, 100, 12), // "updateViewUp"
QT_MOC_LITERAL(9, 113, 15), // "updateViewSide1"
QT_MOC_LITERAL(10, 129, 15), // "updateViewSide2"
QT_MOC_LITERAL(11, 145, 15), // "updateReference"
QT_MOC_LITERAL(12, 161, 14), // "updateResetIMU"
QT_MOC_LITERAL(13, 176, 14), // "updateCalibIMU"
QT_MOC_LITERAL(14, 191, 13) // "updateRefAccl"

    },
    "WindowGUI\0updateDebug\0\0updateCheckAccl\0"
    "updateCheckMag\0updateCheckGyro\0"
    "updateCheckIMU\0updateStateIMU\0"
    "updateViewUp\0updateViewSide1\0"
    "updateViewSide2\0updateReference\0"
    "updateResetIMU\0updateCalibIMU\0"
    "updateRefAccl"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_WindowGUI[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   79,    2, 0x0a /* Public */,
       3,    0,   80,    2, 0x0a /* Public */,
       4,    0,   81,    2, 0x0a /* Public */,
       5,    0,   82,    2, 0x0a /* Public */,
       6,    0,   83,    2, 0x0a /* Public */,
       7,    0,   84,    2, 0x0a /* Public */,
       8,    0,   85,    2, 0x0a /* Public */,
       9,    0,   86,    2, 0x0a /* Public */,
      10,    0,   87,    2, 0x0a /* Public */,
      11,    0,   88,    2, 0x0a /* Public */,
      12,    0,   89,    2, 0x0a /* Public */,
      13,    0,   90,    2, 0x0a /* Public */,
      14,    0,   91,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
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

void WindowGUI::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        WindowGUI *_t = static_cast<WindowGUI *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateDebug(); break;
        case 1: _t->updateCheckAccl(); break;
        case 2: _t->updateCheckMag(); break;
        case 3: _t->updateCheckGyro(); break;
        case 4: _t->updateCheckIMU(); break;
        case 5: _t->updateStateIMU(); break;
        case 6: _t->updateViewUp(); break;
        case 7: _t->updateViewSide1(); break;
        case 8: _t->updateViewSide2(); break;
        case 9: _t->updateReference(); break;
        case 10: _t->updateResetIMU(); break;
        case 11: _t->updateCalibIMU(); break;
        case 12: _t->updateRefAccl(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject WindowGUI::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_WindowGUI.data,
      qt_meta_data_WindowGUI,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *WindowGUI::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *WindowGUI::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_WindowGUI.stringdata0))
        return static_cast<void*>(const_cast< WindowGUI*>(this));
    return QWidget::qt_metacast(_clname);
}

int WindowGUI::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
