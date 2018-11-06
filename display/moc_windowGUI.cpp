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
    QByteArrayData data[18];
    char stringdata0[329];
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
QT_MOC_LITERAL(6, 59, 15), // "glWidget_update"
QT_MOC_LITERAL(7, 75, 20), // "on_core_open_clicked"
QT_MOC_LITERAL(8, 96, 20), // "on_core_save_clicked"
QT_MOC_LITERAL(9, 117, 20), // "on_rect_open_clicked"
QT_MOC_LITERAL(10, 138, 20), // "on_rect_save_clicked"
QT_MOC_LITERAL(11, 159, 26), // "on_disp_enableGyro_clicked"
QT_MOC_LITERAL(12, 186, 26), // "on_disp_enableAccl_clicked"
QT_MOC_LITERAL(13, 213, 26), // "on_disp_enableMagn_clicked"
QT_MOC_LITERAL(14, 240, 25), // "on_disp_enableIMU_clicked"
QT_MOC_LITERAL(15, 266, 18), // "on_view_up_clicked"
QT_MOC_LITERAL(16, 285, 21), // "on_view_side1_clicked"
QT_MOC_LITERAL(17, 307, 21) // "on_view_side2_clicked"

    },
    "windowGUI\0config_read\0\0config_write\0"
    "calib_read\0calib_write\0glWidget_update\0"
    "on_core_open_clicked\0on_core_save_clicked\0"
    "on_rect_open_clicked\0on_rect_save_clicked\0"
    "on_disp_enableGyro_clicked\0"
    "on_disp_enableAccl_clicked\0"
    "on_disp_enableMagn_clicked\0"
    "on_disp_enableIMU_clicked\0on_view_up_clicked\0"
    "on_view_side1_clicked\0on_view_side2_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_windowGUI[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   94,    2, 0x08 /* Private */,
       3,    0,   95,    2, 0x08 /* Private */,
       4,    0,   96,    2, 0x08 /* Private */,
       5,    0,   97,    2, 0x08 /* Private */,
       6,    0,   98,    2, 0x08 /* Private */,
       7,    0,   99,    2, 0x08 /* Private */,
       8,    0,  100,    2, 0x08 /* Private */,
       9,    0,  101,    2, 0x08 /* Private */,
      10,    0,  102,    2, 0x08 /* Private */,
      11,    0,  103,    2, 0x08 /* Private */,
      12,    0,  104,    2, 0x08 /* Private */,
      13,    0,  105,    2, 0x08 /* Private */,
      14,    0,  106,    2, 0x08 /* Private */,
      15,    0,  107,    2, 0x08 /* Private */,
      16,    0,  108,    2, 0x08 /* Private */,
      17,    0,  109,    2, 0x08 /* Private */,

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
        case 4: _t->glWidget_update(); break;
        case 5: _t->on_core_open_clicked(); break;
        case 6: _t->on_core_save_clicked(); break;
        case 7: _t->on_rect_open_clicked(); break;
        case 8: _t->on_rect_save_clicked(); break;
        case 9: _t->on_disp_enableGyro_clicked(); break;
        case 10: _t->on_disp_enableAccl_clicked(); break;
        case 11: _t->on_disp_enableMagn_clicked(); break;
        case 12: _t->on_disp_enableIMU_clicked(); break;
        case 13: _t->on_view_up_clicked(); break;
        case 14: _t->on_view_side1_clicked(); break;
        case 15: _t->on_view_side2_clicked(); break;
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
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 16)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 16;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
