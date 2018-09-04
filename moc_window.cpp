/****************************************************************************
** Meta object code from reading C++ file 'window.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "window.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'window.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Window[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
       8,    7,    7,    7, 0x0a,
      22,    7,    7,    7, 0x0a,
      40,    7,    7,    7, 0x0a,
      57,    7,    7,    7, 0x0a,
      75,    7,    7,    7, 0x0a,
      92,    7,    7,    7, 0x0a,
     109,    7,    7,    7, 0x0a,
     124,    7,    7,    7, 0x0a,
     142,    7,    7,    7, 0x0a,
     160,    7,    7,    7, 0x0a,
     178,    7,    7,    7, 0x0a,
     195,    7,    7,    7, 0x0a,
     212,    7,    7,    7, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_Window[] = {
    "Window\0\0updateDebug()\0updateCheckAccl()\0"
    "updateCheckMag()\0updateCheckGyro()\0"
    "updateCheckIMU()\0updateStateIMU()\0"
    "updateViewUp()\0updateViewSide1()\0"
    "updateViewSide2()\0updateReference()\0"
    "updateResetIMU()\0updateCalibIMU()\0"
    "updateRefAccl()\0"
};

void Window::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Window *_t = static_cast<Window *>(_o);
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

const QMetaObjectExtraData Window::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Window::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_Window,
      qt_meta_data_Window, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Window::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Window::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Window::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Window))
        return static_cast<void*>(const_cast< Window*>(this));
    return QWidget::qt_metacast(_clname);
}

int Window::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
