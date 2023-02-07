/****************************************************************************
** Meta object code from reading C++ file 'onroad.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "onroad.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'onroad.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_OnroadHud_t {
    QByteArrayData data[16];
    char stringdata0[155];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_OnroadHud_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_OnroadHud_t qt_meta_stringdata_OnroadHud = {
    {
QT_MOC_LITERAL(0, 0, 9), // "OnroadHud"
QT_MOC_LITERAL(1, 10, 12), // "valueChanged"
QT_MOC_LITERAL(2, 23, 0), // ""
QT_MOC_LITERAL(3, 24, 5), // "speed"
QT_MOC_LITERAL(4, 30, 9), // "speedUnit"
QT_MOC_LITERAL(5, 40, 8), // "maxSpeed"
QT_MOC_LITERAL(6, 49, 13), // "is_cruise_set"
QT_MOC_LITERAL(7, 63, 10), // "engageable"
QT_MOC_LITERAL(8, 74, 8), // "dmActive"
QT_MOC_LITERAL(9, 83, 6), // "hideDM"
QT_MOC_LITERAL(10, 90, 6), // "status"
QT_MOC_LITERAL(11, 97, 6), // "paused"
QT_MOC_LITERAL(12, 104, 7), // "enabled"
QT_MOC_LITERAL(13, 112, 11), // "lkasEnabled"
QT_MOC_LITERAL(14, 124, 15), // "steeringPressed"
QT_MOC_LITERAL(15, 140, 14) // "steeringTorque"

    },
    "OnroadHud\0valueChanged\0\0speed\0speedUnit\0"
    "maxSpeed\0is_cruise_set\0engageable\0"
    "dmActive\0hideDM\0status\0paused\0enabled\0"
    "lkasEnabled\0steeringPressed\0steeringTorque"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_OnroadHud[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
      13,   20, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   19,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void,

 // properties: name, type, flags
       3, QMetaType::QString, 0x00495003,
       4, QMetaType::QString, 0x00495003,
       5, QMetaType::QString, 0x00495003,
       6, QMetaType::Bool, 0x00495003,
       7, QMetaType::Bool, 0x00495003,
       8, QMetaType::Bool, 0x00495003,
       9, QMetaType::Bool, 0x00495003,
      10, QMetaType::Int, 0x00495003,
      11, QMetaType::Bool, 0x00495003,
      12, QMetaType::Bool, 0x00495003,
      13, QMetaType::Bool, 0x00495003,
      14, QMetaType::Bool, 0x00495003,
      15, QMetaType::Float, 0x00495003,

 // properties: notify_signal_id
       0,
       0,
       0,
       0,
       0,
       0,
       0,
       0,
       0,
       0,
       0,
       0,
       0,

       0        // eod
};

void OnroadHud::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<OnroadHud *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->valueChanged(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (OnroadHud::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&OnroadHud::valueChanged)) {
                *result = 0;
                return;
            }
        }
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty) {
        auto *_t = static_cast<OnroadHud *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< QString*>(_v) = _t->speed; break;
        case 1: *reinterpret_cast< QString*>(_v) = _t->speedUnit; break;
        case 2: *reinterpret_cast< QString*>(_v) = _t->maxSpeed; break;
        case 3: *reinterpret_cast< bool*>(_v) = _t->is_cruise_set; break;
        case 4: *reinterpret_cast< bool*>(_v) = _t->engageable; break;
        case 5: *reinterpret_cast< bool*>(_v) = _t->dmActive; break;
        case 6: *reinterpret_cast< bool*>(_v) = _t->hideDM; break;
        case 7: *reinterpret_cast< int*>(_v) = _t->status; break;
        case 8: *reinterpret_cast< bool*>(_v) = _t->paused; break;
        case 9: *reinterpret_cast< bool*>(_v) = _t->enabled; break;
        case 10: *reinterpret_cast< bool*>(_v) = _t->lkasEnabled; break;
        case 11: *reinterpret_cast< bool*>(_v) = _t->steeringPressed; break;
        case 12: *reinterpret_cast< float*>(_v) = _t->steeringTorque; break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        auto *_t = static_cast<OnroadHud *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0:
            if (_t->speed != *reinterpret_cast< QString*>(_v)) {
                _t->speed = *reinterpret_cast< QString*>(_v);
                Q_EMIT _t->valueChanged();
            }
            break;
        case 1:
            if (_t->speedUnit != *reinterpret_cast< QString*>(_v)) {
                _t->speedUnit = *reinterpret_cast< QString*>(_v);
                Q_EMIT _t->valueChanged();
            }
            break;
        case 2:
            if (_t->maxSpeed != *reinterpret_cast< QString*>(_v)) {
                _t->maxSpeed = *reinterpret_cast< QString*>(_v);
                Q_EMIT _t->valueChanged();
            }
            break;
        case 3:
            if (_t->is_cruise_set != *reinterpret_cast< bool*>(_v)) {
                _t->is_cruise_set = *reinterpret_cast< bool*>(_v);
                Q_EMIT _t->valueChanged();
            }
            break;
        case 4:
            if (_t->engageable != *reinterpret_cast< bool*>(_v)) {
                _t->engageable = *reinterpret_cast< bool*>(_v);
                Q_EMIT _t->valueChanged();
            }
            break;
        case 5:
            if (_t->dmActive != *reinterpret_cast< bool*>(_v)) {
                _t->dmActive = *reinterpret_cast< bool*>(_v);
                Q_EMIT _t->valueChanged();
            }
            break;
        case 6:
            if (_t->hideDM != *reinterpret_cast< bool*>(_v)) {
                _t->hideDM = *reinterpret_cast< bool*>(_v);
                Q_EMIT _t->valueChanged();
            }
            break;
        case 7:
            if (_t->status != *reinterpret_cast< int*>(_v)) {
                _t->status = *reinterpret_cast< int*>(_v);
                Q_EMIT _t->valueChanged();
            }
            break;
        case 8:
            if (_t->paused != *reinterpret_cast< bool*>(_v)) {
                _t->paused = *reinterpret_cast< bool*>(_v);
                Q_EMIT _t->valueChanged();
            }
            break;
        case 9:
            if (_t->enabled != *reinterpret_cast< bool*>(_v)) {
                _t->enabled = *reinterpret_cast< bool*>(_v);
                Q_EMIT _t->valueChanged();
            }
            break;
        case 10:
            if (_t->lkasEnabled != *reinterpret_cast< bool*>(_v)) {
                _t->lkasEnabled = *reinterpret_cast< bool*>(_v);
                Q_EMIT _t->valueChanged();
            }
            break;
        case 11:
            if (_t->steeringPressed != *reinterpret_cast< bool*>(_v)) {
                _t->steeringPressed = *reinterpret_cast< bool*>(_v);
                Q_EMIT _t->valueChanged();
            }
            break;
        case 12:
            if (_t->steeringTorque != *reinterpret_cast< float*>(_v)) {
                _t->steeringTorque = *reinterpret_cast< float*>(_v);
                Q_EMIT _t->valueChanged();
            }
            break;
        default: break;
        }
    } else if (_c == QMetaObject::ResetProperty) {
    }
#endif // QT_NO_PROPERTIES
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject OnroadHud::staticMetaObject = { {
    &QWidget::staticMetaObject,
    qt_meta_stringdata_OnroadHud.data,
    qt_meta_data_OnroadHud,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *OnroadHud::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *OnroadHud::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_OnroadHud.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int OnroadHud::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 1)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 1;
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 13;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 13;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 13;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 13;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 13;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}

// SIGNAL 0
void OnroadHud::valueChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
struct qt_meta_stringdata_OnroadAlerts_t {
    QByteArrayData data[1];
    char stringdata0[13];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_OnroadAlerts_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_OnroadAlerts_t qt_meta_stringdata_OnroadAlerts = {
    {
QT_MOC_LITERAL(0, 0, 12) // "OnroadAlerts"

    },
    "OnroadAlerts"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_OnroadAlerts[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void OnroadAlerts::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject OnroadAlerts::staticMetaObject = { {
    &QWidget::staticMetaObject,
    qt_meta_stringdata_OnroadAlerts.data,
    qt_meta_data_OnroadAlerts,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *OnroadAlerts::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *OnroadAlerts::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_OnroadAlerts.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int OnroadAlerts::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_NvgWindow_t {
    QByteArrayData data[1];
    char stringdata0[10];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_NvgWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_NvgWindow_t qt_meta_stringdata_NvgWindow = {
    {
QT_MOC_LITERAL(0, 0, 9) // "NvgWindow"

    },
    "NvgWindow"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_NvgWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void NvgWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject NvgWindow::staticMetaObject = { {
    &CameraViewWidget::staticMetaObject,
    qt_meta_stringdata_NvgWindow.data,
    qt_meta_data_NvgWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *NvgWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *NvgWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_NvgWindow.stringdata0))
        return static_cast<void*>(this);
    return CameraViewWidget::qt_metacast(_clname);
}

int NvgWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = CameraViewWidget::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_OnroadWindow_t {
    QByteArrayData data[9];
    char stringdata0[104];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_OnroadWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_OnroadWindow_t qt_meta_stringdata_OnroadWindow = {
    {
QT_MOC_LITERAL(0, 0, 12), // "OnroadWindow"
QT_MOC_LITERAL(1, 13, 17), // "updateStateSignal"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 7), // "UIState"
QT_MOC_LITERAL(4, 40, 1), // "s"
QT_MOC_LITERAL(5, 42, 23), // "offroadTransitionSignal"
QT_MOC_LITERAL(6, 66, 7), // "offroad"
QT_MOC_LITERAL(7, 74, 17), // "offroadTransition"
QT_MOC_LITERAL(8, 92, 11) // "updateState"

    },
    "OnroadWindow\0updateStateSignal\0\0UIState\0"
    "s\0offroadTransitionSignal\0offroad\0"
    "offroadTransition\0updateState"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_OnroadWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,
       5,    1,   37,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    1,   40,    2, 0x08 /* Private */,
       8,    1,   43,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::Bool,    6,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void, 0x80000000 | 3,    4,

       0        // eod
};

void OnroadWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<OnroadWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateStateSignal((*reinterpret_cast< const UIState(*)>(_a[1]))); break;
        case 1: _t->offroadTransitionSignal((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->offroadTransition((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->updateState((*reinterpret_cast< const UIState(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (OnroadWindow::*)(const UIState & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&OnroadWindow::updateStateSignal)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (OnroadWindow::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&OnroadWindow::offroadTransitionSignal)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject OnroadWindow::staticMetaObject = { {
    &QWidget::staticMetaObject,
    qt_meta_stringdata_OnroadWindow.data,
    qt_meta_data_OnroadWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *OnroadWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *OnroadWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_OnroadWindow.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int OnroadWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void OnroadWindow::updateStateSignal(const UIState & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void OnroadWindow::offroadTransitionSignal(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
