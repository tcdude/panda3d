/**
 * PANDA 3D SOFTWARE
 * Copyright (c) Carnegie Mellon University.  All rights reserved.
 *
 * All use of this software is subject to the terms of the revised BSD
 * license.  You should have received a copy of this license along
 * with this source code in a file named "LICENSE."
 *
 * @file dtool_super_base.I
 * @author rdb
 * @date 2018-09-09
 */

#include "py_panda.h"

#ifdef HAVE_PYTHON

inline void Dtool_FreeInstance_DTOOL_SUPER_BASE(PyObject *self) {
  Py_TYPE(self)->tp_free(self);
}

inline PyObject *GetSuperBase(PyObject *self) {
  Dtool_PyTypedObject *type = Dtool_GetSuperBase();
  Py_INCREF((PyTypeObject *)type);
  return (PyObject *)type;
};

inline void *Dtool_DowncastInterface_DTOOL_SUPER_BASE(void *from_this, Dtool_PyTypedObject *from_type) {
  return nullptr;
}

inline void *Dtool_UpcastInterface_DTOOL_SUPER_BASE(PyObject *self, Dtool_PyTypedObject *requested_type) {
  return nullptr;
}

inline int Dtool_Init_DTOOL_SUPER_BASE(PyObject *self, PyObject *args, PyObject *kwds) {
  assert(self != nullptr);
  PyErr_Format(PyExc_TypeError, "cannot init constant class %s", Py_TYPE(self)->tp_name);
  return -1;
}

inline void Dtool_PyModuleClassInit_DTOOL_SUPER_BASE(PyObject *module) {
  Dtool_PyTypedObject *base = Dtool_GetSuperBase();

  if (module != nullptr && base != nullptr) {
    Py_INCREF(base);
    PyModule_AddObject(module, "DTOOL_SUPER_BASE", (PyObject *)base);
  }
}

/**
 * Borrows a reference to the DTOOL_SUPER_BASE class.
 */
inline Dtool_PyTypedObject *Dtool_GetSuperBase() {
  static Dtool_PyTypedObject *ptr = nullptr;
  if (ptr != nullptr) {
    return ptr;
  }

  // Did a different module already define this type?
  TypeRegistry *reg = TypeRegistry::ptr();
  if (reg != nullptr) {
    PyObject *type = reg->find_python_type("DTOOL_SUPER_BASE");
    if (type != nullptr) {
      ptr = (Dtool_PyTypedObject *)type;
      return (Dtool_PyTypedObject *)type;
    }
  }

  // No, we will define it on the spot.
  static PyMethodDef Dtool_Methods_DTOOL_SUPER_BASE[] = {
    { "DtoolGetSuperBase", (PyCFunction)&GetSuperBase, METH_NOARGS, "Will Return SUPERbase Class"},
    { nullptr, nullptr, 0, nullptr }
  };

  static PyMemberDef standard_type_members[] = {
    {(char *)"this", (sizeof(void*) == sizeof(int)) ? T_UINT : T_ULONGLONG, offsetof(Dtool_PyInstDef, _ptr_to_object), READONLY, (char *)"C++ 'this' pointer, if any"},
    {(char *)"this_ownership", T_BOOL, offsetof(Dtool_PyInstDef, _memory_rules), READONLY, (char *)"C++ 'this' ownership rules"},
    {(char *)"this_const", T_BOOL, offsetof(Dtool_PyInstDef, _is_const), READONLY, (char *)"C++ 'this' const flag"},
  // {(char *)"this_signature", T_INT, offsetof(Dtool_PyInstDef, _signature),
  // READONLY, (char *)"A type check signature"},
    {(char *)"this_metatype", T_OBJECT, offsetof(Dtool_PyInstDef, _My_Type), READONLY, (char *)"The dtool meta object"},
    {nullptr}  /* Sentinel */
  };

  static Dtool_PyTypedObject Dtool_DTOOL_SUPER_BASE = {
    {
      PyVarObject_HEAD_INIT(nullptr, 0)
      "dtoolconfig.DTOOL_SUPER_BASE",
      sizeof(Dtool_PyInstDef),
      0, // tp_itemsize
      &Dtool_FreeInstance_DTOOL_SUPER_BASE,
      nullptr, // tp_print
      nullptr, // tp_getattr
      nullptr, // tp_setattr
#if PY_MAJOR_VERSION >= 3
      nullptr, // tp_compare
#else
      &DtoolInstance_ComparePointers,
#endif
      nullptr, // tp_repr
      nullptr, // tp_as_number
      nullptr, // tp_as_sequence
      nullptr, // tp_as_mapping
      &DtoolInstance_HashPointer,
      nullptr, // tp_call
      nullptr, // tp_str
      PyObject_GenericGetAttr,
      PyObject_GenericSetAttr,
      nullptr, // tp_as_buffer
      (Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE | Py_TPFLAGS_CHECKTYPES),
      nullptr, // tp_doc
      nullptr, // tp_traverse
      nullptr, // tp_clear
#if PY_MAJOR_VERSION >= 3
      &DtoolInstance_RichComparePointers,
#else
      nullptr, // tp_richcompare
#endif
      0, // tp_weaklistoffset
      nullptr, // tp_iter
      nullptr, // tp_iternext
      Dtool_Methods_DTOOL_SUPER_BASE,
      standard_type_members,
      nullptr, // tp_getset
      nullptr, // tp_base
      nullptr, // tp_dict
      nullptr, // tp_descr_get
      nullptr, // tp_descr_set
      0, // tp_dictoffset
      Dtool_Init_DTOOL_SUPER_BASE,
      PyType_GenericAlloc,
      nullptr, // tp_new //Dtool_new_DTOOL_SUPER_BASE,
      PyObject_Del,
      nullptr, // tp_is_gc
      nullptr, // tp_bases
      nullptr, // tp_mro
      nullptr, // tp_cache
      nullptr, // tp_subclasses
      nullptr, // tp_weaklist
      nullptr, // tp_del
    },
    TypeHandle::none(),
    Dtool_PyModuleClassInit_DTOOL_SUPER_BASE,
    Dtool_UpcastInterface_DTOOL_SUPER_BASE,
    Dtool_DowncastInterface_DTOOL_SUPER_BASE,
    nullptr,
    nullptr,
  };

  ptr = &Dtool_DTOOL_SUPER_BASE;

  Dtool_DTOOL_SUPER_BASE._PyType.tp_dict = PyDict_New();
  PyDict_SetItemString(Dtool_DTOOL_SUPER_BASE._PyType.tp_dict, "DtoolClassDict", Dtool_DTOOL_SUPER_BASE._PyType.tp_dict);

  if (PyType_Ready((PyTypeObject *)&Dtool_DTOOL_SUPER_BASE) < 0) {
    PyErr_SetString(PyExc_TypeError, "PyType_Ready(Dtool_DTOOL_SUPER_BASE)");
    return nullptr;
  }
  Py_INCREF((PyTypeObject *)&Dtool_DTOOL_SUPER_BASE);

  PyDict_SetItemString(Dtool_DTOOL_SUPER_BASE._PyType.tp_dict, "DtoolGetSuperBase", PyCFunction_New(&Dtool_Methods_DTOOL_SUPER_BASE[0], (PyObject *)&Dtool_DTOOL_SUPER_BASE));

  if (reg != nullptr) {
    reg->register_python_type("DTOOL_SUPER_BASE", (PyObject *)&Dtool_DTOOL_SUPER_BASE);
  }
  return &Dtool_DTOOL_SUPER_BASE;
}

#endif  // HAVE_PYTHON
