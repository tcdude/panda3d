/**
 * PANDA 3D SOFTWARE
 * Copyright (c) Carnegie Mellon University.  All rights reserved.
 *
 * All use of this software is subject to the terms of the revised BSD
 * license.  You should have received a copy of this license along
 * with this source code in a file named "LICENSE."
 *
 * @file py_panda.I
 * @author rdb
 * @date 2016-06-06
 */

#ifdef _MSC_VER
#define _IS_FINAL(T) (__is_sealed(T))
#elif defined(__GNUC__)
#define _IS_FINAL(T) (__is_final(T))
#else
#define _IS_FINAL(T) (0)
#endif

/**
 *
 */
INLINE void
DTOOL_Call_ExtractThisPointerForType(PyObject *self, Dtool_PyTypedObject *classdef, void **answer) {
  if (DtoolInstance_Check(self)) {
    *answer = DtoolInstance_UPCAST(self, *classdef);
  } else {
    *answer = nullptr;
  }
}

/**
 * This is a support function for the Python bindings: it extracts the
 * underlying C++ pointer of the given type for a given Python object.  If it
 * was of the wrong type, raises an AttributeError.
 */
INLINE bool
Dtool_Call_ExtractThisPointer(PyObject *self, Dtool_PyTypedObject &classdef, void **answer) {
  if (self == nullptr || !DtoolInstance_Check(self) || DtoolInstance_VOID_PTR(self) == nullptr) {
    Dtool_Raise_TypeError("C++ object is not yet constructed, or already destructed.");
    return false;
  }

  *answer = DtoolInstance_UPCAST(self, classdef);
  return true;
}

/**
 * The same thing as Dtool_Call_ExtractThisPointer, except that it performs
 * the additional check that the pointer is a non-const pointer.  This is
 * called by function wrappers for functions of which all overloads are non-
 * const, and saves a bit of code.
 *
 * The extra method_name argument is used in formatting the error message.
 */
INLINE bool
Dtool_Call_ExtractThisPointer_NonConst(PyObject *self, Dtool_PyTypedObject &classdef,
                                       void **answer, const char *method_name) {

  if (self == nullptr || !DtoolInstance_Check(self) || DtoolInstance_VOID_PTR(self) == nullptr) {
    Dtool_Raise_TypeError("C++ object is not yet constructed, or already destructed.");
    return false;
  }

  if (DtoolInstance_IS_CONST(self)) {
    // All overloads of this function are non-const.
    PyErr_Format(PyExc_TypeError,
                 "Cannot call %s() on a const object.",
                 method_name);
    return false;
  }

  *answer = DtoolInstance_UPCAST(self, classdef);
  return true;
}

/**
 * Extracts the C++ pointer for an object, given its Python wrapper object,
 * for passing as the parameter to a C++ function.
 *
 * self is the Python wrapper object in question.
 *
 * classdef is the Python class wrapper for the C++ class in which the this
 * pointer should be returned.  (This may require an upcast operation, if self
 * is not already an instance of classdef.)
 *
 * param and function_name are used for error reporting only, and describe the
 * particular function and parameter index for this parameter.
 *
 * const_ok is true if the function is declared const and can therefore be
 * called with either a const or non-const "this" pointer, or false if the
 * function is declared non-const, and can therefore be called with only a
 * non-const "this" pointer.
 *
 * The return value is the C++ pointer that was extracted, or NULL if there
 * was a problem (in which case the Python exception state will have been
 * set).
 */
INLINE void *
DTOOL_Call_GetPointerThisClass(PyObject *self, Dtool_PyTypedObject *classdef,
                               int param, const string &function_name, bool const_ok,
                               bool report_errors) {
  // if (PyErr_Occurred()) { return nullptr; }
  if (self == nullptr) {
    if (report_errors) {
      return Dtool_Raise_TypeError("self is nullptr");
    }
    return nullptr;
  }

  if (DtoolInstance_Check(self)) {
    void *result = DtoolInstance_UPCAST(self, *classdef);

    if (result != nullptr) {
      if (const_ok || !DtoolInstance_IS_CONST(self)) {
        return result;
      }

      if (report_errors) {
        return PyErr_Format(PyExc_TypeError,
                            "%s() argument %d may not be const",
                            function_name.c_str(), param);
      }
      return nullptr;
    }
  }

  if (report_errors) {
    return Dtool_Raise_ArgTypeError(self, param, function_name.c_str(), classdef->_PyType.tp_name);
  }

  return nullptr;
}

/**
 * Template function that can be used to extract any TypedObject pointer from
 * a wrapped Python object.
 */
template<class T> INLINE bool
DtoolInstance_GetPointer(PyObject *self, T *&into) {
  if (DtoolInstance_Check(self)) {
    Dtool_PyTypedObject *target_class = (Dtool_PyTypedObject *)get_type_handle(T).get_python_type();
    if (target_class != nullptr) {
      if (_IS_FINAL(T)) {
        if (DtoolInstance_TYPE(self) == target_class) {
          into = (T *)DtoolInstance_VOID_PTR(self);
        } else {
          return false;
        }
      } else {
        into = (T *)DtoolInstance_UPCAST(self, *target_class);
      }
      return (into != nullptr);
    }
  }
  into = nullptr;
  return false;
}

/**
 * Template function that can be used to extract any TypedObject pointer from
 * a wrapped Python object.  In this case, the Dtool_PyTypedObject is known.
 */
template<class T> INLINE bool
DtoolInstance_GetPointer(PyObject *self, T *&into, Dtool_PyTypedObject &target_class) {
  if (DtoolInstance_Check(self)) {
    if (_IS_FINAL(T)) {
      if (DtoolInstance_TYPE(self) == &target_class) {
        into = (T *)DtoolInstance_VOID_PTR(self);
      } else {
        return false;
      }
    } else {
      into = (T *)DtoolInstance_UPCAST(self, target_class);
    }
    return (into != nullptr);
  }
  into = nullptr;
  return false;
}

/**
 * Function to create a hash from a wrapped Python object.
 */
INLINE Py_hash_t DtoolInstance_HashPointer(PyObject *self) {
  if (self != nullptr && DtoolInstance_Check(self)) {
    return (Py_hash_t)(intptr_t)DtoolInstance_VOID_PTR(self);
  }
  return -1;
}

/**
 * Python 2-style comparison function that compares objects by pointer.
 */
INLINE int DtoolInstance_ComparePointers(PyObject *v1, PyObject *v2) {
  void *v1_this = DtoolInstance_Check(v1) ? DtoolInstance_VOID_PTR(v1) : nullptr;
  void *v2_this = DtoolInstance_Check(v2) ? DtoolInstance_VOID_PTR(v2) : nullptr;
  if (v1_this != nullptr && v2_this != nullptr) {
    return (v1_this > v2_this) - (v1_this < v2_this);
  } else {
    return (v1 > v2) - (v1 < v2);
  }
}

/**
 * Rich comparison function that compares objects by pointer.
 */
INLINE PyObject *DtoolInstance_RichComparePointers(PyObject *v1, PyObject *v2, int op) {
  int cmpval = DtoolInstance_ComparePointers(v1, v2);
  Py_RETURN_RICHCOMPARE(cmpval, 0, op);
}


/**
 * This is similar to a PyErr_Occurred() check, except that it also checks
 * Notify to see if an assertion has occurred.  If that is the case, then it
 * raises an AssertionError.
 *
 * Returns true if there is an active exception, false otherwise.
 *
 * In the NDEBUG case, this is simply a #define to _PyErr_OCCURRED() (which is
 * an undocumented inline version of PyErr_Occurred()).
 */
INLINE bool _Dtool_CheckErrorOccurred() {
  if (_PyErr_OCCURRED()) {
    return true;
  }
  if (Notify::ptr()->has_assert_failed()) {
    Dtool_Raise_AssertionError();
    return true;
  }
  return false;
}

/**
 * Raises an AssertionError containing the last thrown assert message, and
 * clears the assertion flag.  Returns NULL.
 */
INLINE PyObject *Dtool_Raise_AssertionError() {
  Notify *notify = Notify::ptr();
#if PY_MAJOR_VERSION >= 3
  PyObject *message = PyUnicode_FromString(notify->get_assert_error_message().c_str());
#else
  PyObject *message = PyString_FromString(notify->get_assert_error_message().c_str());
#endif
  Py_INCREF(PyExc_AssertionError);
  PyErr_Restore(PyExc_AssertionError, message, nullptr);
  notify->clear_assert_failed();
  return nullptr;
}

/**
 * Raises a TypeError with the given message, and returns NULL.
 */
INLINE PyObject *Dtool_Raise_TypeError(const char *message) {
  // PyErr_Restore is what PyErr_SetString would have ended up calling
  // eventually anyway, so we might as well just get to the point.
  Py_INCREF(PyExc_TypeError);
#if PY_MAJOR_VERSION >= 3
  PyErr_Restore(PyExc_TypeError, PyUnicode_FromString(message), nullptr);
#else
  PyErr_Restore(PyExc_TypeError, PyString_FromString(message), nullptr);
#endif
  return nullptr;
}

/**
 * Raises a TypeError of the form: function_name() argument n must be type,
 * not type for a given object passed to a function.
 *
 * Always returns NULL so that it can be conveniently used as a return
 * expression for wrapper functions that return a PyObject pointer.
 */
INLINE PyObject *
Dtool_Raise_ArgTypeError(PyObject *obj, int param, const char *function_name, const char *type_name) {
#if PY_MAJOR_VERSION >= 3
  PyObject *message = PyUnicode_FromFormat(
#else
  PyObject *message = PyString_FromFormat(
#endif
    "%s() argument %d must be %s, not %s",
    function_name, param, type_name,
    Py_TYPE(obj)->tp_name);

  Py_INCREF(PyExc_TypeError);
  PyErr_Restore(PyExc_TypeError, message, nullptr);
  return nullptr;
}

/**
 * Raises an AttributeError of the form: 'type' has no attribute 'attr'
 *
 * Always returns NULL so that it can be conveniently used as a return
 * expression for wrapper functions that return a PyObject pointer.
 */
INLINE PyObject *
Dtool_Raise_AttributeError(PyObject *obj, const char *attribute) {
#if PY_MAJOR_VERSION >= 3
  PyObject *message = PyUnicode_FromFormat(
#else
  PyObject *message = PyString_FromFormat(
#endif
    "'%.100s' object has no attribute '%.200s'",
    Py_TYPE(obj)->tp_name, attribute);

  Py_INCREF(PyExc_TypeError);
  PyErr_Restore(PyExc_TypeError, message, nullptr);
  return nullptr;
}

/**
 * Raises a TypeError of the form: Arguments must match: <list of overloads>
 *
 * However, in release builds, this instead is defined to a function that just
 * prints out a generic message, to help reduce the amount of strings in the
 * compiled library.
 *
 * Always returns NULL so that it can be conveniently used as a return
 * expression for wrapper functions that return a PyObject pointer.
 */
INLINE PyObject *_Dtool_Raise_BadArgumentsError() {
  return Dtool_Raise_TypeError("arguments do not match any function overload");
}

/**
 * Convenience method that checks for exceptions, and if one occurred, returns
 * NULL, otherwise Py_None.
 */
INLINE PyObject *_Dtool_Return_None() {
  if (UNLIKELY(_PyErr_OCCURRED())) {
    return nullptr;
  }
#ifndef NDEBUG
  if (UNLIKELY(Notify::ptr()->has_assert_failed())) {
    return Dtool_Raise_AssertionError();
  }
#endif
  Py_INCREF(Py_None);
  return Py_None;
}

/**
 * Convenience method that checks for exceptions, and if one occurred, returns
 * NULL, otherwise the given boolean value as a PyObject *.
 */
INLINE PyObject *Dtool_Return_Bool(bool value) {
  if (UNLIKELY(_PyErr_OCCURRED())) {
    return nullptr;
  }
#ifndef NDEBUG
  if (UNLIKELY(Notify::ptr()->has_assert_failed())) {
    return Dtool_Raise_AssertionError();
  }
#endif
  PyObject *result = (value ? Py_True : Py_False);
  Py_INCREF(result);
  return result;
}

/**
 * Convenience method that checks for exceptions, and if one occurred, returns
 * NULL, otherwise the given return value.  Its reference count is not
 * increased.
 */
INLINE PyObject *_Dtool_Return(PyObject *value) {
  if (UNLIKELY(_PyErr_OCCURRED())) {
    return nullptr;
  }
#ifndef NDEBUG
  if (UNLIKELY(Notify::ptr()->has_assert_failed())) {
    return Dtool_Raise_AssertionError();
  }
#endif
  return value;
}

#if PY_VERSION_HEX < 0x03040000
inline PyObject *Dtool_EnumType_Str(PyObject *self) {
  PyObject *name = PyObject_GetAttrString(self, "name");
#if PY_MAJOR_VERSION >= 3
  PyObject *repr = PyUnicode_FromFormat("%s.%s", Py_TYPE(self)->tp_name, PyString_AS_STRING(name));
#else
  PyObject *repr = PyString_FromFormat("%s.%s", Py_TYPE(self)->tp_name, PyString_AS_STRING(name));
#endif
  Py_DECREF(name);
  return repr;
}

inline PyObject *Dtool_EnumType_Repr(PyObject *self) {
  PyObject *name = PyObject_GetAttrString(self, "name");
  PyObject *value = PyObject_GetAttrString(self, "value");
#if PY_MAJOR_VERSION >= 3
  PyObject *repr = PyUnicode_FromFormat("<%s.%s: %ld>", Py_TYPE(self)->tp_name, PyString_AS_STRING(name), PyLongOrInt_AS_LONG(value));
#else
  PyObject *repr = PyString_FromFormat("<%s.%s: %ld>", Py_TYPE(self)->tp_name, PyString_AS_STRING(name), PyLongOrInt_AS_LONG(value));
#endif
  Py_DECREF(name);
  Py_DECREF(value);
  return repr;
}
#endif

/**
 * Creates a Python 3.4-style enum type.  Steals reference to 'names', which
 * should be a tuple of (name, value) pairs.
 */
INLINE PyTypeObject *
Dtool_EnumType_Create(const char *name, PyObject *names, const char *module) {
  static PyObject *enum_class = nullptr;
#if PY_VERSION_HEX >= 0x03040000
  static PyObject *enum_meta = nullptr;
  static PyObject *enum_create = nullptr;
  if (enum_meta == nullptr) {
    PyObject *enum_module = PyImport_ImportModule("enum");
    nassertr_always(enum_module != nullptr, nullptr);

    enum_class = PyObject_GetAttrString(enum_module, "Enum");
    enum_meta = PyObject_GetAttrString(enum_module, "EnumMeta");
    enum_create = PyObject_GetAttrString(enum_meta, "_create_");
    nassertr(enum_meta != nullptr, nullptr);
  }

  PyObject *result = PyObject_CallFunction(enum_create, (char *)"OsN", enum_class, name, names);
  nassertr(result != nullptr, nullptr);
#else
  static PyObject *name_str;
  static PyObject *name_sunder_str;
  static PyObject *value_str;
  static PyObject *value_sunder_str;
  // Emulate something vaguely like the enum module.
  if (enum_class == nullptr) {
#if PY_MAJOR_VERSION >= 3
    name_str = PyUnicode_InternFromString("name");
    value_str = PyUnicode_InternFromString("value");
    name_sunder_str = PyUnicode_InternFromString("_name_");
    value_sunder_str = PyUnicode_InternFromString("_value_");
#else
    name_str = PyString_InternFromString("name");
    value_str = PyString_InternFromString("value");
    name_sunder_str = PyString_InternFromString("_name_");
    value_sunder_str = PyString_InternFromString("_value_");
#endif
    PyObject *name_value_tuple = PyTuple_New(4);
    PyTuple_SET_ITEM(name_value_tuple, 0, name_str);
    PyTuple_SET_ITEM(name_value_tuple, 1, value_str);
    PyTuple_SET_ITEM(name_value_tuple, 2, name_sunder_str);
    PyTuple_SET_ITEM(name_value_tuple, 3, value_sunder_str);
    Py_INCREF(name_str);
    Py_INCREF(value_str);

    PyObject *slots_dict = PyDict_New();
    PyDict_SetItemString(slots_dict, "__slots__", name_value_tuple);
    Py_DECREF(name_value_tuple);

    enum_class = PyObject_CallFunction((PyObject *)&PyType_Type, (char *)"s()N", "Enum", slots_dict);
    nassertr(enum_class != nullptr, nullptr);
  }
  PyObject *result = PyObject_CallFunction((PyObject *)&PyType_Type, (char *)"s(O)N", name, enum_class, PyDict_New());
  nassertr(result != nullptr, nullptr);

  ((PyTypeObject *)result)->tp_str = Dtool_EnumType_Str;
  ((PyTypeObject *)result)->tp_repr = Dtool_EnumType_Repr;

  // Copy the names as instances of the above to the class dict.
  Py_ssize_t size = PyTuple_GET_SIZE(names);
  for (Py_ssize_t i = 0; i < size; ++i) {
    PyObject *item = PyTuple_GET_ITEM(names, i);
    PyObject *name = PyTuple_GET_ITEM(item, 0);
    PyObject *value = PyTuple_GET_ITEM(item, 1);
    PyObject *member = _PyObject_CallNoArg(result);
    PyObject_SetAttr(member, name_str, name);
    PyObject_SetAttr(member, name_sunder_str, name);
    PyObject_SetAttr(member, value_str, value);
    PyObject_SetAttr(member, value_sunder_str, value);
    PyObject_SetAttr(result, name, member);
    Py_DECREF(member);
  }
  Py_DECREF(names);
#endif

  if (module != nullptr) {
    PyObject *modstr = PyUnicode_FromString(module);
    PyObject_SetAttrString(result, "__module__", modstr);
    Py_DECREF(modstr);
  }
  nassertr(PyType_Check(result), nullptr);
  return (PyTypeObject *)result;
}

/**
 * Converts the enum value to a C long.
 */
INLINE long Dtool_EnumValue_AsLong(PyObject *value) {
  PyObject *val = PyObject_GetAttrString(value, "value");
  if (val != nullptr) {
    long as_long = PyLongOrInt_AS_LONG(val);
    Py_DECREF(val);
    return as_long;
  } else {
    return -1;
  }
}

/**
 *
 */
INLINE PyObject *
DTool_CreatePyInstanceTyped(void *local_this_in, Dtool_PyTypedObject &known_class_type, bool memory_rules, bool is_const, int type_index) {
  // We can't do the NULL check here like in DTool_CreatePyInstance, since the
  // caller will have to get the type index to pass to this function to begin
  // with.  That code probably would have crashed by now if it was really NULL
  // for whatever reason.
  nassertr(local_this_in != nullptr, nullptr);

  // IF the class is possibly a run time typed object
  if (type_index > 0) {
    // get best fit class...
    Dtool_PyTypedObject *target_class = (Dtool_PyTypedObject *)TypeHandle::from_index(type_index).get_python_type();
    if (target_class != nullptr) {
      // cast to the type...
      void *new_local_this = target_class->_Dtool_DowncastInterface(local_this_in, &known_class_type);
      if (new_local_this != nullptr) {
        // ask class to allocate an instance..
        Dtool_PyInstDef *self = (Dtool_PyInstDef *) target_class->_PyType.tp_new(&target_class->_PyType, nullptr, nullptr);
        if (self != nullptr) {
          self->_ptr_to_object = new_local_this;
          self->_memory_rules = memory_rules;
          self->_is_const = is_const;
          // self->_signature = PY_PANDA_SIGNATURE;
          self->_My_Type = target_class;
          return (PyObject *)self;
        }
      }
    }
  }

  // if we get this far .. just wrap the thing in the known type ?? better
  // than aborting...I guess....
  Dtool_PyInstDef *self = (Dtool_PyInstDef *) known_class_type._PyType.tp_new(&known_class_type._PyType, nullptr, nullptr);
  if (self != nullptr) {
    self->_ptr_to_object = local_this_in;
    self->_memory_rules = memory_rules;
    self->_is_const = is_const;
    // self->_signature = PY_PANDA_SIGNATURE;
    self->_My_Type = &known_class_type;
  }
  return (PyObject *)self;
}

// DTool_CreatePyInstance .. wrapper function to finalize the existance of a
// general dtool py instance..
INLINE PyObject *
DTool_CreatePyInstance(void *local_this, Dtool_PyTypedObject &in_classdef, bool memory_rules, bool is_const) {
  if (local_this == nullptr) {
    // This is actually a very common case, so let's allow this, but return
    // Py_None consistently.  This eliminates code in the wrappers.
    Py_INCREF(Py_None);
    return Py_None;
  }

  Dtool_PyTypedObject *classdef = &in_classdef;
  Dtool_PyInstDef *self = (Dtool_PyInstDef *) classdef->_PyType.tp_new(&classdef->_PyType, nullptr, nullptr);
  if (self != nullptr) {
    self->_ptr_to_object = local_this;
    self->_memory_rules = memory_rules;
    self->_is_const = is_const;
    self->_My_Type = classdef;
  }
  return (PyObject *)self;
}

/**
 * These functions wrap a pointer for a class that defines get_type_handle().
 */
template<class T> INLINE PyObject *
DTool_CreatePyInstance(const T *obj, bool memory_rules) {
  Dtool_PyTypedObject *known_class = (Dtool_PyTypedObject *)get_type_handle(T).get_python_type();
  nassertr(known_class != nullptr, nullptr);
  return DTool_CreatePyInstance((void*) obj, *known_class, memory_rules, true);
}

template<class T> INLINE PyObject *
DTool_CreatePyInstance(T *obj, bool memory_rules) {
  Dtool_PyTypedObject *known_class = (Dtool_PyTypedObject *)get_type_handle(T).get_python_type();
  nassertr(known_class != nullptr, nullptr);
  return DTool_CreatePyInstance((void*) obj, *known_class, memory_rules, false);
}

template<class T> INLINE PyObject *
DTool_CreatePyInstanceTyped(const T *obj, bool memory_rules) {
  Dtool_PyTypedObject *known_class = (Dtool_PyTypedObject *)get_type_handle(T).get_python_type();
  nassertr(known_class != nullptr, nullptr);
  return DTool_CreatePyInstanceTyped((void*) obj, *known_class, memory_rules, true, obj->get_type().get_index());
}

template<class T> INLINE PyObject *
DTool_CreatePyInstanceTyped(T *obj, bool memory_rules) {
  Dtool_PyTypedObject *known_class = (Dtool_PyTypedObject *)get_type_handle(T).get_python_type();
  nassertr(known_class != nullptr, nullptr);
  return DTool_CreatePyInstanceTyped((void*) obj, *known_class, memory_rules, false, obj->get_type().get_index());
}

// Th Finalizer for simple instances..
INLINE int
DTool_PyInit_Finalize(PyObject *self, void *local_this, Dtool_PyTypedObject *type, bool memory_rules, bool is_const) {
  // lets put some code in here that checks to see the memory is properly
  // configured.. prior to my call ..

  ((Dtool_PyInstDef *)self)->_My_Type = type;
  ((Dtool_PyInstDef *)self)->_ptr_to_object = local_this;
  ((Dtool_PyInstDef *)self)->_memory_rules = memory_rules;
  ((Dtool_PyInstDef *)self)->_is_const = is_const;
  return 0;
}

#if PY_MAJOR_VERSION >= 3
INLINE PyObject *Dtool_PyModuleInitHelper(LibraryDef *defs[], PyModuleDef *module_def) {
#else
INLINE PyObject *Dtool_PyModuleInitHelper(LibraryDef *defs[], const char *modulename) {
#endif
  // Check the version so we can print a helpful error if it doesn't match.
  string version = Py_GetVersion();

  if (version[0] != '0' + PY_MAJOR_VERSION ||
      version[2] != '0' + PY_MINOR_VERSION) {
    // Raise a helpful error message.  We can safely do this because the
    // signature and behavior for PyErr_SetString has remained consistent.
    std::ostringstream errs;
    errs << "this module was compiled for Python "
         << PY_MAJOR_VERSION << "." << PY_MINOR_VERSION << ", which is "
         << "incompatible with Python " << version.substr(0, 3);
    std::string error = errs.str();
    PyErr_SetString(PyExc_ImportError, error.c_str());
    return nullptr;
  }

  // the module level function inits....
  std::map<std::string, PyMethodDef *> functions;
  for (int xx = 0; defs[xx] != nullptr; xx++) {
    PyMethodDef *in = defs[xx]->_methods;

    for (; in->ml_name != nullptr; in++) {
      if (functions.find(in->ml_name) == functions.end()) {
        functions[in->ml_name] = in;
      }
    }
  }

  PyMethodDef *newdef = new PyMethodDef[functions.size() + 1];
  std::map<std::string, PyMethodDef *>::iterator mi;
  int offset = 0;
  for (mi = functions.begin(); mi != functions.end(); mi++, offset++) {
    newdef[offset] = *mi->second;
  }
  newdef[offset].ml_doc = nullptr;
  newdef[offset].ml_name = nullptr;
  newdef[offset].ml_meth = nullptr;
  newdef[offset].ml_flags = 0;

#if PY_MAJOR_VERSION >= 3
  module_def->m_methods = newdef;
  PyObject *module = PyModule_Create(module_def);
#else
  PyObject *module = Py_InitModule((char *)modulename, newdef);
#endif

  if (module == nullptr) {
#if PY_MAJOR_VERSION >= 3
    return Dtool_Raise_TypeError("PyModule_Create returned NULL");
#else
    return Dtool_Raise_TypeError("Py_InitModule returned NULL");
#endif
  }

  // MAIN_DIR needs to be set very early; this seems like a convenient place
  // to do that.  Perhaps we'll find a better place for this in the future.
  static bool initialized_main_dir = false;
  if (!initialized_main_dir) {
    // Grab the __main__ module.
    PyObject *main_module = PyImport_ImportModule("__main__");
    if (main_module == nullptr) {
      std::cerr << "Unable to import __main__\n";
    }

    // Extract the __file__ attribute, if present.
    Filename main_dir;
    PyObject *file_attr = PyObject_GetAttrString(main_module, "__file__");
    if (file_attr == nullptr) {
      // Must be running in the interactive interpreter.  Use the CWD.
      main_dir = ExecutionEnvironment::get_cwd();
    } else {
#if PY_MAJOR_VERSION >= 3
      Py_ssize_t length;
      wchar_t *buffer = PyUnicode_AsWideCharString(file_attr, &length);
      if (buffer != nullptr) {
        main_dir = Filename::from_os_specific_w(std::wstring(buffer, length));
        main_dir.make_absolute();
        main_dir = main_dir.get_dirname();
        PyMem_Free(buffer);
      }
#else
      char *buffer;
      Py_ssize_t length;
      if (PyString_AsStringAndSize(file_attr, &buffer, &length) != -1) {
        main_dir = Filename::from_os_specific(std::string(buffer, length));
        main_dir.make_absolute();
        main_dir = main_dir.get_dirname();
      }
#endif
      else {
        std::cerr << "Invalid string for __main__.__file__\n";
      }
    }
    ExecutionEnvironment::shadow_environment_variable("MAIN_DIR", main_dir.to_os_specific());
    PyErr_Clear();
    initialized_main_dir = true;
  }

  PyModule_AddIntConstant(module, "Dtool_PyNativeInterface", 1);
  return module;
}

// HACK.... Be careful Dtool_BorrowThisReference This function can be used to
// grab the "THIS" pointer from an object and use it Required to support
// historical inheritance in the form of "is this instance of"..
INLINE PyObject *Dtool_BorrowThisReference(PyObject *self, PyObject *args) {
  PyObject *from_in = nullptr;
  PyObject *to_in = nullptr;
  if (PyArg_UnpackTuple(args, "Dtool_BorrowThisReference", 2, 2, &to_in, &from_in)) {

    if (DtoolInstance_Check(from_in) && DtoolInstance_Check(to_in)) {
      Dtool_PyInstDef *from = (Dtool_PyInstDef *) from_in;
      Dtool_PyInstDef *to = (Dtool_PyInstDef *) to_in;

      // if (PyObject_TypeCheck(to_in, Py_TYPE(from_in))) {
      if (from->_My_Type == to->_My_Type) {
        to->_memory_rules = false;
        to->_is_const = from->_is_const;
        to->_ptr_to_object = from->_ptr_to_object;

        Py_INCREF(Py_None);
        return Py_None;
      }

      return PyErr_Format(PyExc_TypeError, "types %s and %s do not match",
                          Py_TYPE(from)->tp_name, Py_TYPE(to)->tp_name);
    } else {
      return Dtool_Raise_TypeError("One of these does not appear to be DTOOL Instance ??");
    }
  }
  return nullptr;
}

/**
 * This is a support function for a synthesized __copy__() method from a C++
 * make_copy() method.
 */
INLINE PyObject *copy_from_make_copy(PyObject *self, PyObject *noargs) {
  PyObject *callable = PyObject_GetAttrString(self, "make_copy");
  if (callable == nullptr) {
    return nullptr;
  }
  PyObject *result = _PyObject_CallNoArg(callable);
  Py_DECREF(callable);
  return result;
}

/**
 * This is a support function for a synthesized __copy__() method from a C++
 * copy constructor.
 */
INLINE PyObject *copy_from_copy_constructor(PyObject *self, PyObject *noargs) {
  PyObject *callable = (PyObject *)Py_TYPE(self);
  return _PyObject_FastCall(callable, &self, 1);
}

/**
 * This is a support function for a synthesized __deepcopy__() method for any
 * class that has a __copy__() method.  The sythethic method simply invokes
 * __copy__().
 */
INLINE PyObject *map_deepcopy_to_copy(PyObject *self, PyObject *args) {
  PyObject *callable = PyObject_GetAttrString(self, "__copy__");
  if (callable == nullptr) {
    return nullptr;
  }
  PyObject *result = _PyObject_CallNoArg(callable);
  Py_DECREF(callable);
  return result;
}

/**
 * Checks that the tuple is empty.
 */
ALWAYS_INLINE bool
Dtool_CheckNoArgs(PyObject *args) {
  return PyTuple_GET_SIZE(args) == 0;
}

/**
 * Checks that the tuple is empty, and that the dict is empty or NULL.
 */
ALWAYS_INLINE bool
Dtool_CheckNoArgs(PyObject *args, PyObject *kwds) {
  return PyTuple_GET_SIZE(args) == 0 &&
    (kwds == nullptr || PyDict_GET_SIZE(kwds) == 0);
}

/**
 * A more efficient version of PyArg_ParseTupleAndKeywords for the special
 * case where there is only a single PyObject argument.
 */
INLINE bool
Dtool_ExtractArg(PyObject **result, PyObject *args, PyObject *kwds,
                 const char *keyword) {

  if (PyTuple_GET_SIZE(args) == 1) {
    if (kwds == nullptr || PyDict_GET_SIZE(kwds) == 0) {
      *result = PyTuple_GET_ITEM(args, 0);
      return true;
    }
  } else if (PyTuple_GET_SIZE(args) == 0) {
    PyObject *key;
    Py_ssize_t ppos = 0;
    if (kwds != nullptr && PyDict_GET_SIZE(kwds) == 1 &&
        PyDict_Next(kwds, &ppos, &key, result)) {
      // We got the item, we just need to make sure that it had the right key.
#if PY_VERSION_HEX >= 0x03060000
      return PyUnicode_CheckExact(key) && _PyUnicode_EqualToASCIIString(key, keyword);
#elif PY_MAJOR_VERSION >= 3
      return PyUnicode_CheckExact(key) && PyUnicode_CompareWithASCIIString(key, keyword) == 0;
#else
      return PyString_CheckExact(key) && strcmp(PyString_AS_STRING(key), keyword) == 0;
#endif
    }
  }

  return false;
}

/**
 * Variant of Dtool_ExtractArg that does not accept a keyword argument.
 */
INLINE bool
Dtool_ExtractArg(PyObject **result, PyObject *args, PyObject *kwds) {
  if (PyTuple_GET_SIZE(args) == 1 &&
      (kwds == nullptr || PyDict_GET_SIZE(kwds) == 0)) {
    *result = PyTuple_GET_ITEM(args, 0);
    return true;
  }
  return false;
}

/**
 * A more efficient version of PyArg_ParseTupleAndKeywords for the special
 * case where there is only a single optional PyObject argument.
 *
 * Returns true if valid (including if there were 0 items), false if there was
 * an error, such as an invalid number of parameters.
 */
INLINE bool
Dtool_ExtractOptionalArg(PyObject **result, PyObject *args, PyObject *kwds,
                         const char *keyword) {

  if (PyTuple_GET_SIZE(args) == 1) {
    if (kwds == nullptr || PyDict_GET_SIZE(kwds) == 0) {
      *result = PyTuple_GET_ITEM(args, 0);
      return true;
    }
  } else if (PyTuple_GET_SIZE(args) == 0) {
    if (kwds != nullptr && PyDict_GET_SIZE(kwds) == 1) {
      PyObject *key;
      Py_ssize_t ppos = 0;
      if (!PyDict_Next(kwds, &ppos, &key, result)) {
        return true;
      }

      // We got the item, we just need to make sure that it had the right key.
#if PY_VERSION_HEX >= 0x03060000
      return PyUnicode_CheckExact(key) && _PyUnicode_EqualToASCIIString(key, keyword);
#elif PY_MAJOR_VERSION >= 3
      return PyUnicode_CheckExact(key) && PyUnicode_CompareWithASCIIString(key, keyword) == 0;
#else
      return PyString_CheckExact(key) && strcmp(PyString_AS_STRING(key), keyword) == 0;
#endif
    } else {
      return true;
    }
  }

  return false;
}

/**
 * Variant of Dtool_ExtractOptionalArg that does not accept a keyword argument.
 */
INLINE bool
Dtool_ExtractOptionalArg(PyObject **result, PyObject *args, PyObject *kwds) {
  if (kwds != nullptr && PyDict_GET_SIZE(kwds) != 0) {
    return false;
  }
  if (PyTuple_GET_SIZE(args) == 1) {
    *result = PyTuple_GET_ITEM(args, 0);
    return true;
  }
  return (PyTuple_GET_SIZE(args) == 0);
}

/**
 * The following functions wrap an arbitrary C++ value into a PyObject.
 */
ALWAYS_INLINE PyObject *Dtool_WrapValue(int value) {
#if PY_MAJOR_VERSION >= 3
  return PyLong_FromLong((long)value);
#else
  return PyInt_FromLong((long)value);
#endif
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(unsigned int value) {
#if PY_MAJOR_VERSION >= 3 && SIZEOF_INT < SIZEOF_LONG
  return PyLong_FromLong((long)value);
#elif PY_MAJOR_VERSION >= 3
  return PyLong_FromUnsignedLong((unsigned long)value);
#elif SIZEOF_INT < SIZEOF_LONG
  return PyInt_FromLong((long)value);
#else
  return (value > LONG_MAX)
    ? PyLong_FromUnsignedLong((unsigned long)value)
    : PyInt_FromLong((long)value);
#endif
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(long value) {
#if PY_MAJOR_VERSION >= 3
  return PyLong_FromLong(value);
#else
  return PyInt_FromLong(value);
#endif
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(unsigned long value) {
#if PY_MAJOR_VERSION >= 3
  return PyLong_FromUnsignedLong(value);
#else
  return (value > LONG_MAX)
    ? PyLong_FromUnsignedLong(value)
    : PyInt_FromLong((long)value);
#endif
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(long long value) {
  return PyLong_FromLongLong(value);
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(unsigned long long value) {
  // size_t is sometimes defined as unsigned long long, and we want to map
  // that to int in Python 2 so it can be returned from a __len__.
#if PY_MAJOR_VERSION >= 3
  return PyLong_FromUnsignedLongLong(value);
#else
  return (value > LONG_MAX)
    ? PyLong_FromUnsignedLongLong(value)
    : PyInt_FromLong((long)value);
#endif
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(bool value) {
  PyObject *result = (value ? Py_True : Py_False);
  Py_INCREF(result);
  return result;
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(double value) {
  return PyFloat_FromDouble(value);
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(const char *value) {
  if (value == nullptr) {
    Py_INCREF(Py_None);
    return Py_None;
  } else {
#if PY_MAJOR_VERSION >= 3
    return PyUnicode_FromString(value);
#else
    return PyString_FromString(value);
#endif
  }
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(const wchar_t *value) {
  if (value == nullptr) {
    Py_INCREF(Py_None);
    return Py_None;
  } else {
    return PyUnicode_FromWideChar(value, (Py_ssize_t)wcslen(value));
  }
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(const std::string &value) {
#if PY_MAJOR_VERSION >= 3
  return PyUnicode_FromStringAndSize(value.data(), (Py_ssize_t)value.length());
#else
  return PyString_FromStringAndSize(value.data(), (Py_ssize_t)value.length());
#endif
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(const std::wstring &value) {
  return PyUnicode_FromWideChar(value.data(), (Py_ssize_t)value.length());
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(const std::string *value) {
  if (value == nullptr) {
    Py_INCREF(Py_None);
    return Py_None;
  } else {
#if PY_MAJOR_VERSION >= 3
    return PyUnicode_FromStringAndSize(value->data(), (Py_ssize_t)value->length());
#else
    return PyString_FromStringAndSize(value->data(), (Py_ssize_t)value->length());
#endif
  }
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(const std::wstring *value) {
  if (value == nullptr) {
    Py_INCREF(Py_None);
    return Py_None;
  } else {
    return PyUnicode_FromWideChar(value->data(), (Py_ssize_t)value->length());
  }
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(char value) {
#if PY_MAJOR_VERSION >= 3
  return PyUnicode_FromStringAndSize(&value, 1);
#else
  return PyString_FromStringAndSize(&value, 1);
#endif
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(wchar_t value) {
  return PyUnicode_FromWideChar(&value, 1);
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(std::nullptr_t) {
  Py_INCREF(Py_None);
  return Py_None;
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(PyObject *value) {
  return value;
}

ALWAYS_INLINE PyObject *Dtool_WrapValue(const vector_uchar &value) {
#if PY_MAJOR_VERSION >= 3
  return PyBytes_FromStringAndSize((char *)value.data(), (Py_ssize_t)value.size());
#else
  return PyString_FromStringAndSize((char *)value.data(), (Py_ssize_t)value.size());
#endif
}

#if PY_MAJOR_VERSION >= 0x02060000
ALWAYS_INLINE PyObject *Dtool_WrapValue(Py_buffer *value) {
  if (value == nullptr) {
    return value;
  } else {
    return PyMemoryView_FromBuffer(value);
  }
}
#endif

template<class T1, class T2>
ALWAYS_INLINE PyObject *Dtool_WrapValue(const std::pair<T1, T2> &value) {
  PyObject *tuple = PyTuple_New(2);
  PyTuple_SET_ITEM(tuple, 0, Dtool_WrapValue(value.first));
  PyTuple_SET_ITEM(tuple, 1, Dtool_WrapValue(value.second));
  return tuple;
}
