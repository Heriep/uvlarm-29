// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from kobuki_ros_interfaces:msg/CliffEvent.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "kobuki_ros_interfaces/msg/detail/cliff_event__struct.h"
#include "kobuki_ros_interfaces/msg/detail/cliff_event__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool kobuki_ros_interfaces__msg__cliff_event__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[50];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("kobuki_ros_interfaces.msg._cliff_event.CliffEvent", full_classname_dest, 49) == 0);
  }
  kobuki_ros_interfaces__msg__CliffEvent * ros_message = _ros_message;
  {  // sensor
    PyObject * field = PyObject_GetAttrString(_pymsg, "sensor");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sensor = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // state
    PyObject * field = PyObject_GetAttrString(_pymsg, "state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // bottom
    PyObject * field = PyObject_GetAttrString(_pymsg, "bottom");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->bottom = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * kobuki_ros_interfaces__msg__cliff_event__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CliffEvent */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("kobuki_ros_interfaces.msg._cliff_event");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CliffEvent");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  kobuki_ros_interfaces__msg__CliffEvent * ros_message = (kobuki_ros_interfaces__msg__CliffEvent *)raw_ros_message;
  {  // sensor
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sensor);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sensor", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bottom
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->bottom);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bottom", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}