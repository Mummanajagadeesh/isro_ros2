// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mavros_msgs:msg/MagnetometerReporter.idl
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
#include "mavros_msgs/msg/detail/magnetometer_reporter__struct.h"
#include "mavros_msgs/msg/detail/magnetometer_reporter__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__msg__magnetometer_reporter__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[60];
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
    assert(strncmp("mavros_msgs.msg._magnetometer_reporter.MagnetometerReporter", full_classname_dest, 59) == 0);
  }
  mavros_msgs__msg__MagnetometerReporter * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // report
    PyObject * field = PyObject_GetAttrString(_pymsg, "report");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->report = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // confidence
    PyObject * field = PyObject_GetAttrString(_pymsg, "confidence");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->confidence = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // compass_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "compass_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->compass_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // cal_mask
    PyObject * field = PyObject_GetAttrString(_pymsg, "cal_mask");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->cal_mask = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // cal_status
    PyObject * field = PyObject_GetAttrString(_pymsg, "cal_status");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->cal_status = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // autosaved
    PyObject * field = PyObject_GetAttrString(_pymsg, "autosaved");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->autosaved = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // fitness
    PyObject * field = PyObject_GetAttrString(_pymsg, "fitness");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fitness = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ofs_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "ofs_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ofs_x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ofs_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "ofs_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ofs_y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ofs_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "ofs_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ofs_z = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // diag_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "diag_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->diag_x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // diag_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "diag_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->diag_y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // diag_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "diag_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->diag_z = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // offdiag_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "offdiag_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->offdiag_x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // offdiag_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "offdiag_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->offdiag_y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // offdiag_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "offdiag_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->offdiag_z = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // orientation_confidence
    PyObject * field = PyObject_GetAttrString(_pymsg, "orientation_confidence");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->orientation_confidence = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // old_orientation
    PyObject * field = PyObject_GetAttrString(_pymsg, "old_orientation");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->old_orientation = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // new_orientation
    PyObject * field = PyObject_GetAttrString(_pymsg, "new_orientation");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->new_orientation = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // scale_factor
    PyObject * field = PyObject_GetAttrString(_pymsg, "scale_factor");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->scale_factor = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__msg__magnetometer_reporter__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MagnetometerReporter */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.msg._magnetometer_reporter");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MagnetometerReporter");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mavros_msgs__msg__MagnetometerReporter * ros_message = (mavros_msgs__msg__MagnetometerReporter *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // report
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->report);
    {
      int rc = PyObject_SetAttrString(_pymessage, "report", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // confidence
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->confidence);
    {
      int rc = PyObject_SetAttrString(_pymessage, "confidence", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // compass_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->compass_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "compass_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cal_mask
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->cal_mask);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cal_mask", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cal_status
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->cal_status);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cal_status", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // autosaved
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->autosaved);
    {
      int rc = PyObject_SetAttrString(_pymessage, "autosaved", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fitness
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fitness);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fitness", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ofs_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ofs_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ofs_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ofs_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ofs_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ofs_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ofs_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ofs_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ofs_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // diag_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->diag_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "diag_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // diag_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->diag_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "diag_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // diag_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->diag_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "diag_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // offdiag_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->offdiag_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "offdiag_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // offdiag_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->offdiag_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "offdiag_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // offdiag_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->offdiag_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "offdiag_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // orientation_confidence
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->orientation_confidence);
    {
      int rc = PyObject_SetAttrString(_pymessage, "orientation_confidence", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // old_orientation
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->old_orientation);
    {
      int rc = PyObject_SetAttrString(_pymessage, "old_orientation", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // new_orientation
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->new_orientation);
    {
      int rc = PyObject_SetAttrString(_pymessage, "new_orientation", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // scale_factor
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->scale_factor);
    {
      int rc = PyObject_SetAttrString(_pymessage, "scale_factor", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
