// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from mavros_msgs:msg/OpenDroneIDSystem.idl
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
#include "mavros_msgs/msg/detail/open_drone_id_system__struct.h"
#include "mavros_msgs/msg/detail/open_drone_id_system__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool mavros_msgs__msg__open_drone_id_system__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[56];
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
    assert(strncmp("mavros_msgs.msg._open_drone_id_system.OpenDroneIDSystem", full_classname_dest, 55) == 0);
  }
  mavros_msgs__msg__OpenDroneIDSystem * ros_message = _ros_message;
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
  {  // id_or_mac
    PyObject * field = PyObject_GetAttrString(_pymsg, "id_or_mac");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->id_or_mac, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // operator_location_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "operator_location_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->operator_location_type = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // classification_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "classification_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->classification_type = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // operator_latitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "operator_latitude");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->operator_latitude = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // operator_longitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "operator_longitude");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->operator_longitude = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // area_count
    PyObject * field = PyObject_GetAttrString(_pymsg, "area_count");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->area_count = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // area_radius
    PyObject * field = PyObject_GetAttrString(_pymsg, "area_radius");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->area_radius = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // area_ceiling
    PyObject * field = PyObject_GetAttrString(_pymsg, "area_ceiling");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->area_ceiling = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // area_floor
    PyObject * field = PyObject_GetAttrString(_pymsg, "area_floor");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->area_floor = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // category_eu
    PyObject * field = PyObject_GetAttrString(_pymsg, "category_eu");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->category_eu = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // class_eu
    PyObject * field = PyObject_GetAttrString(_pymsg, "class_eu");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->class_eu = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // operator_altitude_geo
    PyObject * field = PyObject_GetAttrString(_pymsg, "operator_altitude_geo");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->operator_altitude_geo = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * mavros_msgs__msg__open_drone_id_system__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of OpenDroneIDSystem */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("mavros_msgs.msg._open_drone_id_system");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "OpenDroneIDSystem");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  mavros_msgs__msg__OpenDroneIDSystem * ros_message = (mavros_msgs__msg__OpenDroneIDSystem *)raw_ros_message;
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
  {  // id_or_mac
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->id_or_mac.data,
      strlen(ros_message->id_or_mac.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "id_or_mac", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // operator_location_type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->operator_location_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "operator_location_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // classification_type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->classification_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "classification_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // operator_latitude
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->operator_latitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "operator_latitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // operator_longitude
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->operator_longitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "operator_longitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // area_count
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->area_count);
    {
      int rc = PyObject_SetAttrString(_pymessage, "area_count", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // area_radius
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->area_radius);
    {
      int rc = PyObject_SetAttrString(_pymessage, "area_radius", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // area_ceiling
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->area_ceiling);
    {
      int rc = PyObject_SetAttrString(_pymessage, "area_ceiling", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // area_floor
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->area_floor);
    {
      int rc = PyObject_SetAttrString(_pymessage, "area_floor", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // category_eu
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->category_eu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "category_eu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // class_eu
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->class_eu);
    {
      int rc = PyObject_SetAttrString(_pymessage, "class_eu", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // operator_altitude_geo
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->operator_altitude_geo);
    {
      int rc = PyObject_SetAttrString(_pymessage, "operator_altitude_geo", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
