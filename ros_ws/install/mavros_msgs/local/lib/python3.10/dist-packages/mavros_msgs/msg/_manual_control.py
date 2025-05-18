# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/ManualControl.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ManualControl(type):
    """Metaclass of message 'ManualControl'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('mavros_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'mavros_msgs.msg.ManualControl')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__manual_control
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__manual_control
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__manual_control
            cls._TYPE_SUPPORT = module.type_support_msg__msg__manual_control
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__manual_control

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ManualControl(metaclass=Metaclass_ManualControl):
    """Message class 'ManualControl'."""

    __slots__ = [
        '_header',
        '_x',
        '_y',
        '_z',
        '_r',
        '_buttons',
        '_buttons2',
        '_enabled_extensions',
        '_s',
        '_t',
        '_aux1',
        '_aux2',
        '_aux3',
        '_aux4',
        '_aux5',
        '_aux6',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'x': 'float',
        'y': 'float',
        'z': 'float',
        'r': 'float',
        'buttons': 'uint16',
        'buttons2': 'uint16',
        'enabled_extensions': 'uint8',
        's': 'float',
        't': 'float',
        'aux1': 'float',
        'aux2': 'float',
        'aux3': 'float',
        'aux4': 'float',
        'aux5': 'float',
        'aux6': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.z = kwargs.get('z', float())
        self.r = kwargs.get('r', float())
        self.buttons = kwargs.get('buttons', int())
        self.buttons2 = kwargs.get('buttons2', int())
        self.enabled_extensions = kwargs.get('enabled_extensions', int())
        self.s = kwargs.get('s', float())
        self.t = kwargs.get('t', float())
        self.aux1 = kwargs.get('aux1', float())
        self.aux2 = kwargs.get('aux2', float())
        self.aux3 = kwargs.get('aux3', float())
        self.aux4 = kwargs.get('aux4', float())
        self.aux5 = kwargs.get('aux5', float())
        self.aux6 = kwargs.get('aux6', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        if self.r != other.r:
            return False
        if self.buttons != other.buttons:
            return False
        if self.buttons2 != other.buttons2:
            return False
        if self.enabled_extensions != other.enabled_extensions:
            return False
        if self.s != other.s:
            return False
        if self.t != other.t:
            return False
        if self.aux1 != other.aux1:
            return False
        if self.aux2 != other.aux2:
            return False
        if self.aux3 != other.aux3:
            return False
        if self.aux4 != other.aux4:
            return False
        if self.aux5 != other.aux5:
            return False
        if self.aux6 != other.aux6:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x = value

    @builtins.property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y = value

    @builtins.property
    def z(self):
        """Message field 'z'."""
        return self._z

    @z.setter
    def z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'z' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z = value

    @builtins.property
    def r(self):
        """Message field 'r'."""
        return self._r

    @r.setter
    def r(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'r' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'r' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._r = value

    @builtins.property
    def buttons(self):
        """Message field 'buttons'."""
        return self._buttons

    @buttons.setter
    def buttons(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'buttons' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'buttons' field must be an unsigned integer in [0, 65535]"
        self._buttons = value

    @builtins.property
    def buttons2(self):
        """Message field 'buttons2'."""
        return self._buttons2

    @buttons2.setter
    def buttons2(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'buttons2' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'buttons2' field must be an unsigned integer in [0, 65535]"
        self._buttons2 = value

    @builtins.property
    def enabled_extensions(self):
        """Message field 'enabled_extensions'."""
        return self._enabled_extensions

    @enabled_extensions.setter
    def enabled_extensions(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'enabled_extensions' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'enabled_extensions' field must be an unsigned integer in [0, 255]"
        self._enabled_extensions = value

    @builtins.property
    def s(self):
        """Message field 's'."""
        return self._s

    @s.setter
    def s(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 's' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 's' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._s = value

    @builtins.property
    def t(self):
        """Message field 't'."""
        return self._t

    @t.setter
    def t(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 't' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 't' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._t = value

    @builtins.property
    def aux1(self):
        """Message field 'aux1'."""
        return self._aux1

    @aux1.setter
    def aux1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aux1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'aux1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._aux1 = value

    @builtins.property
    def aux2(self):
        """Message field 'aux2'."""
        return self._aux2

    @aux2.setter
    def aux2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aux2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'aux2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._aux2 = value

    @builtins.property
    def aux3(self):
        """Message field 'aux3'."""
        return self._aux3

    @aux3.setter
    def aux3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aux3' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'aux3' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._aux3 = value

    @builtins.property
    def aux4(self):
        """Message field 'aux4'."""
        return self._aux4

    @aux4.setter
    def aux4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aux4' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'aux4' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._aux4 = value

    @builtins.property
    def aux5(self):
        """Message field 'aux5'."""
        return self._aux5

    @aux5.setter
    def aux5(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aux5' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'aux5' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._aux5 = value

    @builtins.property
    def aux6(self):
        """Message field 'aux6'."""
        return self._aux6

    @aux6.setter
    def aux6(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'aux6' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'aux6' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._aux6 = value
