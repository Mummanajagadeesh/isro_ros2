# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/MagnetometerReporter.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MagnetometerReporter(type):
    """Metaclass of message 'MagnetometerReporter'."""

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
                'mavros_msgs.msg.MagnetometerReporter')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__magnetometer_reporter
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__magnetometer_reporter
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__magnetometer_reporter
            cls._TYPE_SUPPORT = module.type_support_msg__msg__magnetometer_reporter
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__magnetometer_reporter

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


class MagnetometerReporter(metaclass=Metaclass_MagnetometerReporter):
    """Message class 'MagnetometerReporter'."""

    __slots__ = [
        '_header',
        '_report',
        '_confidence',
        '_compass_id',
        '_cal_mask',
        '_cal_status',
        '_autosaved',
        '_fitness',
        '_ofs_x',
        '_ofs_y',
        '_ofs_z',
        '_diag_x',
        '_diag_y',
        '_diag_z',
        '_offdiag_x',
        '_offdiag_y',
        '_offdiag_z',
        '_orientation_confidence',
        '_old_orientation',
        '_new_orientation',
        '_scale_factor',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'report': 'uint8',
        'confidence': 'float',
        'compass_id': 'uint8',
        'cal_mask': 'uint8',
        'cal_status': 'uint8',
        'autosaved': 'uint8',
        'fitness': 'float',
        'ofs_x': 'float',
        'ofs_y': 'float',
        'ofs_z': 'float',
        'diag_x': 'float',
        'diag_y': 'float',
        'diag_z': 'float',
        'offdiag_x': 'float',
        'offdiag_y': 'float',
        'offdiag_z': 'float',
        'orientation_confidence': 'float',
        'old_orientation': 'uint8',
        'new_orientation': 'uint8',
        'scale_factor': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.report = kwargs.get('report', int())
        self.confidence = kwargs.get('confidence', float())
        self.compass_id = kwargs.get('compass_id', int())
        self.cal_mask = kwargs.get('cal_mask', int())
        self.cal_status = kwargs.get('cal_status', int())
        self.autosaved = kwargs.get('autosaved', int())
        self.fitness = kwargs.get('fitness', float())
        self.ofs_x = kwargs.get('ofs_x', float())
        self.ofs_y = kwargs.get('ofs_y', float())
        self.ofs_z = kwargs.get('ofs_z', float())
        self.diag_x = kwargs.get('diag_x', float())
        self.diag_y = kwargs.get('diag_y', float())
        self.diag_z = kwargs.get('diag_z', float())
        self.offdiag_x = kwargs.get('offdiag_x', float())
        self.offdiag_y = kwargs.get('offdiag_y', float())
        self.offdiag_z = kwargs.get('offdiag_z', float())
        self.orientation_confidence = kwargs.get('orientation_confidence', float())
        self.old_orientation = kwargs.get('old_orientation', int())
        self.new_orientation = kwargs.get('new_orientation', int())
        self.scale_factor = kwargs.get('scale_factor', float())

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
        if self.report != other.report:
            return False
        if self.confidence != other.confidence:
            return False
        if self.compass_id != other.compass_id:
            return False
        if self.cal_mask != other.cal_mask:
            return False
        if self.cal_status != other.cal_status:
            return False
        if self.autosaved != other.autosaved:
            return False
        if self.fitness != other.fitness:
            return False
        if self.ofs_x != other.ofs_x:
            return False
        if self.ofs_y != other.ofs_y:
            return False
        if self.ofs_z != other.ofs_z:
            return False
        if self.diag_x != other.diag_x:
            return False
        if self.diag_y != other.diag_y:
            return False
        if self.diag_z != other.diag_z:
            return False
        if self.offdiag_x != other.offdiag_x:
            return False
        if self.offdiag_y != other.offdiag_y:
            return False
        if self.offdiag_z != other.offdiag_z:
            return False
        if self.orientation_confidence != other.orientation_confidence:
            return False
        if self.old_orientation != other.old_orientation:
            return False
        if self.new_orientation != other.new_orientation:
            return False
        if self.scale_factor != other.scale_factor:
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
    def report(self):
        """Message field 'report'."""
        return self._report

    @report.setter
    def report(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'report' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'report' field must be an unsigned integer in [0, 255]"
        self._report = value

    @builtins.property
    def confidence(self):
        """Message field 'confidence'."""
        return self._confidence

    @confidence.setter
    def confidence(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'confidence' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'confidence' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._confidence = value

    @builtins.property
    def compass_id(self):
        """Message field 'compass_id'."""
        return self._compass_id

    @compass_id.setter
    def compass_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'compass_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'compass_id' field must be an unsigned integer in [0, 255]"
        self._compass_id = value

    @builtins.property
    def cal_mask(self):
        """Message field 'cal_mask'."""
        return self._cal_mask

    @cal_mask.setter
    def cal_mask(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cal_mask' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'cal_mask' field must be an unsigned integer in [0, 255]"
        self._cal_mask = value

    @builtins.property
    def cal_status(self):
        """Message field 'cal_status'."""
        return self._cal_status

    @cal_status.setter
    def cal_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cal_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'cal_status' field must be an unsigned integer in [0, 255]"
        self._cal_status = value

    @builtins.property
    def autosaved(self):
        """Message field 'autosaved'."""
        return self._autosaved

    @autosaved.setter
    def autosaved(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'autosaved' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'autosaved' field must be an unsigned integer in [0, 255]"
        self._autosaved = value

    @builtins.property
    def fitness(self):
        """Message field 'fitness'."""
        return self._fitness

    @fitness.setter
    def fitness(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fitness' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'fitness' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._fitness = value

    @builtins.property
    def ofs_x(self):
        """Message field 'ofs_x'."""
        return self._ofs_x

    @ofs_x.setter
    def ofs_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ofs_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'ofs_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ofs_x = value

    @builtins.property
    def ofs_y(self):
        """Message field 'ofs_y'."""
        return self._ofs_y

    @ofs_y.setter
    def ofs_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ofs_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'ofs_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ofs_y = value

    @builtins.property
    def ofs_z(self):
        """Message field 'ofs_z'."""
        return self._ofs_z

    @ofs_z.setter
    def ofs_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ofs_z' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'ofs_z' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ofs_z = value

    @builtins.property
    def diag_x(self):
        """Message field 'diag_x'."""
        return self._diag_x

    @diag_x.setter
    def diag_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'diag_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'diag_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._diag_x = value

    @builtins.property
    def diag_y(self):
        """Message field 'diag_y'."""
        return self._diag_y

    @diag_y.setter
    def diag_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'diag_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'diag_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._diag_y = value

    @builtins.property
    def diag_z(self):
        """Message field 'diag_z'."""
        return self._diag_z

    @diag_z.setter
    def diag_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'diag_z' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'diag_z' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._diag_z = value

    @builtins.property
    def offdiag_x(self):
        """Message field 'offdiag_x'."""
        return self._offdiag_x

    @offdiag_x.setter
    def offdiag_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'offdiag_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'offdiag_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._offdiag_x = value

    @builtins.property
    def offdiag_y(self):
        """Message field 'offdiag_y'."""
        return self._offdiag_y

    @offdiag_y.setter
    def offdiag_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'offdiag_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'offdiag_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._offdiag_y = value

    @builtins.property
    def offdiag_z(self):
        """Message field 'offdiag_z'."""
        return self._offdiag_z

    @offdiag_z.setter
    def offdiag_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'offdiag_z' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'offdiag_z' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._offdiag_z = value

    @builtins.property
    def orientation_confidence(self):
        """Message field 'orientation_confidence'."""
        return self._orientation_confidence

    @orientation_confidence.setter
    def orientation_confidence(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'orientation_confidence' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'orientation_confidence' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._orientation_confidence = value

    @builtins.property
    def old_orientation(self):
        """Message field 'old_orientation'."""
        return self._old_orientation

    @old_orientation.setter
    def old_orientation(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'old_orientation' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'old_orientation' field must be an unsigned integer in [0, 255]"
        self._old_orientation = value

    @builtins.property
    def new_orientation(self):
        """Message field 'new_orientation'."""
        return self._new_orientation

    @new_orientation.setter
    def new_orientation(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'new_orientation' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'new_orientation' field must be an unsigned integer in [0, 255]"
        self._new_orientation = value

    @builtins.property
    def scale_factor(self):
        """Message field 'scale_factor'."""
        return self._scale_factor

    @scale_factor.setter
    def scale_factor(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'scale_factor' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'scale_factor' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._scale_factor = value
