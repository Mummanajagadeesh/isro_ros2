# generated from rosidl_generator_py/resource/_idl.py.em
# with input from mavros_msgs:msg/OpenDroneIDBasicID.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OpenDroneIDBasicID(type):
    """Metaclass of message 'OpenDroneIDBasicID'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'ID_TYPE_NONE': 0,
        'ID_TYPE_SERIAL_NUMBER': 1,
        'ID_TYPE_CAA_REGISTRATION_ID': 2,
        'ID_TYPE_UTM_ASSIGNED_UUID': 3,
        'ID_TYPE_SPECIFIC_SESSION_ID': 4,
        'UA_TYPE_NONE': 0,
        'UA_TYPE_AEROPLANE': 1,
        'UA_TYPE_HELICOPTER_OR_MULTIROTOR': 2,
        'UA_TYPE_GYROPLANE': 3,
        'UA_TYPE_HYBRID_LIFT': 4,
        'UA_TYPE_ORNITHOPTER': 5,
        'UA_TYPE_GLIDER': 6,
        'UA_TYPE_KITE': 7,
        'UA_TYPE_FREE_BALLOON': 8,
        'UA_TYPE_CAPTIVE_BALLOON': 9,
        'UA_TYPE_AIRSHIP': 10,
        'UA_TYPE_FREE_FALL_PARACHUTE': 11,
        'UA_TYPE_ROCKET': 12,
        'UA_TYPE_TETHERED_POWERED_AIRCRAFT': 13,
        'UA_TYPE_GROUND_OBSTACLE': 14,
        'UA_TYPE_OTHER': 15,
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
                'mavros_msgs.msg.OpenDroneIDBasicID')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__open_drone_id_basic_id
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__open_drone_id_basic_id
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__open_drone_id_basic_id
            cls._TYPE_SUPPORT = module.type_support_msg__msg__open_drone_id_basic_id
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__open_drone_id_basic_id

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ID_TYPE_NONE': cls.__constants['ID_TYPE_NONE'],
            'ID_TYPE_SERIAL_NUMBER': cls.__constants['ID_TYPE_SERIAL_NUMBER'],
            'ID_TYPE_CAA_REGISTRATION_ID': cls.__constants['ID_TYPE_CAA_REGISTRATION_ID'],
            'ID_TYPE_UTM_ASSIGNED_UUID': cls.__constants['ID_TYPE_UTM_ASSIGNED_UUID'],
            'ID_TYPE_SPECIFIC_SESSION_ID': cls.__constants['ID_TYPE_SPECIFIC_SESSION_ID'],
            'UA_TYPE_NONE': cls.__constants['UA_TYPE_NONE'],
            'UA_TYPE_AEROPLANE': cls.__constants['UA_TYPE_AEROPLANE'],
            'UA_TYPE_HELICOPTER_OR_MULTIROTOR': cls.__constants['UA_TYPE_HELICOPTER_OR_MULTIROTOR'],
            'UA_TYPE_GYROPLANE': cls.__constants['UA_TYPE_GYROPLANE'],
            'UA_TYPE_HYBRID_LIFT': cls.__constants['UA_TYPE_HYBRID_LIFT'],
            'UA_TYPE_ORNITHOPTER': cls.__constants['UA_TYPE_ORNITHOPTER'],
            'UA_TYPE_GLIDER': cls.__constants['UA_TYPE_GLIDER'],
            'UA_TYPE_KITE': cls.__constants['UA_TYPE_KITE'],
            'UA_TYPE_FREE_BALLOON': cls.__constants['UA_TYPE_FREE_BALLOON'],
            'UA_TYPE_CAPTIVE_BALLOON': cls.__constants['UA_TYPE_CAPTIVE_BALLOON'],
            'UA_TYPE_AIRSHIP': cls.__constants['UA_TYPE_AIRSHIP'],
            'UA_TYPE_FREE_FALL_PARACHUTE': cls.__constants['UA_TYPE_FREE_FALL_PARACHUTE'],
            'UA_TYPE_ROCKET': cls.__constants['UA_TYPE_ROCKET'],
            'UA_TYPE_TETHERED_POWERED_AIRCRAFT': cls.__constants['UA_TYPE_TETHERED_POWERED_AIRCRAFT'],
            'UA_TYPE_GROUND_OBSTACLE': cls.__constants['UA_TYPE_GROUND_OBSTACLE'],
            'UA_TYPE_OTHER': cls.__constants['UA_TYPE_OTHER'],
        }

    @property
    def ID_TYPE_NONE(self):
        """Message constant 'ID_TYPE_NONE'."""
        return Metaclass_OpenDroneIDBasicID.__constants['ID_TYPE_NONE']

    @property
    def ID_TYPE_SERIAL_NUMBER(self):
        """Message constant 'ID_TYPE_SERIAL_NUMBER'."""
        return Metaclass_OpenDroneIDBasicID.__constants['ID_TYPE_SERIAL_NUMBER']

    @property
    def ID_TYPE_CAA_REGISTRATION_ID(self):
        """Message constant 'ID_TYPE_CAA_REGISTRATION_ID'."""
        return Metaclass_OpenDroneIDBasicID.__constants['ID_TYPE_CAA_REGISTRATION_ID']

    @property
    def ID_TYPE_UTM_ASSIGNED_UUID(self):
        """Message constant 'ID_TYPE_UTM_ASSIGNED_UUID'."""
        return Metaclass_OpenDroneIDBasicID.__constants['ID_TYPE_UTM_ASSIGNED_UUID']

    @property
    def ID_TYPE_SPECIFIC_SESSION_ID(self):
        """Message constant 'ID_TYPE_SPECIFIC_SESSION_ID'."""
        return Metaclass_OpenDroneIDBasicID.__constants['ID_TYPE_SPECIFIC_SESSION_ID']

    @property
    def UA_TYPE_NONE(self):
        """Message constant 'UA_TYPE_NONE'."""
        return Metaclass_OpenDroneIDBasicID.__constants['UA_TYPE_NONE']

    @property
    def UA_TYPE_AEROPLANE(self):
        """Message constant 'UA_TYPE_AEROPLANE'."""
        return Metaclass_OpenDroneIDBasicID.__constants['UA_TYPE_AEROPLANE']

    @property
    def UA_TYPE_HELICOPTER_OR_MULTIROTOR(self):
        """Message constant 'UA_TYPE_HELICOPTER_OR_MULTIROTOR'."""
        return Metaclass_OpenDroneIDBasicID.__constants['UA_TYPE_HELICOPTER_OR_MULTIROTOR']

    @property
    def UA_TYPE_GYROPLANE(self):
        """Message constant 'UA_TYPE_GYROPLANE'."""
        return Metaclass_OpenDroneIDBasicID.__constants['UA_TYPE_GYROPLANE']

    @property
    def UA_TYPE_HYBRID_LIFT(self):
        """Message constant 'UA_TYPE_HYBRID_LIFT'."""
        return Metaclass_OpenDroneIDBasicID.__constants['UA_TYPE_HYBRID_LIFT']

    @property
    def UA_TYPE_ORNITHOPTER(self):
        """Message constant 'UA_TYPE_ORNITHOPTER'."""
        return Metaclass_OpenDroneIDBasicID.__constants['UA_TYPE_ORNITHOPTER']

    @property
    def UA_TYPE_GLIDER(self):
        """Message constant 'UA_TYPE_GLIDER'."""
        return Metaclass_OpenDroneIDBasicID.__constants['UA_TYPE_GLIDER']

    @property
    def UA_TYPE_KITE(self):
        """Message constant 'UA_TYPE_KITE'."""
        return Metaclass_OpenDroneIDBasicID.__constants['UA_TYPE_KITE']

    @property
    def UA_TYPE_FREE_BALLOON(self):
        """Message constant 'UA_TYPE_FREE_BALLOON'."""
        return Metaclass_OpenDroneIDBasicID.__constants['UA_TYPE_FREE_BALLOON']

    @property
    def UA_TYPE_CAPTIVE_BALLOON(self):
        """Message constant 'UA_TYPE_CAPTIVE_BALLOON'."""
        return Metaclass_OpenDroneIDBasicID.__constants['UA_TYPE_CAPTIVE_BALLOON']

    @property
    def UA_TYPE_AIRSHIP(self):
        """Message constant 'UA_TYPE_AIRSHIP'."""
        return Metaclass_OpenDroneIDBasicID.__constants['UA_TYPE_AIRSHIP']

    @property
    def UA_TYPE_FREE_FALL_PARACHUTE(self):
        """Message constant 'UA_TYPE_FREE_FALL_PARACHUTE'."""
        return Metaclass_OpenDroneIDBasicID.__constants['UA_TYPE_FREE_FALL_PARACHUTE']

    @property
    def UA_TYPE_ROCKET(self):
        """Message constant 'UA_TYPE_ROCKET'."""
        return Metaclass_OpenDroneIDBasicID.__constants['UA_TYPE_ROCKET']

    @property
    def UA_TYPE_TETHERED_POWERED_AIRCRAFT(self):
        """Message constant 'UA_TYPE_TETHERED_POWERED_AIRCRAFT'."""
        return Metaclass_OpenDroneIDBasicID.__constants['UA_TYPE_TETHERED_POWERED_AIRCRAFT']

    @property
    def UA_TYPE_GROUND_OBSTACLE(self):
        """Message constant 'UA_TYPE_GROUND_OBSTACLE'."""
        return Metaclass_OpenDroneIDBasicID.__constants['UA_TYPE_GROUND_OBSTACLE']

    @property
    def UA_TYPE_OTHER(self):
        """Message constant 'UA_TYPE_OTHER'."""
        return Metaclass_OpenDroneIDBasicID.__constants['UA_TYPE_OTHER']


class OpenDroneIDBasicID(metaclass=Metaclass_OpenDroneIDBasicID):
    """
    Message class 'OpenDroneIDBasicID'.

    Constants:
      ID_TYPE_NONE
      ID_TYPE_SERIAL_NUMBER
      ID_TYPE_CAA_REGISTRATION_ID
      ID_TYPE_UTM_ASSIGNED_UUID
      ID_TYPE_SPECIFIC_SESSION_ID
      UA_TYPE_NONE
      UA_TYPE_AEROPLANE
      UA_TYPE_HELICOPTER_OR_MULTIROTOR
      UA_TYPE_GYROPLANE
      UA_TYPE_HYBRID_LIFT
      UA_TYPE_ORNITHOPTER
      UA_TYPE_GLIDER
      UA_TYPE_KITE
      UA_TYPE_FREE_BALLOON
      UA_TYPE_CAPTIVE_BALLOON
      UA_TYPE_AIRSHIP
      UA_TYPE_FREE_FALL_PARACHUTE
      UA_TYPE_ROCKET
      UA_TYPE_TETHERED_POWERED_AIRCRAFT
      UA_TYPE_GROUND_OBSTACLE
      UA_TYPE_OTHER
    """

    __slots__ = [
        '_header',
        '_id_or_mac',
        '_id_type',
        '_ua_type',
        '_uas_id',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'id_or_mac': 'string',
        'id_type': 'uint8',
        'ua_type': 'uint8',
        'uas_id': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.id_or_mac = kwargs.get('id_or_mac', str())
        self.id_type = kwargs.get('id_type', int())
        self.ua_type = kwargs.get('ua_type', int())
        self.uas_id = kwargs.get('uas_id', str())

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
        if self.id_or_mac != other.id_or_mac:
            return False
        if self.id_type != other.id_type:
            return False
        if self.ua_type != other.ua_type:
            return False
        if self.uas_id != other.uas_id:
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
    def id_or_mac(self):
        """Message field 'id_or_mac'."""
        return self._id_or_mac

    @id_or_mac.setter
    def id_or_mac(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'id_or_mac' field must be of type 'str'"
        self._id_or_mac = value

    @builtins.property
    def id_type(self):
        """Message field 'id_type'."""
        return self._id_type

    @id_type.setter
    def id_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'id_type' field must be an unsigned integer in [0, 255]"
        self._id_type = value

    @builtins.property
    def ua_type(self):
        """Message field 'ua_type'."""
        return self._ua_type

    @ua_type.setter
    def ua_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ua_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'ua_type' field must be an unsigned integer in [0, 255]"
        self._ua_type = value

    @builtins.property
    def uas_id(self):
        """Message field 'uas_id'."""
        return self._uas_id

    @uas_id.setter
    def uas_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'uas_id' field must be of type 'str'"
        self._uas_id = value
