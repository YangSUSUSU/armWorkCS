# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from llm_msgs/set_speedRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class set_speedRequest(genpy.Message):
  _md5sum = "fce368f208823771f7ce8907032a98d8"
  _type = "llm_msgs/set_speedRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float32 speed0Ratio
float32 speed1Ratio
float32 speed2Ratio
float32 speed3Ratio
float32 speed4Ratio
float32 speed5Ratio
"""
  __slots__ = ['speed0Ratio','speed1Ratio','speed2Ratio','speed3Ratio','speed4Ratio','speed5Ratio']
  _slot_types = ['float32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       speed0Ratio,speed1Ratio,speed2Ratio,speed3Ratio,speed4Ratio,speed5Ratio

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(set_speedRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.speed0Ratio is None:
        self.speed0Ratio = 0.
      if self.speed1Ratio is None:
        self.speed1Ratio = 0.
      if self.speed2Ratio is None:
        self.speed2Ratio = 0.
      if self.speed3Ratio is None:
        self.speed3Ratio = 0.
      if self.speed4Ratio is None:
        self.speed4Ratio = 0.
      if self.speed5Ratio is None:
        self.speed5Ratio = 0.
    else:
      self.speed0Ratio = 0.
      self.speed1Ratio = 0.
      self.speed2Ratio = 0.
      self.speed3Ratio = 0.
      self.speed4Ratio = 0.
      self.speed5Ratio = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_6f().pack(_x.speed0Ratio, _x.speed1Ratio, _x.speed2Ratio, _x.speed3Ratio, _x.speed4Ratio, _x.speed5Ratio))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 24
      (_x.speed0Ratio, _x.speed1Ratio, _x.speed2Ratio, _x.speed3Ratio, _x.speed4Ratio, _x.speed5Ratio,) = _get_struct_6f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_6f().pack(_x.speed0Ratio, _x.speed1Ratio, _x.speed2Ratio, _x.speed3Ratio, _x.speed4Ratio, _x.speed5Ratio))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 24
      (_x.speed0Ratio, _x.speed1Ratio, _x.speed2Ratio, _x.speed3Ratio, _x.speed4Ratio, _x.speed5Ratio,) = _get_struct_6f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6f = None
def _get_struct_6f():
    global _struct_6f
    if _struct_6f is None:
        _struct_6f = struct.Struct("<6f")
    return _struct_6f
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from llm_msgs/set_speedResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class set_speedResponse(genpy.Message):
  _md5sum = "9bb3ff7624b31e52edd1d0b2bbbae418"
  _type = "llm_msgs/set_speedResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool speed_accepted

"""
  __slots__ = ['speed_accepted']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       speed_accepted

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(set_speedResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.speed_accepted is None:
        self.speed_accepted = False
    else:
      self.speed_accepted = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.speed_accepted
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 1
      (self.speed_accepted,) = _get_struct_B().unpack(str[start:end])
      self.speed_accepted = bool(self.speed_accepted)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.speed_accepted
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 1
      (self.speed_accepted,) = _get_struct_B().unpack(str[start:end])
      self.speed_accepted = bool(self.speed_accepted)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
class set_speed(object):
  _type          = 'llm_msgs/set_speed'
  _md5sum = '4d1d00d67ce0ba0765e4ad67b563d391'
  _request_class  = set_speedRequest
  _response_class = set_speedResponse