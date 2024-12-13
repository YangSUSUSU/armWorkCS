# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from llm_msgs/set_angleRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class set_angleRequest(genpy.Message):
  _md5sum = "dc73e456d4729e71af66373d93d820b7"
  _type = "llm_msgs/set_angleRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """float32 angle0Ratio
float32 angle1Ratio
float32 angle2Ratio
float32 angle3Ratio
float32 angle4Ratio
float32 angle5Ratio
"""
  __slots__ = ['angle0Ratio','angle1Ratio','angle2Ratio','angle3Ratio','angle4Ratio','angle5Ratio']
  _slot_types = ['float32','float32','float32','float32','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       angle0Ratio,angle1Ratio,angle2Ratio,angle3Ratio,angle4Ratio,angle5Ratio

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(set_angleRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.angle0Ratio is None:
        self.angle0Ratio = 0.
      if self.angle1Ratio is None:
        self.angle1Ratio = 0.
      if self.angle2Ratio is None:
        self.angle2Ratio = 0.
      if self.angle3Ratio is None:
        self.angle3Ratio = 0.
      if self.angle4Ratio is None:
        self.angle4Ratio = 0.
      if self.angle5Ratio is None:
        self.angle5Ratio = 0.
    else:
      self.angle0Ratio = 0.
      self.angle1Ratio = 0.
      self.angle2Ratio = 0.
      self.angle3Ratio = 0.
      self.angle4Ratio = 0.
      self.angle5Ratio = 0.

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
      buff.write(_get_struct_6f().pack(_x.angle0Ratio, _x.angle1Ratio, _x.angle2Ratio, _x.angle3Ratio, _x.angle4Ratio, _x.angle5Ratio))
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
      (_x.angle0Ratio, _x.angle1Ratio, _x.angle2Ratio, _x.angle3Ratio, _x.angle4Ratio, _x.angle5Ratio,) = _get_struct_6f().unpack(str[start:end])
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
      buff.write(_get_struct_6f().pack(_x.angle0Ratio, _x.angle1Ratio, _x.angle2Ratio, _x.angle3Ratio, _x.angle4Ratio, _x.angle5Ratio))
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
      (_x.angle0Ratio, _x.angle1Ratio, _x.angle2Ratio, _x.angle3Ratio, _x.angle4Ratio, _x.angle5Ratio,) = _get_struct_6f().unpack(str[start:end])
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
"""autogenerated by genpy from llm_msgs/set_angleResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class set_angleResponse(genpy.Message):
  _md5sum = "c1508b076c4c46f43d5103fcfc81271e"
  _type = "llm_msgs/set_angleResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool angle_accepted
"""
  __slots__ = ['angle_accepted']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       angle_accepted

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(set_angleResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.angle_accepted is None:
        self.angle_accepted = False
    else:
      self.angle_accepted = False

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
      _x = self.angle_accepted
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
      (self.angle_accepted,) = _get_struct_B().unpack(str[start:end])
      self.angle_accepted = bool(self.angle_accepted)
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
      _x = self.angle_accepted
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
      (self.angle_accepted,) = _get_struct_B().unpack(str[start:end])
      self.angle_accepted = bool(self.angle_accepted)
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
class set_angle(object):
  _type          = 'llm_msgs/set_angle'
  _md5sum = '3f54cd874965b27feb654abae7abbdcb'
  _request_class  = set_angleRequest
  _response_class = set_angleResponse