# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from tihan_mpc/mpc_path.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class mpc_path(genpy.Message):
  _md5sum = "85e4d761326cc6c39a08e19f13b42c05"
  _type = "tihan_mpc/mpc_path"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """Header header
string solve_status # e.g. optimal, infeasible, etc.
float64 solve_time  # time used to solve the optimization problem in seconds
float64[] xs        # MPC solution for x coordinate (m)
float64[] ys        # MPC solution for y coordinate (m)
float64[] vs        # MPC solution for speed (m/s)
float64[] psis      # MPC solution for yaw angle (rad)
float64[] xr        # MPC reference for ""
float64[] yr        # MPC reference for ""
float64[] vr        # MPC reference for ""
float64[] psir      # MPC reference for ""
float64[] df        # MPC solution for front steering angle (rad)
float64[] acc       # MPC solution for acceleration (m/s^2)

float64[] ss        # MPC solution for Frenet s (m)
float64[] eys       # MPC solution for Frenet ey (m)
float64[] epsis     # MPC solution for Frenet epsi (m)

float64[] crf       # curvature reference profile for Frenet (rad/m)
float64 vrf         # curvature-aware velocity reference for Frenet (m/s)
bool wp_end

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id
"""
  __slots__ = ['header','solve_status','solve_time','xs','ys','vs','psis','xr','yr','vr','psir','df','acc','ss','eys','epsis','crf','vrf','wp_end']
  _slot_types = ['std_msgs/Header','string','float64','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64[]','float64','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,solve_status,solve_time,xs,ys,vs,psis,xr,yr,vr,psir,df,acc,ss,eys,epsis,crf,vrf,wp_end

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(mpc_path, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.solve_status is None:
        self.solve_status = ''
      if self.solve_time is None:
        self.solve_time = 0.
      if self.xs is None:
        self.xs = []
      if self.ys is None:
        self.ys = []
      if self.vs is None:
        self.vs = []
      if self.psis is None:
        self.psis = []
      if self.xr is None:
        self.xr = []
      if self.yr is None:
        self.yr = []
      if self.vr is None:
        self.vr = []
      if self.psir is None:
        self.psir = []
      if self.df is None:
        self.df = []
      if self.acc is None:
        self.acc = []
      if self.ss is None:
        self.ss = []
      if self.eys is None:
        self.eys = []
      if self.epsis is None:
        self.epsis = []
      if self.crf is None:
        self.crf = []
      if self.vrf is None:
        self.vrf = 0.
      if self.wp_end is None:
        self.wp_end = False
    else:
      self.header = std_msgs.msg.Header()
      self.solve_status = ''
      self.solve_time = 0.
      self.xs = []
      self.ys = []
      self.vs = []
      self.psis = []
      self.xr = []
      self.yr = []
      self.vr = []
      self.psir = []
      self.df = []
      self.acc = []
      self.ss = []
      self.eys = []
      self.epsis = []
      self.crf = []
      self.vrf = 0.
      self.wp_end = False

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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.solve_status
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.solve_time
      buff.write(_get_struct_d().pack(_x))
      length = len(self.xs)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.xs))
      length = len(self.ys)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.ys))
      length = len(self.vs)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.vs))
      length = len(self.psis)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.psis))
      length = len(self.xr)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.xr))
      length = len(self.yr)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.yr))
      length = len(self.vr)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.vr))
      length = len(self.psir)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.psir))
      length = len(self.df)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.df))
      length = len(self.acc)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.acc))
      length = len(self.ss)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.ss))
      length = len(self.eys)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.eys))
      length = len(self.epsis)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.epsis))
      length = len(self.crf)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(struct.Struct(pattern).pack(*self.crf))
      _x = self
      buff.write(_get_struct_dB().pack(_x.vrf, _x.wp_end))
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
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.solve_status = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.solve_status = str[start:end]
      start = end
      end += 8
      (self.solve_time,) = _get_struct_d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.xs = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.ys = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.vs = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.psis = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.xr = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.yr = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.vr = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.psir = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.df = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.acc = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.ss = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.eys = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.epsis = s.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.crf = s.unpack(str[start:end])
      _x = self
      start = end
      end += 9
      (_x.vrf, _x.wp_end,) = _get_struct_dB().unpack(str[start:end])
      self.wp_end = bool(self.wp_end)
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
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.solve_status
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self.solve_time
      buff.write(_get_struct_d().pack(_x))
      length = len(self.xs)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.xs.tostring())
      length = len(self.ys)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.ys.tostring())
      length = len(self.vs)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.vs.tostring())
      length = len(self.psis)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.psis.tostring())
      length = len(self.xr)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.xr.tostring())
      length = len(self.yr)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.yr.tostring())
      length = len(self.vr)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.vr.tostring())
      length = len(self.psir)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.psir.tostring())
      length = len(self.df)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.df.tostring())
      length = len(self.acc)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.acc.tostring())
      length = len(self.ss)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.ss.tostring())
      length = len(self.eys)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.eys.tostring())
      length = len(self.epsis)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.epsis.tostring())
      length = len(self.crf)
      buff.write(_struct_I.pack(length))
      pattern = '<%sd'%length
      buff.write(self.crf.tostring())
      _x = self
      buff.write(_get_struct_dB().pack(_x.vrf, _x.wp_end))
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
      if self.header is None:
        self.header = std_msgs.msg.Header()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.solve_status = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.solve_status = str[start:end]
      start = end
      end += 8
      (self.solve_time,) = _get_struct_d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.xs = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.ys = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.vs = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.psis = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.xr = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.yr = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.vr = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.psir = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.df = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.acc = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.ss = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.eys = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.epsis = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sd'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.crf = numpy.frombuffer(str[start:end], dtype=numpy.float64, count=length)
      _x = self
      start = end
      end += 9
      (_x.vrf, _x.wp_end,) = _get_struct_dB().unpack(str[start:end])
      self.wp_end = bool(self.wp_end)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_d = None
def _get_struct_d():
    global _struct_d
    if _struct_d is None:
        _struct_d = struct.Struct("<d")
    return _struct_d
_struct_dB = None
def _get_struct_dB():
    global _struct_dB
    if _struct_dB is None:
        _struct_dB = struct.Struct("<dB")
    return _struct_dB
