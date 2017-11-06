"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class Keyframe_t(object):
    __slots__ = ["position", "velocity", "yaw", "vel_constr"]

    def __init__(self):
        self.position = [ 0.0 for dim0 in range(3) ]
        self.velocity = [ 0.0 for dim0 in range(3) ]
        self.yaw = 0.0
        self.vel_constr = False

    def encode(self):
        buf = BytesIO()
        buf.write(Keyframe_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack('>3f', *self.position[:3]))
        buf.write(struct.pack('>3f', *self.velocity[:3]))
        buf.write(struct.pack(">fb", self.yaw, self.vel_constr))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != Keyframe_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return Keyframe_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = Keyframe_t()
        self.position = struct.unpack('>3f', buf.read(12))
        self.velocity = struct.unpack('>3f', buf.read(12))
        self.yaw = struct.unpack(">f", buf.read(4))[0]
        self.vel_constr = bool(struct.unpack('b', buf.read(1))[0])
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if Keyframe_t in parents: return 0
        tmphash = (0xc5f4c6b7363280a5) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if Keyframe_t._packed_fingerprint is None:
            Keyframe_t._packed_fingerprint = struct.pack(">Q", Keyframe_t._get_hash_recursive([]))
        return Keyframe_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

