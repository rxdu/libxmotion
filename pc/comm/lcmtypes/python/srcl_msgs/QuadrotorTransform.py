"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import srcl_msgs.Pose_t

class QuadrotorTransform(object):
    __slots__ = ["base_to_world", "laser_to_base"]

    def __init__(self):
        self.base_to_world = srcl_msgs.Pose_t()
        self.laser_to_base = srcl_msgs.Pose_t()

    def encode(self):
        buf = BytesIO()
        buf.write(QuadrotorTransform._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        assert self.base_to_world._get_packed_fingerprint() == srcl_msgs.Pose_t._get_packed_fingerprint()
        self.base_to_world._encode_one(buf)
        assert self.laser_to_base._get_packed_fingerprint() == srcl_msgs.Pose_t._get_packed_fingerprint()
        self.laser_to_base._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != QuadrotorTransform._get_packed_fingerprint():
            raise ValueError("Decode error")
        return QuadrotorTransform._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = QuadrotorTransform()
        self.base_to_world = srcl_msgs.Pose_t._decode_one(buf)
        self.laser_to_base = srcl_msgs.Pose_t._decode_one(buf)
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if QuadrotorTransform in parents: return 0
        newparents = parents + [QuadrotorTransform]
        tmphash = (0x10299cea50417ef5+ srcl_msgs.Pose_t._get_hash_recursive(newparents)+ srcl_msgs.Pose_t._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if QuadrotorTransform._packed_fingerprint is None:
            QuadrotorTransform._packed_fingerprint = struct.pack(">Q", QuadrotorTransform._get_hash_recursive([]))
        return QuadrotorTransform._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

