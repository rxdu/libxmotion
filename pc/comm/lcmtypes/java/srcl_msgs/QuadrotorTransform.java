/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package srcl_msgs;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class QuadrotorTransform implements lcm.lcm.LCMEncodable
{
    public srcl_msgs.Pose_t base_to_world;
    public srcl_msgs.Pose_t laser_to_base;
 
    public QuadrotorTransform()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x10299cea50417ef5L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(srcl_msgs.QuadrotorTransform.class))
            return 0L;
 
        classes.add(srcl_msgs.QuadrotorTransform.class);
        long hash = LCM_FINGERPRINT_BASE
             + srcl_msgs.Pose_t._hashRecursive(classes)
             + srcl_msgs.Pose_t._hashRecursive(classes)
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        this.base_to_world._encodeRecursive(outs); 
 
        this.laser_to_base._encodeRecursive(outs); 
 
    }
 
    public QuadrotorTransform(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public QuadrotorTransform(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static srcl_msgs.QuadrotorTransform _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        srcl_msgs.QuadrotorTransform o = new srcl_msgs.QuadrotorTransform();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.base_to_world = srcl_msgs.Pose_t._decodeRecursiveFactory(ins);
 
        this.laser_to_base = srcl_msgs.Pose_t._decodeRecursiveFactory(ins);
 
    }
 
    public srcl_msgs.QuadrotorTransform copy()
    {
        srcl_msgs.QuadrotorTransform outobj = new srcl_msgs.QuadrotorTransform();
        outobj.base_to_world = this.base_to_world.copy();
 
        outobj.laser_to_base = this.laser_to_base.copy();
 
        return outobj;
    }
 
}

