/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package srcl_msgs;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class Point3Df_t implements lcm.lcm.LCMEncodable
{
    public float x;
    public float y;
    public float z;
 
    public Point3Df_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x2a14f112c253ac0cL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(srcl_msgs.Point3Df_t.class))
            return 0L;
 
        classes.add(srcl_msgs.Point3Df_t.class);
        long hash = LCM_FINGERPRINT_BASE
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
        outs.writeFloat(this.x); 
 
        outs.writeFloat(this.y); 
 
        outs.writeFloat(this.z); 
 
    }
 
    public Point3Df_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public Point3Df_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static srcl_msgs.Point3Df_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        srcl_msgs.Point3Df_t o = new srcl_msgs.Point3Df_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.x = ins.readFloat();
 
        this.y = ins.readFloat();
 
        this.z = ins.readFloat();
 
    }
 
    public srcl_msgs.Point3Df_t copy()
    {
        srcl_msgs.Point3Df_t outobj = new srcl_msgs.Point3Df_t();
        outobj.x = this.x;
 
        outobj.y = this.y;
 
        outobj.z = this.z;
 
        return outobj;
    }
 
}

