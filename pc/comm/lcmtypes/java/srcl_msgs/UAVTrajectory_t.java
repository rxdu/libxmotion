/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package srcl_msgs;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class UAVTrajectory_t implements lcm.lcm.LCMEncodable
{
    public long waypoint_num;
    public srcl_msgs.UAVTrajectoryPoint_t trajectory[];
 
    public UAVTrajectory_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xf3438377578526b8L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(srcl_msgs.UAVTrajectory_t.class))
            return 0L;
 
        classes.add(srcl_msgs.UAVTrajectory_t.class);
        long hash = LCM_FINGERPRINT_BASE
             + srcl_msgs.UAVTrajectoryPoint_t._hashRecursive(classes)
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
        outs.writeLong(this.waypoint_num); 
 
        for (int a = 0; a < this.waypoint_num; a++) {
            this.trajectory[a]._encodeRecursive(outs); 
        }
 
    }
 
    public UAVTrajectory_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public UAVTrajectory_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static srcl_msgs.UAVTrajectory_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        srcl_msgs.UAVTrajectory_t o = new srcl_msgs.UAVTrajectory_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.waypoint_num = ins.readLong();
 
        this.trajectory = new srcl_msgs.UAVTrajectoryPoint_t[(int) waypoint_num];
        for (int a = 0; a < this.waypoint_num; a++) {
            this.trajectory[a] = srcl_msgs.UAVTrajectoryPoint_t._decodeRecursiveFactory(ins);
        }
 
    }
 
    public srcl_msgs.UAVTrajectory_t copy()
    {
        srcl_msgs.UAVTrajectory_t outobj = new srcl_msgs.UAVTrajectory_t();
        outobj.waypoint_num = this.waypoint_num;
 
        outobj.trajectory = new srcl_msgs.UAVTrajectoryPoint_t[(int) waypoint_num];
        for (int a = 0; a < this.waypoint_num; a++) {
            outobj.trajectory[a] = this.trajectory[a].copy();
        }
 
        return outobj;
    }
 
}

