/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/rdu/Workspace/librav/src/libraries/uavcan/dsdl/uavcan/equipment/ahrs/1003.RawIMU.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_AHRS_RAWIMU_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_AHRS_RAWIMU_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <uavcan/Timestamp.hpp>

/******************************* Source text **********************************
#
# Raw IMU data with timestamps.
#
# THIS DEFINITION MAY BE CHANGED IN A NON-BACKWARD-COMPATIBLE WAY IN THE FUTURE.
#

#
# Data acquisition timestamp in the bus shared time base.
#
uavcan.Timestamp timestamp

#
# Integration interval, seconds.
# Set to a non-positive value if the integrated samples are not available
# (in this case, only the latest point samples will be valid).
#
float32 integration_interval

#
# Angular velocity samples in radian/second.
# The samples are represented in the body frame, the axes are ordered as follows:
#   1. angular velocity around X (roll rate)
#   2. angular velocity around Y (pitch rate)
#   3. angular velocity around Z (yaw rate)
#
float16[3] rate_gyro_latest                 # Latest sample, radian/second
float32[3] rate_gyro_integral               # Integrated samples, radian/second

#
# Linear acceleration samples in meter/(second^2).
# The samples are represented in the body frame, the axes are ordered as follows:
#   1. linear acceleration along X (forward positive)
#   2. linear acceleration along Y (right positive)
#   3. linear acceleration along Z (down positive)
#
float16[3] accelerometer_latest             # Latest sample, meter/(second^2)
float32[3] accelerometer_integral           # Integrated samples, meter/(second^2)

#
# Covariance matrix. The diagonal entries are ordered as follows:
#   1. roll rate                (radian^2)/(second^2)
#   2. pitch rate               (radian^2)/(second^2)
#   3. yaw rate                 (radian^2)/(second^2)
#   4. forward acceleration     (meter^2)/(second^4)
#   5. rightward acceleration   (meter^2)/(second^4)
#   6. downward acceleration    (meter^2)/(second^4)
#
float16[<=36] covariance
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.ahrs.RawIMU
uavcan.Timestamp timestamp
saturated float32 integration_interval
saturated float16[3] rate_gyro_latest
saturated float32[3] rate_gyro_integral
saturated float16[3] accelerometer_latest
saturated float32[3] accelerometer_integral
saturated float16[<=36] covariance
******************************************************************************/

#undef timestamp
#undef integration_interval
#undef rate_gyro_latest
#undef rate_gyro_integral
#undef accelerometer_latest
#undef accelerometer_integral
#undef covariance

namespace uavcan
{
namespace equipment
{
namespace ahrs
{

template <int _tmpl>
struct UAVCAN_EXPORT RawIMU_
{
    typedef const RawIMU_<_tmpl>& ParameterType;
    typedef RawIMU_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::Timestamp timestamp;
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > integration_interval;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 3 > rate_gyro_latest;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 3 > rate_gyro_integral;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 3 > accelerometer_latest;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 3 > accelerometer_integral;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 36 > covariance;
    };

    enum
    {
        MinBitLen
            = FieldTypes::timestamp::MinBitLen
            + FieldTypes::integration_interval::MinBitLen
            + FieldTypes::rate_gyro_latest::MinBitLen
            + FieldTypes::rate_gyro_integral::MinBitLen
            + FieldTypes::accelerometer_latest::MinBitLen
            + FieldTypes::accelerometer_integral::MinBitLen
            + FieldTypes::covariance::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::timestamp::MaxBitLen
            + FieldTypes::integration_interval::MaxBitLen
            + FieldTypes::rate_gyro_latest::MaxBitLen
            + FieldTypes::rate_gyro_integral::MaxBitLen
            + FieldTypes::accelerometer_latest::MaxBitLen
            + FieldTypes::accelerometer_integral::MaxBitLen
            + FieldTypes::covariance::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::timestamp >::Type timestamp;
    typename ::uavcan::StorageType< typename FieldTypes::integration_interval >::Type integration_interval;
    typename ::uavcan::StorageType< typename FieldTypes::rate_gyro_latest >::Type rate_gyro_latest;
    typename ::uavcan::StorageType< typename FieldTypes::rate_gyro_integral >::Type rate_gyro_integral;
    typename ::uavcan::StorageType< typename FieldTypes::accelerometer_latest >::Type accelerometer_latest;
    typename ::uavcan::StorageType< typename FieldTypes::accelerometer_integral >::Type accelerometer_integral;
    typename ::uavcan::StorageType< typename FieldTypes::covariance >::Type covariance;

    RawIMU_()
        : timestamp()
        , integration_interval()
        , rate_gyro_latest()
        , rate_gyro_integral()
        , accelerometer_latest()
        , accelerometer_integral()
        , covariance()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<958 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 1003 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.ahrs.RawIMU";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool RawIMU_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        timestamp == rhs.timestamp &&
        integration_interval == rhs.integration_interval &&
        rate_gyro_latest == rhs.rate_gyro_latest &&
        rate_gyro_integral == rhs.rate_gyro_integral &&
        accelerometer_latest == rhs.accelerometer_latest &&
        accelerometer_integral == rhs.accelerometer_integral &&
        covariance == rhs.covariance;
}

template <int _tmpl>
bool RawIMU_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(timestamp, rhs.timestamp) &&
        ::uavcan::areClose(integration_interval, rhs.integration_interval) &&
        ::uavcan::areClose(rate_gyro_latest, rhs.rate_gyro_latest) &&
        ::uavcan::areClose(rate_gyro_integral, rhs.rate_gyro_integral) &&
        ::uavcan::areClose(accelerometer_latest, rhs.accelerometer_latest) &&
        ::uavcan::areClose(accelerometer_integral, rhs.accelerometer_integral) &&
        ::uavcan::areClose(covariance, rhs.covariance);
}

template <int _tmpl>
int RawIMU_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::timestamp::encode(self.timestamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::integration_interval::encode(self.integration_interval, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::rate_gyro_latest::encode(self.rate_gyro_latest, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::rate_gyro_integral::encode(self.rate_gyro_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::accelerometer_latest::encode(self.accelerometer_latest, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::accelerometer_integral::encode(self.accelerometer_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::covariance::encode(self.covariance, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int RawIMU_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::timestamp::decode(self.timestamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::integration_interval::decode(self.integration_interval, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::rate_gyro_latest::decode(self.rate_gyro_latest, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::rate_gyro_integral::decode(self.rate_gyro_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::accelerometer_latest::decode(self.accelerometer_latest, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::accelerometer_integral::decode(self.accelerometer_integral, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::covariance::decode(self.covariance, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature RawIMU_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xB6173FAA5FD293D0ULL);

    FieldTypes::timestamp::extendDataTypeSignature(signature);
    FieldTypes::integration_interval::extendDataTypeSignature(signature);
    FieldTypes::rate_gyro_latest::extendDataTypeSignature(signature);
    FieldTypes::rate_gyro_integral::extendDataTypeSignature(signature);
    FieldTypes::accelerometer_latest::extendDataTypeSignature(signature);
    FieldTypes::accelerometer_integral::extendDataTypeSignature(signature);
    FieldTypes::covariance::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef RawIMU_<0> RawIMU;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::ahrs::RawIMU > _uavcan_gdtr_registrator_RawIMU;

}

} // Namespace ahrs
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::ahrs::RawIMU >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::ahrs::RawIMU::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::ahrs::RawIMU >::stream(Stream& s, ::uavcan::equipment::ahrs::RawIMU::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "timestamp: ";
    YamlStreamer< ::uavcan::equipment::ahrs::RawIMU::FieldTypes::timestamp >::stream(s, obj.timestamp, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "integration_interval: ";
    YamlStreamer< ::uavcan::equipment::ahrs::RawIMU::FieldTypes::integration_interval >::stream(s, obj.integration_interval, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "rate_gyro_latest: ";
    YamlStreamer< ::uavcan::equipment::ahrs::RawIMU::FieldTypes::rate_gyro_latest >::stream(s, obj.rate_gyro_latest, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "rate_gyro_integral: ";
    YamlStreamer< ::uavcan::equipment::ahrs::RawIMU::FieldTypes::rate_gyro_integral >::stream(s, obj.rate_gyro_integral, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "accelerometer_latest: ";
    YamlStreamer< ::uavcan::equipment::ahrs::RawIMU::FieldTypes::accelerometer_latest >::stream(s, obj.accelerometer_latest, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "accelerometer_integral: ";
    YamlStreamer< ::uavcan::equipment::ahrs::RawIMU::FieldTypes::accelerometer_integral >::stream(s, obj.accelerometer_integral, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "covariance: ";
    YamlStreamer< ::uavcan::equipment::ahrs::RawIMU::FieldTypes::covariance >::stream(s, obj.covariance, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace ahrs
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::ahrs::RawIMU::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::ahrs::RawIMU >::stream(s, obj, 0);
    return s;
}

} // Namespace ahrs
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_AHRS_RAWIMU_HPP_INCLUDED