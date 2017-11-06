/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/rdu/Workspace/librav/src/libraries/uavcan/dsdl/uavcan/equipment/camera_gimbal/1041.GEOPOICommand.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_CAMERA_GIMBAL_GEOPOICOMMAND_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_CAMERA_GIMBAL_GEOPOICOMMAND_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <uavcan/equipment/camera_gimbal/Mode.hpp>

/******************************* Source text **********************************
#
# Generic camera gimbal control.
#
# This message can only be used in the following modes:
#  - COMMAND_MODE_GEO_POI
#

uint8 gimbal_id

#
# Target operation mode - how to handle this message.
# See the list of acceptable modes above.
#
Mode mode

#
# Coordinates of the POI (point of interest).
#
int32 longitude_deg_1e7    # 1 LSB = 1e-7 deg
int32 latitude_deg_1e7
int22 height_cm            # 1 LSB = 10 mm

uint2 HEIGHT_REFERENCE_ELLIPSOID = 0
uint2 HEIGHT_REFERENCE_MEAN_SEA_LEVEL = 1
uint2 height_reference
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.camera_gimbal.GEOPOICommand
saturated uint8 gimbal_id
uavcan.equipment.camera_gimbal.Mode mode
saturated int32 longitude_deg_1e7
saturated int32 latitude_deg_1e7
saturated int22 height_cm
saturated uint2 height_reference
******************************************************************************/

#undef gimbal_id
#undef mode
#undef longitude_deg_1e7
#undef latitude_deg_1e7
#undef height_cm
#undef height_reference
#undef HEIGHT_REFERENCE_ELLIPSOID
#undef HEIGHT_REFERENCE_MEAN_SEA_LEVEL

namespace uavcan
{
namespace equipment
{
namespace camera_gimbal
{

template <int _tmpl>
struct UAVCAN_EXPORT GEOPOICommand_
{
    typedef const GEOPOICommand_<_tmpl>& ParameterType;
    typedef GEOPOICommand_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 2, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > HEIGHT_REFERENCE_ELLIPSOID;
        typedef ::uavcan::IntegerSpec< 2, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > HEIGHT_REFERENCE_MEAN_SEA_LEVEL;
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > gimbal_id;
        typedef ::uavcan::equipment::camera_gimbal::Mode mode;
        typedef ::uavcan::IntegerSpec< 32, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > longitude_deg_1e7;
        typedef ::uavcan::IntegerSpec< 32, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > latitude_deg_1e7;
        typedef ::uavcan::IntegerSpec< 22, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > height_cm;
        typedef ::uavcan::IntegerSpec< 2, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > height_reference;
    };

    enum
    {
        MinBitLen
            = FieldTypes::gimbal_id::MinBitLen
            + FieldTypes::mode::MinBitLen
            + FieldTypes::longitude_deg_1e7::MinBitLen
            + FieldTypes::latitude_deg_1e7::MinBitLen
            + FieldTypes::height_cm::MinBitLen
            + FieldTypes::height_reference::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::gimbal_id::MaxBitLen
            + FieldTypes::mode::MaxBitLen
            + FieldTypes::longitude_deg_1e7::MaxBitLen
            + FieldTypes::latitude_deg_1e7::MaxBitLen
            + FieldTypes::height_cm::MaxBitLen
            + FieldTypes::height_reference::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::HEIGHT_REFERENCE_ELLIPSOID >::Type HEIGHT_REFERENCE_ELLIPSOID; // 0
    static const typename ::uavcan::StorageType< typename ConstantTypes::HEIGHT_REFERENCE_MEAN_SEA_LEVEL >::Type HEIGHT_REFERENCE_MEAN_SEA_LEVEL; // 1

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::gimbal_id >::Type gimbal_id;
    typename ::uavcan::StorageType< typename FieldTypes::mode >::Type mode;
    typename ::uavcan::StorageType< typename FieldTypes::longitude_deg_1e7 >::Type longitude_deg_1e7;
    typename ::uavcan::StorageType< typename FieldTypes::latitude_deg_1e7 >::Type latitude_deg_1e7;
    typename ::uavcan::StorageType< typename FieldTypes::height_cm >::Type height_cm;
    typename ::uavcan::StorageType< typename FieldTypes::height_reference >::Type height_reference;

    GEOPOICommand_()
        : gimbal_id()
        , mode()
        , longitude_deg_1e7()
        , latitude_deg_1e7()
        , height_cm()
        , height_reference()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<104 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 1041 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.camera_gimbal.GEOPOICommand";
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
bool GEOPOICommand_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        gimbal_id == rhs.gimbal_id &&
        mode == rhs.mode &&
        longitude_deg_1e7 == rhs.longitude_deg_1e7 &&
        latitude_deg_1e7 == rhs.latitude_deg_1e7 &&
        height_cm == rhs.height_cm &&
        height_reference == rhs.height_reference;
}

template <int _tmpl>
bool GEOPOICommand_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(gimbal_id, rhs.gimbal_id) &&
        ::uavcan::areClose(mode, rhs.mode) &&
        ::uavcan::areClose(longitude_deg_1e7, rhs.longitude_deg_1e7) &&
        ::uavcan::areClose(latitude_deg_1e7, rhs.latitude_deg_1e7) &&
        ::uavcan::areClose(height_cm, rhs.height_cm) &&
        ::uavcan::areClose(height_reference, rhs.height_reference);
}

template <int _tmpl>
int GEOPOICommand_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::gimbal_id::encode(self.gimbal_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::mode::encode(self.mode, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::longitude_deg_1e7::encode(self.longitude_deg_1e7, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::latitude_deg_1e7::encode(self.latitude_deg_1e7, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::height_cm::encode(self.height_cm, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::height_reference::encode(self.height_reference, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int GEOPOICommand_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::gimbal_id::decode(self.gimbal_id, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::mode::decode(self.mode, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::longitude_deg_1e7::decode(self.longitude_deg_1e7, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::latitude_deg_1e7::decode(self.latitude_deg_1e7, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::height_cm::decode(self.height_cm, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::height_reference::decode(self.height_reference, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature GEOPOICommand_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x7B0630AB712FC30FULL);

    FieldTypes::gimbal_id::extendDataTypeSignature(signature);
    FieldTypes::mode::extendDataTypeSignature(signature);
    FieldTypes::longitude_deg_1e7::extendDataTypeSignature(signature);
    FieldTypes::latitude_deg_1e7::extendDataTypeSignature(signature);
    FieldTypes::height_cm::extendDataTypeSignature(signature);
    FieldTypes::height_reference::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename GEOPOICommand_<_tmpl>::ConstantTypes::HEIGHT_REFERENCE_ELLIPSOID >::Type
    GEOPOICommand_<_tmpl>::HEIGHT_REFERENCE_ELLIPSOID = 0U; // 0

template <int _tmpl>
const typename ::uavcan::StorageType< typename GEOPOICommand_<_tmpl>::ConstantTypes::HEIGHT_REFERENCE_MEAN_SEA_LEVEL >::Type
    GEOPOICommand_<_tmpl>::HEIGHT_REFERENCE_MEAN_SEA_LEVEL = 1U; // 1

/*
 * Final typedef
 */
typedef GEOPOICommand_<0> GEOPOICommand;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::camera_gimbal::GEOPOICommand > _uavcan_gdtr_registrator_GEOPOICommand;

}

} // Namespace camera_gimbal
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::camera_gimbal::GEOPOICommand >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::camera_gimbal::GEOPOICommand::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::camera_gimbal::GEOPOICommand >::stream(Stream& s, ::uavcan::equipment::camera_gimbal::GEOPOICommand::ParameterType obj, const int level)
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
    s << "gimbal_id: ";
    YamlStreamer< ::uavcan::equipment::camera_gimbal::GEOPOICommand::FieldTypes::gimbal_id >::stream(s, obj.gimbal_id, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "mode: ";
    YamlStreamer< ::uavcan::equipment::camera_gimbal::GEOPOICommand::FieldTypes::mode >::stream(s, obj.mode, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "longitude_deg_1e7: ";
    YamlStreamer< ::uavcan::equipment::camera_gimbal::GEOPOICommand::FieldTypes::longitude_deg_1e7 >::stream(s, obj.longitude_deg_1e7, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "latitude_deg_1e7: ";
    YamlStreamer< ::uavcan::equipment::camera_gimbal::GEOPOICommand::FieldTypes::latitude_deg_1e7 >::stream(s, obj.latitude_deg_1e7, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "height_cm: ";
    YamlStreamer< ::uavcan::equipment::camera_gimbal::GEOPOICommand::FieldTypes::height_cm >::stream(s, obj.height_cm, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "height_reference: ";
    YamlStreamer< ::uavcan::equipment::camera_gimbal::GEOPOICommand::FieldTypes::height_reference >::stream(s, obj.height_reference, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace camera_gimbal
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::camera_gimbal::GEOPOICommand::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::camera_gimbal::GEOPOICommand >::stream(s, obj, 0);
    return s;
}

} // Namespace camera_gimbal
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_CAMERA_GIMBAL_GEOPOICOMMAND_HPP_INCLUDED