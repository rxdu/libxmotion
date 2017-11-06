/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/rdu/Workspace/librav/src/libraries/uavcan/libuavcan/test/dsdl_test/root_ns_a/NestedMessage.uavcan
 */

#ifndef ROOT_NS_A_NESTEDMESSAGE_HPP_INCLUDED
#define ROOT_NS_A_NESTEDMESSAGE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <root_ns_a/EmptyMessage.hpp>

/******************************* Source text **********************************
float32 VALUE  =  2  +(  2  *2)
bool BOOLEAN = true + false
int2 field
EmptyMessage empty
******************************************************************************/

/********************* DSDL signature source definition ***********************
root_ns_a.NestedMessage
saturated int2 field
root_ns_a.EmptyMessage empty
******************************************************************************/

#undef field
#undef empty
#undef VALUE
#undef BOOLEAN

namespace root_ns_a
{

template <int _tmpl>
struct UAVCAN_EXPORT NestedMessage_
{
    typedef const NestedMessage_<_tmpl>& ParameterType;
    typedef NestedMessage_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > VALUE;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > BOOLEAN;
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 2, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > field;
        typedef ::root_ns_a::EmptyMessage empty;
    };

    enum
    {
        MinBitLen
            = FieldTypes::field::MinBitLen
            + FieldTypes::empty::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::field::MaxBitLen
            + FieldTypes::empty::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::VALUE >::Type VALUE; // 2+(2*2)
    static const typename ::uavcan::StorageType< typename ConstantTypes::BOOLEAN >::Type BOOLEAN; // true+false

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::field >::Type field;
    typename ::uavcan::StorageType< typename FieldTypes::empty >::Type empty;

    NestedMessage_()
        : field()
        , empty()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<2 == MaxBitLen>::check();
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
    // This type has no default data type ID

    static const char* getDataTypeFullName()
    {
        return "root_ns_a.NestedMessage";
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
bool NestedMessage_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        field == rhs.field &&
        empty == rhs.empty;
}

template <int _tmpl>
bool NestedMessage_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(field, rhs.field) &&
        ::uavcan::areClose(empty, rhs.empty);
}

template <int _tmpl>
int NestedMessage_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::field::encode(self.field, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::empty::encode(self.empty, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int NestedMessage_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::field::decode(self.field, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::empty::decode(self.empty, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature NestedMessage_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x67C93D41724FE10DULL);

    FieldTypes::field::extendDataTypeSignature(signature);
    FieldTypes::empty::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename NestedMessage_<_tmpl>::ConstantTypes::VALUE >::Type
    NestedMessage_<_tmpl>::VALUE = 6.0; // 2+(2*2)

template <int _tmpl>
const typename ::uavcan::StorageType< typename NestedMessage_<_tmpl>::ConstantTypes::BOOLEAN >::Type
    NestedMessage_<_tmpl>::BOOLEAN = 1; // true+false

/*
 * Final typedef
 */
typedef NestedMessage_<0> NestedMessage;

// No default registration

} // Namespace root_ns_a

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::root_ns_a::NestedMessage >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::root_ns_a::NestedMessage::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::root_ns_a::NestedMessage >::stream(Stream& s, ::root_ns_a::NestedMessage::ParameterType obj, const int level)
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
    s << "field: ";
    YamlStreamer< ::root_ns_a::NestedMessage::FieldTypes::field >::stream(s, obj.field, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "empty: ";
    YamlStreamer< ::root_ns_a::NestedMessage::FieldTypes::empty >::stream(s, obj.empty, level + 1);
}

}

namespace root_ns_a
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::root_ns_a::NestedMessage::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::root_ns_a::NestedMessage >::stream(s, obj, 0);
    return s;
}

} // Namespace root_ns_a

#endif // ROOT_NS_A_NESTEDMESSAGE_HPP_INCLUDED