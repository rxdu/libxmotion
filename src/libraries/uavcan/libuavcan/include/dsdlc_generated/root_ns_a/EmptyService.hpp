/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/rdu/Workspace/librav/src/libraries/uavcan/libuavcan/test/dsdl_test/root_ns_a/128.EmptyService.uavcan
 */

#ifndef ROOT_NS_A_EMPTYSERVICE_HPP_INCLUDED
#define ROOT_NS_A_EMPTYSERVICE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
---
******************************************************************************/

/********************* DSDL signature source definition ***********************
root_ns_a.EmptyService
---
******************************************************************************/

namespace root_ns_a
{

struct UAVCAN_EXPORT EmptyService_
{
    template <int _tmpl>
    struct Request_
    {
        typedef const Request_<_tmpl>& ParameterType;
        typedef Request_<_tmpl>& ReferenceType;

        struct ConstantTypes
        {
        };

        struct FieldTypes
        {
        };

        enum
        {
            MinBitLen
        };

        enum
        {
            MaxBitLen
        };

        // Constants

        // Fields

        Request_()
        {
            ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

    #if UAVCAN_DEBUG
            /*
             * Cross-checking MaxBitLen provided by the DSDL compiler.
             * This check shall never be performed in user code because MaxBitLen value
             * actually depends on the nested types, thus it is not invariant.
             */
            ::uavcan::StaticAssert<0 == MaxBitLen>::check();
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

    };

    template <int _tmpl>
    struct Response_
    {
        typedef const Response_<_tmpl>& ParameterType;
        typedef Response_<_tmpl>& ReferenceType;

        struct ConstantTypes
        {
        };

        struct FieldTypes
        {
        };

        enum
        {
            MinBitLen
        };

        enum
        {
            MaxBitLen
        };

        // Constants

        // Fields

        Response_()
        {
            ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

    #if UAVCAN_DEBUG
            /*
             * Cross-checking MaxBitLen provided by the DSDL compiler.
             * This check shall never be performed in user code because MaxBitLen value
             * actually depends on the nested types, thus it is not invariant.
             */
            ::uavcan::StaticAssert<0 == MaxBitLen>::check();
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

    };

    typedef Request_<0> Request;
    typedef Response_<0> Response;

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindService };
    enum { DefaultDataTypeID = 128 };

    static const char* getDataTypeFullName()
    {
        return "root_ns_a.EmptyService";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

private:
    EmptyService_(); // Don't create objects of this type. Use Request/Response instead.
};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool EmptyService_::Request_<_tmpl>::operator==(ParameterType rhs) const
{
    (void)rhs;
    return true;
}

template <int _tmpl>
bool EmptyService_::Request_<_tmpl>::isClose(ParameterType rhs) const
{
    (void)rhs;
    return true;
}

template <int _tmpl>
int EmptyService_::Request_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    return res;
}

template <int _tmpl>
int EmptyService_::Request_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    return res;
}

template <int _tmpl>
bool EmptyService_::Response_<_tmpl>::operator==(ParameterType rhs) const
{
    (void)rhs;
    return true;
}

template <int _tmpl>
bool EmptyService_::Response_<_tmpl>::isClose(ParameterType rhs) const
{
    (void)rhs;
    return true;
}

template <int _tmpl>
int EmptyService_::Response_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    return res;
}

template <int _tmpl>
int EmptyService_::Response_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    return res;
}

/*
 * Out of line type method definitions
 */
inline ::uavcan::DataTypeSignature EmptyService_::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xE74617107A34AA9CULL);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef EmptyService_ EmptyService;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::root_ns_a::EmptyService > _uavcan_gdtr_registrator_EmptyService;

}

} // Namespace root_ns_a

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::root_ns_a::EmptyService::Request >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::root_ns_a::EmptyService::Request::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::root_ns_a::EmptyService::Request >::stream(Stream& s, ::root_ns_a::EmptyService::Request::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
}

template <>
class UAVCAN_EXPORT YamlStreamer< ::root_ns_a::EmptyService::Response >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::root_ns_a::EmptyService::Response::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::root_ns_a::EmptyService::Response >::stream(Stream& s, ::root_ns_a::EmptyService::Response::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
}

}

namespace root_ns_a
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::root_ns_a::EmptyService::Request::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::root_ns_a::EmptyService::Request >::stream(s, obj, 0);
    return s;
}

template <typename Stream>
inline Stream& operator<<(Stream& s, ::root_ns_a::EmptyService::Response::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::root_ns_a::EmptyService::Response >::stream(s, obj, 0);
    return s;
}

} // Namespace root_ns_a

#endif // ROOT_NS_A_EMPTYSERVICE_HPP_INCLUDED