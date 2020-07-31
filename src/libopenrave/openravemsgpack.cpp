// -*- coding: utf-8 -*-
// Copyright (C) 2006-2020 Rosen Diankov <rosen.diankov@gmail.com>
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "libopenrave.h"

#include <openrave/openravemsgpack.h>

#if OPENRAVE_MSGPACK
#include <msgpack.hpp>
#include <rapidjson/document.h>

namespace msgpack {

MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {

namespace adaptor {

template <typename Encoding, typename Allocator, typename StackAllocator>
struct convert< rapidjson::GenericDocument<Encoding, Allocator, StackAllocator> > {
    msgpack::object const& operator()(msgpack::object const& o, rapidjson::GenericDocument<Encoding, Allocator, StackAllocator>& v) const {
        switch (o.type)
        {
            case msgpack::type::BOOLEAN: v.SetBool(o.via.boolean); break;;
            case msgpack::type::POSITIVE_INTEGER: v.SetUint64(o.via.u64); break;
            case msgpack::type::NEGATIVE_INTEGER: v.SetInt64(o.via.i64); break;
            case msgpack::type::FLOAT: v.SetDouble(o.via.f64); break;
            case msgpack::type::BIN: // fall through
            case msgpack::type::STR: v.SetString(o.via.str.ptr, o.via.str.size, v.GetAllocator()); break;
            case msgpack::type::ARRAY:{
                v.SetArray();
                v.Reserve(o.via.array.size, v.GetAllocator());
                msgpack::object* ptr = o.via.array.ptr;
                msgpack::object* END = ptr + o.via.array.size;
                for (; ptr < END; ++ptr)
                {
                    rapidjson::GenericDocument<Encoding, Allocator, StackAllocator> element(&v.GetAllocator());
                    ptr->convert(&element);
                    v.PushBack(static_cast<rapidjson::GenericValue<Encoding, Allocator>&>(element), v.GetAllocator());
                }
            }
                break;
            case msgpack::type::MAP: {
                v.SetObject();
                msgpack::object_kv* ptr = o.via.map.ptr;
                msgpack::object_kv* END = ptr + o.via.map.size;
                for (; ptr < END; ++ptr)
                {
                    rapidjson::GenericValue<Encoding, Allocator> key(ptr->key.via.str.ptr, ptr->key.via.str.size, v.GetAllocator());
                    rapidjson::GenericDocument<Encoding, Allocator, StackAllocator> val(&v.GetAllocator());
                    ptr->val.convert(&val);

                    v.AddMember(key, val, v.GetAllocator());
                }
            }
                break;
            case msgpack::type::NIL:
            default:
                v.SetNull(); break;

        }
        return o;
    }
};

template <typename Encoding, typename Allocator>
struct convert< rapidjson::GenericValue<Encoding, Allocator> > {
    msgpack::object const& operator()(msgpack::object const& o, rapidjson::GenericValue<Encoding, Allocator>& v) const {
        rapidjson::GenericDocument<Encoding, Allocator> d;
        o >> d;
        return v = d;
    }
};

template <typename Encoding, typename Allocator>
struct pack< rapidjson::GenericValue<Encoding, Allocator> > {
    template <typename Stream>
    msgpack::packer<Stream>& operator()(msgpack::packer<Stream>& o, rapidjson::GenericValue<Encoding, Allocator> const& v) const {
        switch (v.GetType())
        {
            case rapidjson::kNullType:
                return o.pack_nil();
            case rapidjson::kFalseType:
                return o.pack_false();
            case rapidjson::kTrueType:
                return o.pack_true();
            case rapidjson::kObjectType:
            {
                o.pack_map(v.MemberCount());
                typename rapidjson::GenericValue<Encoding, Allocator>::ConstMemberIterator i = v.MemberBegin(), END = v.MemberEnd();
                for (; i != END; ++i)
                {
                    o.pack_str(i->name.GetStringLength()).pack_str_body(i->name.GetString(), i->name.GetStringLength());
                    o.pack(i->value);
                }
                return o;
            }
            case rapidjson::kArrayType:
            {
                o.pack_array(v.Size());
                typename rapidjson::GenericValue<Encoding, Allocator>::ConstValueIterator i = v.Begin(), END = v.End();
                for (;i < END; ++i)
                    o.pack(*i);
                return o;
            }
            case rapidjson::kStringType:
                return o.pack_str(v.GetStringLength()).pack_str_body(v.GetString(), v.GetStringLength());
            case rapidjson::kNumberType:
                if (v.IsInt())
                    return o.pack_int(v.GetInt());
                if (v.IsUint())
                    return o.pack_unsigned_int(v.GetUint());
                if (v.IsInt64())
                    return o.pack_int64(v.GetInt64());
                if (v.IsUint64())
                    return o.pack_uint64(v.GetUint64());
                if (v.IsDouble()||v.IsNumber())
                    return o.pack_double(v.GetDouble());
            default:
                return o;
        }
    }
};

template <typename Encoding, typename Allocator, typename StackAllocator>
struct pack< rapidjson::GenericDocument<Encoding, Allocator, StackAllocator> > {
    template <typename Stream>
    msgpack::packer<Stream>& operator()(msgpack::packer<Stream>& o, rapidjson::GenericDocument<Encoding, Allocator, StackAllocator> const& v) const {
        o << static_cast<const rapidjson::GenericValue<Encoding, Allocator>&>(v);
        return o;
    }
};

template <typename Encoding, typename Allocator>
struct object_with_zone< rapidjson::GenericValue<Encoding, Allocator> > {
    void operator()(msgpack::object::with_zone& o, rapidjson::GenericValue<Encoding, Allocator> const& v) const {
        switch (v.GetType())
        {
            case rapidjson::kNullType:
                o.type = type::NIL;
                break;
            case rapidjson::kFalseType:
                o.type = type::BOOLEAN;
                o.via.boolean = false;
                break;
            case rapidjson::kTrueType:
                o.type = type::BOOLEAN;
                o.via.boolean = true;
                break;
            case rapidjson::kObjectType:
            {
                o.type = type::MAP;
                if (v.ObjectEmpty()) {
                    o.via.map.ptr = NULL;
                    o.via.map.size = 0;
                }
                else {
                    size_t sz = v.MemberCount();
                    object_kv* p = (object_kv*)o.zone.allocate_align(sizeof(object_kv)*sz);
                    object_kv* const pend = p + sz;
                    o.via.map.ptr = p;
                    o.via.map.size = sz;
                    typename rapidjson::GenericValue<Encoding, Allocator>::ConstMemberIterator it(v.MemberBegin());
                    do {
                        p->key = msgpack::object(it->name, o.zone);
                        p->val = msgpack::object(it->value, o.zone);
                        ++p;
                        ++it;
                    } while (p < pend);
                }
                break;
            }
            case rapidjson::kArrayType:
            {
                o.type = type::ARRAY;
                if (v.Empty()) {
                    o.via.array.ptr = NULL;
                    o.via.array.size = 0;
                }
                else {
                    msgpack::object* p = (msgpack::object*)o.zone.allocate_align(sizeof(msgpack::object)*v.Size());
                    msgpack::object* const pend = p + v.Size();
                    o.via.array.ptr = p;
                    o.via.array.size = v.Size();
                    typename rapidjson::GenericValue<Encoding, Allocator>::ConstValueIterator it(v.Begin());
                    do {
                        *p = msgpack::object(*it, o.zone);
                        ++p;
                        ++it;
                    } while (p < pend);
                }
                break;
            }
            case rapidjson::kStringType:
            {
                o.type = type::STR;
                size_t size = v.GetStringLength();
                char* ptr = (char*)o.zone.allocate_align(size);
                memcpy(ptr, v.GetString(), size);
                o.via.str.ptr = ptr;
                o.via.str.size = size;
                break;
            }
            case rapidjson::kNumberType:
                if (v.IsInt())
                {
                    o.type = type::NEGATIVE_INTEGER;
                    o.via.i64 = v.GetInt();
                }
                else if (v.IsUint())
                {
                    o.type = type::POSITIVE_INTEGER;
                    o.via.u64 = v.GetUint();
                }
                else if (v.IsInt64())
                {
                    o.type = type::NEGATIVE_INTEGER;
                    o.via.i64 = v.GetInt64();
                }
                else if (v.IsUint64())
                {
                    o.type = type::POSITIVE_INTEGER;
                    o.via.u64 = v.GetUint64();
                }
                else if (v.IsDouble())
                {
                    o.type = type::FLOAT;
                    o.via.f64 = v.GetDouble();
                }
                break;
            default:
                break;

        }
    }
};

template <typename Encoding, typename Allocator, typename StackAllocator>
struct object_with_zone< rapidjson::GenericDocument<Encoding, Allocator, StackAllocator> > {
    void operator()(msgpack::object::with_zone& o, rapidjson::GenericDocument<Encoding, Allocator, StackAllocator> const& v) const {
        o << static_cast<rapidjson::GenericValue<Encoding, Allocator> const&>(v);
    }
};

} // namespace adaptor

class vbuffer {
public:
    vbuffer(std::vector<char>& v) : _v(v) { }
    void write(const char* buf, size_t len) {
        std::copy_n(buf, len, std::back_inserter(_v));
    }
private:
    std::vector<char>& _v;
};


class osbuffer {
public:
    osbuffer(std::ostream& os) : _os(os) { }
    void write(const char* buf, size_t len) {
        _os.write(buf, len);
    }
private:
    std::ostream& _os;
};

} // MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS)

} // namespace msgpack

void OpenRAVE::MsgPack::DumpMsgPack(const rapidjson::Value& value, std::ostream& os)
{
    msgpack::osbuffer buf(os);
    msgpack::pack(&buf, value);
}

void OpenRAVE::MsgPack::DumpMsgPack(const rapidjson::Value& value, std::vector<char>& output)
{   
    msgpack::vbuffer buf(output);
    msgpack::pack(&buf, value);
}

void OpenRAVE::MsgPack::ParseMsgPack(rapidjson::Document& d, const std::string& str)
{
    msgpack::unpacked unpacked;
    msgpack::unpack(&unpacked, str.data(), str.size());
    unpacked.get().convert(d);
}

void OpenRAVE::MsgPack::ParseMsgPack(rapidjson::Document& d, std::istream& is)
{
    std::string str;
    str.assign(std::istreambuf_iterator<char>(is), std::istreambuf_iterator<char>());
    OpenRAVE::MsgPack::ParseMsgPack(d, str);
}

#else

void OpenRAVE::MsgPack::DumpMsgPack(const rapidjson::Value& value, std::ostream& os)
{
    throw OPENRAVE_EXCEPTION_FORMAT0("MsgPack support is not enabled", ORE_NotImplemented);
}

void OpenRAVE::MsgPack::DumpMsgPack(const rapidjson::Value& value, std::vector<char>& output)
{
    throw OPENRAVE_EXCEPTION_FORMAT0("MsgPack support is not enabled", ORE_NotImplemented);
}

void OpenRAVE::MsgPack::ParseMsgPack(rapidjson::Document& d, const std::string& str)
{
    throw OPENRAVE_EXCEPTION_FORMAT0("MsgPack support is not enabled", ORE_NotImplemented);
}

void OpenRAVE::MsgPack::ParseMsgPack(rapidjson::Document& d, std::istream& is)
{
    throw OPENRAVE_EXCEPTION_FORMAT0("MsgPack support is not enabled", ORE_NotImplemented);
}

#endif
