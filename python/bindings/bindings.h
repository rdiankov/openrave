// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef OPENRAVE_BOOST_PYTHON_BINDINGS
#define OPENRAVE_BOOST_PYTHON_BINDINGS

#include <Python.h>
#include <boost/array.hpp>
#include <boost/multi_array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/format.hpp>
#include <boost/python.hpp>
#include <boost/assert.hpp>
#include <boost/cstdint.hpp>

#include <complex>
#include <algorithm>

using namespace boost::python;

namespace mydetail {
template<class T>
class numpy_type_map {
public:
  static const int typenum;
};

template<> const int numpy_type_map<float>::typenum = NPY_FLOAT;
template<> const int numpy_type_map<std::complex<float> >::typenum = NPY_CFLOAT;
template<> const int numpy_type_map<double>::typenum = NPY_DOUBLE;
template<> const int numpy_type_map<std::complex<double> >::typenum = NPY_CDOUBLE;
template<> const int numpy_type_map<long double>::typenum = NPY_LONGDOUBLE;
template<> const int numpy_type_map<std::complex<long double> >::typenum = NPY_CLONGDOUBLE;
template<> const int numpy_type_map<boost::int8_t>::typenum = NPY_INT8;
template<> const int numpy_type_map<boost::uint8_t>::typenum = NPY_UINT8;
template<> const int numpy_type_map<boost::int16_t>::typenum = NPY_INT16;
template<> const int numpy_type_map<boost::uint16_t>::typenum = NPY_UINT16;
template<> const int numpy_type_map<boost::int32_t>::typenum = NPY_INT32;
template<> const int numpy_type_map<boost::uint32_t>::typenum = NPY_UINT32;
template<> const int numpy_type_map<boost::int64_t>::typenum = NPY_INT64;
template<> const int numpy_type_map<boost::uint64_t>::typenum = NPY_INT64;

template< typename T >
struct get_dtype
{
	static const char * name() { throw std::logic_error( "get_dtype not specialised for this type" ); }
};

#define DECLARE_DTYPE_FOR( type, dtype ) template<> struct get_dtype<type> { static const char * name() { return dtype; } };

DECLARE_DTYPE_FOR( double, "float64" )
DECLARE_DTYPE_FOR( float, "float32" )
DECLARE_DTYPE_FOR( int, "int32" )
DECLARE_DTYPE_FOR( unsigned, "uint32" )
DECLARE_DTYPE_FOR( long, "int64" )
DECLARE_DTYPE_FOR( unsigned long, "uint64" )

}

// Copyright (c) 2008, Michael Droettboom All rights reserved.
// 
// numpy_boost class is Licensed under the BSD license.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
// 
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
// 
//     * The names of its contributors may not be used to endorse or
//       promote products derived from this software without specific
//       prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//template<class T, int NDims>
//class numpy_boost : public boost::multi_array_ref<T, NDims>
//{
//public:
//    typedef numpy_boost<T, NDims> self_type;
//    typedef boost::multi_array_ref<T, NDims> super;
//    typedef typename super::size_type size_type;
//    typedef T* TPtr;
//
//private:
//    PyArrayObject* array;
//
//    void init_from_array(PyArrayObject* a) {
//        array = a;
//        super::base_ = (TPtr)PyArray_DATA(a);
//
//        // It would seem like we would want to choose C or Fortran
//        // ordering here based on the flags in the Numpy array.  However,
//        // those flags are purely informational, the actually information
//        // about storage order is recorded in the strides.
//        super::storage_ = boost::c_storage_order();
//
//        boost::detail::multi_array::copy_n(PyArray_DIMS(a), NDims, super::extent_list_.begin());
//        for (size_t i = 0; i < NDims; ++i) {
//            super::stride_list_[i] = PyArray_STRIDE(a, i) / sizeof(T);
//        }
//        std::fill_n(super::index_base_list_.begin(), NDims, 0);
//        super::origin_offset_ = 0;
//        super::directional_offset_ = 0;
//        super::num_elements_ = std::accumulate(super::extent_list_.begin(), super::extent_list_.end(), size_type(1), std::multiplies<size_type>());
//    }
//
//public:
//    numpy_boost(PyObject* obj) :
//        super(NULL, std::vector<boost::uint32_t>(NDims, 0)),
//        array(NULL)
//    {
//        PyArrayObject* a;
//
//        a = (PyArrayObject*)PyArray_FromObject(obj, mydetail::numpy_type_map<T>::typenum, NDims, NDims);
//        BOOST_ASSERT(a!=NULL);
//        init_from_array(a);
//    }
//
//    numpy_boost(const self_type &other) :
//        super(NULL, std::vector<boost::uint32_t>(NDims, 0)),
//        array(NULL)
//    {
//        Py_INCREF(other.array);
//        init_from_array(other.array);
//    }
//
//    template<class ExtentsList>
//    explicit numpy_boost(const ExtentsList& extents) : super(NULL, std::vector<boost::uint32_t>(NDims, 0)), array(NULL)
//    {
//        npy_int shape[NDims];
//        PyArrayObject* a;
//        boost::detail::multi_array::copy_n(extents, NDims, shape);
//        a = (PyArrayObject*)PyArray_SimpleNew(NDims, shape, mydetail::numpy_type_map<T>::typenum);
//        BOOST_ASSERT(a!=NULL);
//        init_from_array(a);
//    }
//
//    virtual ~numpy_boost() {
//        Py_DECREF(array);
//    }
//
//    void operator=(const self_type &other) {
//        Py_DECREF(array);
//        Py_INCREF(other.array);
//        init_from_array(other.array);
//    }
//
//    PyObject* py_ptr() {
//        Py_INCREF(array);
//        return (PyObject*)array;
//    }
//};

// namespace impl
template< typename MultiArrayType >
struct numpy_multi_array_converter
{
     typedef MultiArrayType multi_array_t;
     typedef std::vector< std::size_t > shape_t;

	static void register_to_and_from_python()
	{
		register_from_python();
		register_to_python();
	}

	static void register_to_python()
	{
		to_python_converter< multi_array_t, numpy_multi_array_converter< multi_array_t> >();
	}

	static void register_from_python()
	{
		converter::registry::push_back(&convertible,&construct,type_id<multi_array_t>());
	}

    static void* convertible( PyObject * obj )
	{
		try {
			shape_t shape;
			get_shape( object( handle<>( borrowed( obj ) ) ), shape );
			if( multi_array_t::dimensionality != shape.size() ) return 0;
		}
		catch( ... ) {
			return 0;
		}

		return obj;
    }

	template< typename C, multi_array_t C::* pm >
    static void set_member_with_resize( C & c, const multi_array_t & a )
	{
		std::vector< unsigned > extents;
		for( unsigned dim = 0; a.num_dimensions() != dim; ++dim )
            extents.push_back( a.shape()[ dim ] );
		(c.*pm).resize( extents );
		(c.*pm) = a;
	}
    
    static void construct(PyObject* obj, converter::rvalue_from_python_stage1_data* data )
	{
		//get the storage
        typedef converter::rvalue_from_python_storage< multi_array_t > storage_t;
        storage_t * the_storage = reinterpret_cast< storage_t * >( data );
        void * memory_chunk = the_storage->storage.bytes;
        //new (memory_chunk) multi_array_t(obj);

		//new placement
		object py_obj( handle<>( borrowed( obj ) ) );
		shape_t shape;
		get_shape( py_obj, shape );
		multi_array_t * a = new (memory_chunk) multi_array_t( shape );
        
		//extract each element from numpy array and put in c array
		index i( a->num_dimensions(), 0 );
		do {
			boost::python::list numpy_index;
			for( unsigned dim = 0; a->num_dimensions() != dim; ++dim )
                numpy_index.append( i[ dim ] );
			(*a)(i) = extract<typename multi_array_t::element>(py_obj[ boost::python::tuple( numpy_index ) ]);
		}
		while( increment_index( i, *a ) );
        
        data->convertible = memory_chunk;
    }
    
	static PyObject* convert( const multi_array_t & c_array )
	{
		object numpy = object( handle<>(::PyImport_Import(object("numpy").ptr())));
		if( ! numpy  ) throw std::logic_error( "Could not import numpy" );
		object array_function = numpy.attr("empty");
		if( ! array_function  ) throw std::logic_error( "Could not find array function" );

		//create a numpy array to put it in
		boost::python::list extents;
		for( unsigned dim = 0; c_array.num_dimensions() != dim; ++dim ) 
extents.append( c_array.shape()[ dim ] );

		object result(array_function( extents,numpy.attr( "dtype" )( mydetail::get_dtype<typename multi_array_t::element>::name())));

		//copy the elements	
		index i( c_array.num_dimensions(), 0 );
		do {
			boost::python::list numpy_index;
			for( unsigned dim = 0; c_array.num_dimensions() != dim; ++dim )
                numpy_index.append(i[dim]);
			result[ tuple( numpy_index ) ] = c_array( i );
		} while( increment_index( i, c_array ) );

		return incref( result.ptr() );
	}

protected:
	static void get_shape(object obj, shape_t& shape)
	{
		shape.clear();
		object py_shape = obj.attr("shape");
		const std::size_t N = len( py_shape );
		for( std::size_t i = 0; N != i; ++i )
            shape.push_back( extract<std::size_t >(py_shape[i]));
	}

    /// o iterate over entries in num_dimensions independent fashion.
	typedef std::vector< typename multi_array_t::index > index;

	/// Iterates over entries in num_dimensions independent fashion
	static bool increment_index( index & i, const multi_array_t & c_array )
	{
		for( size_t dim = 0; i.size() != dim; ++dim ) {
			++i[dim];
			if( i[dim] != static_cast<typename multi_array_t::index>(c_array.shape()[dim]) )
                return true;
			else
                i[dim] = 0;
		}
		return false;
	}
};

// register const versions of the classes
//template <class T> inline T* get_pointer( boost::shared_ptr<const T> 
//const& p){
//     return const_cast<T*>(p.get());
//}
//
//template <class T> struct pintee< boost::shared_ptr<const T> >{
//     typedef T type;
//};
//
//boost::python::register_ptr_to_python< boost::shared_ptr<const my_class> >();

struct int_from_int
{
    int_from_int()
    {
        converter::registry::push_back(&convertible, &construct, type_id<int>());
    }

    static void* convertible( PyObject* obj)
    {
        PyObject* newobj = PyNumber_Int(obj);
        if (!PyString_Check(obj) && newobj) {
            Py_DECREF(newobj);
            return obj;
        }
        else {
            if (newobj) {
                Py_DECREF(newobj);
            }
            PyErr_Clear();
            return 0;
        }
    }

    static void construct(PyObject* _obj, converter::rvalue_from_python_stage1_data* data)
    {
        PyObject* newobj = PyNumber_Int(_obj);
        int* storage = (int*)((converter::rvalue_from_python_storage<int>*)data)->storage.bytes;
        *storage = extract<int>(newobj);
        Py_DECREF(newobj);
        data->convertible = storage;
    }
};

template<typename T>
struct T_from_number
{
    T_from_number()
    {
        converter::registry::push_back(&convertible, &construct, type_id<T>());
    }

    static void* convertible( PyObject* obj)
    {
        PyObject* newobj = PyNumber_Float(obj);
        if (!PyString_Check(obj) && newobj) {
            Py_DECREF(newobj);
            return obj;
        }
        else {
            if (newobj) {
                Py_DECREF(newobj);
            }
            PyErr_Clear();
            return 0;
        }
    }

    static void construct(PyObject* _obj, converter::rvalue_from_python_stage1_data* data)
    {
        PyObject* newobj = PyNumber_Float(_obj);
        T* storage = (T*)((converter::rvalue_from_python_storage<T>*)data)->storage.bytes;
        *storage = extract<T>(newobj);
        Py_DECREF(newobj);
        data->convertible = storage;
    }
};

#endif
