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
#define BOOST_ENABLE_ASSERT_HANDLER
#include "bindings.h"

namespace mydetail {
template< typename T >
struct get_dtype
{
    static const char * name() {
        throw std::logic_error( "get_dtype not specialised for this type" );
    }
};

#define DECLARE_DTYPE_FOR( type, dtype ) template<> struct get_dtype<type> { static const char * name() { return dtype; } };

DECLARE_DTYPE_FOR( double, "float64" )
DECLARE_DTYPE_FOR( float, "float32" )
DECLARE_DTYPE_FOR( int, "int32" )
DECLARE_DTYPE_FOR( uint32_t, "uint32" )
DECLARE_DTYPE_FOR( int64_t, "int64" )
DECLARE_DTYPE_FOR( uint64_t, "uint64" )

}

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
        catch(... ) {
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
        if( !numpy  ) throw std::logic_error( "Could not import numpy" );
        object array_function = numpy.attr("empty");
        if( !array_function  ) throw std::logic_error( "Could not find array function" );

        //create a numpy array to put it in
        boost::python::list extents;
        for( unsigned dim = 0; c_array.num_dimensions() != dim; ++dim )
            extents.append( c_array.shape()[ dim ] );

        object result(array_function( extents,numpy.attr( "dtype" ) ( mydetail::get_dtype<typename multi_array_t::element>::name())));

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

void init_python_bindings()
{
    numpy_multi_array_converter< boost::multi_array<float,1> >::register_to_and_from_python();
    numpy_multi_array_converter< boost::multi_array<float,2> >::register_to_and_from_python();
    numpy_multi_array_converter< boost::multi_array<float,3> >::register_to_and_from_python();
    numpy_multi_array_converter< boost::multi_array<double,1> >::register_to_and_from_python();
    numpy_multi_array_converter< boost::multi_array<double,2> >::register_to_and_from_python();
    numpy_multi_array_converter< boost::multi_array<double,3> >::register_to_and_from_python();
    numpy_multi_array_converter< boost::multi_array<int,1> >::register_to_and_from_python();
    numpy_multi_array_converter< boost::multi_array<int,2> >::register_to_and_from_python();

    class_<PyVoidHandle, boost::shared_ptr<PyVoidHandle> >("VoidHandle")
    .def("close",&PyVoidHandle::close)
    ;
}
