# -*- coding: utf-8 -*-


cmake_minimum_required(VERSION 3.20)  # For CMP0118

project(openravepytypestubs)

message(STATUS "Using cmake version ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}")

# ---

find_package(OpenRAVE 0.151.0 REQUIRED)

if (OpenRAVE_PYTHON3)
    message(STATUS "Found openrave's python3 dir: ${OpenRAVE_PYTHON3_DIR}")
    find_package(Python3 3.9 REQUIRED COMPONENTS Interpreter Development NumPy)
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import distutils.sysconfig as sysconf; import os; \
            print(os.path.relpath(sysconf.get_python_lib(1, prefix='${CMAKE_INSTALL_PREFIX}'), '${CMAKE_INSTALL_PREFIX}'))"
        OUTPUT_VARIABLE OPENRAVE_PYTHON3_INSTALL_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    )
endif()

# ----

if("${CMAKE_VERSION}" VERSION_GREATER_EQUAL "3.26")
    set(COPY_COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different)
else()
    set(COPY_COMMAND cp -rp)
endif()

find_program(PYBIND11_STUBGEN pybind11-stubgen)
find_file(OPENRAVEPY_INT_SO_FILE openravepy_int.so PATHS ${CMAKE_INSTALL_PREFIX}/${OPENRAVE_PYTHON3_INSTALL_DIR}/openravepy/_openravepy_)
if (PYBIND11_STUBGEN)
    # Openravecommon stubs
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/__init__.pyi
        COMMAND bash -c "${PYBIND11_STUBGEN} openravepy -o ${CMAKE_CURRENT_BINARY_DIR}/python"
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_
        COMMAND ${COPY_COMMAND} ${CMAKE_CURRENT_BINARY_DIR}/python/_openravepy_-stubs/. ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/
        COMMAND ${CMAKE_COMMAND} -E echo "from ._openravepy_ import *" > ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/__init__.pyi
        COMMAND ${CMAKE_COMMAND} -E echo "from ._openravepy_ import __author__ as __author__" >> ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/__init__.pyi
        COMMAND ${CMAKE_COMMAND} -E echo "from ._openravepy_ import __copyright__ as __copyright__" >> ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/__init__.pyi
        COMMAND ${CMAKE_COMMAND} -E echo "from ._openravepy_ import __docformat__ as __docformat__" >> ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/__init__.pyi
        COMMAND ${CMAKE_COMMAND} -E echo "from ._openravepy_ import __license__ as __license__" >> ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/__init__.pyi
        COMMAND ${CMAKE_COMMAND} -E echo "from ._openravepy_ import __version__ as __version__" >> ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/__init__.pyi
        COMMAND ${CMAKE_COMMAND} -E echo "from ._openravepy_.misc import *" > ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/misc.pyi
        COMMAND ${CMAKE_COMMAND} -E echo "from ._openravepy_.openrave_ext import *" > ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/openravepy_ext.pyi
        COMMAND ${CMAKE_COMMAND} -E echo "from ._openravepy_.openrave_int import *" > ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/openravepy_int.pyi
        COMMAND ${Python3_EXECUTABLE} -c [=[from mypy.stubgen import main\; main\(\)]=] -o ${CMAKE_CURRENT_BINARY_DIR}/python -m openravepy.misc
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy/misc.pyi ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/misc/__init__.pyi
        COMMAND ${Python3_EXECUTABLE} -c [=[from mypy.stubgen import main\; main\(\)]=] -o ${CMAKE_CURRENT_BINARY_DIR}/python -m openravepy.openravepy_ext
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy/openravepy_ext.pyi ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/openravepy_ext/__init__.pyi
        COMMAND ${Python3_EXECUTABLE} -c [=[from mypy.stubgen import main\; main\(\)]=] -o ${CMAKE_CURRENT_BINARY_DIR}/python -m openravepy.databases.convexdecomposition
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy/databases/convexdecomposition.pyi ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/databases/convexdecomposition/__init__.pyi
        COMMAND ${Python3_EXECUTABLE} -c [=[from mypy.stubgen import main\; main\(\)]=] -o ${CMAKE_CURRENT_BINARY_DIR}/python -m openravepy.databases.grasping
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy/databases/grasping.pyi ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/databases/grasping/__init__.pyi
        COMMAND ${Python3_EXECUTABLE} -c [=[from mypy.stubgen import main\; main\(\)]=] -o ${CMAKE_CURRENT_BINARY_DIR}/python -m openravepy.databases.inversekinematics
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy/databases/inversekinematics.pyi ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/databases/inversekinematics/__init__.pyi
        COMMAND ${Python3_EXECUTABLE} -c [=[from mypy.stubgen import main\; main\(\)]=] -o ${CMAKE_CURRENT_BINARY_DIR}/python -m openravepy.databases.inversereachability
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy/databases/inversereachability.pyi ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/databases/inversereachability/__init__.pyi
        COMMAND ${Python3_EXECUTABLE} -c [=[from mypy.stubgen import main\; main\(\)]=] -o ${CMAKE_CURRENT_BINARY_DIR}/python -m openravepy.databases.kinematicreachability
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy/databases/kinematicreachability.pyi ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/databases/kinematicreachability/__init__.pyi
        COMMAND ${Python3_EXECUTABLE} -c [=[from mypy.stubgen import main\; main\(\)]=] -o ${CMAKE_CURRENT_BINARY_DIR}/python -m openravepy.databases.linkstatistics
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy/databases/linkstatistics.pyi ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/databases/linkstatistics/__init__.pyi
        COMMAND ${Python3_EXECUTABLE} -c [=[from mypy.stubgen import main\; main\(\)]=] -o ${CMAKE_CURRENT_BINARY_DIR}/python -m openravepy.databases.visibilitymodel
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy/databases/visibilitymodel.pyi ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/databases/visibilitymodel/__init__.pyi
        # COMMAND ${Python3_EXECUTABLE} -c [=[from mypy.stubgen import main\; main\(\)]=] -o ${CMAKE_CURRENT_BINARY_DIR}/python -m openravepy.interfaces.BaseManipulation
        # COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy/interfaces/BaseManipulation.pyi ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/interfaces/BaseManipulation/__init__.pyi
        # COMMAND ${Python3_EXECUTABLE} -c [=[from mypy.stubgen import main\; main\(\)]=] -o ${CMAKE_CURRENT_BINARY_DIR}/python -m openravepy.interfaces.Grasper
        # COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy/interfaces/Grasper.pyi ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/interfaces/Grasper/__init__.pyi
        # COMMAND ${Python3_EXECUTABLE} -c [=[from mypy.stubgen import main\; main\(\)]=] -o ${CMAKE_CURRENT_BINARY_DIR}/python -m openravepy.interfaces.TaskManipulation
        # COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy/interfaces/TaskManipulation.pyi ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/interfaces/TaskManipulation/__init__.pyi
        # COMMAND ${Python3_EXECUTABLE} -c [=[from mypy.stubgen import main\; main\(\)]=] -o ${CMAKE_CURRENT_BINARY_DIR}/python -m openravepy.interfaces.visualfeedback
        # COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy/interfaces/visualfeedback.pyi ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/interfaces/visualfeedback/__init__.pyi
        DEPENDS ${OPENRAVEPY_INT_SO_FILE}
    )
    add_custom_target(openravepy_int_stubs ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs/_openravepy_/__init__.pyi)
    install(
        DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/python/openravepy-stubs
        DESTINATION ${OPENRAVE_PYTHON3_INSTALL_DIR}
        FILES_MATCHING PATTERN "*.pyi"
    )
else ()
    message(WARNING "Could not find pybind11-stubgen.")
endif (PYBIND11_STUBGEN)
