// -*- coding: utf-8 -*-
// Copyright (C) 2012-2014 Rosen Diankov <rosen.diankov@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/** \brief  Header file for all ikfast c++ files/shared objects.

    The ikfast inverse kinematics compiler is part of OpenRAVE.

    The file is divided into two sections:
    - <b>Common</b> - the abstract classes section that all ikfast share regardless of their settings
    - <b>Library Specific</b> - the library-specific definitions, which depends on the precision/settings that the library was compiled with

    The defines are as follows, they are also used for the ikfast C++ class:

   - IKFAST_HEADER_COMMON - common classes
   - IKFAST_HAS_LIBRARY - if defined, will include library-specific functions. by default this is off
   - IKFAST_CLIBRARY - Define this linking statically or dynamically to get correct visibility.
   - IKFAST_NO_MAIN - Remove the ``main`` function, usually used with IKFAST_CLIBRARY
   - IKFAST_ASSERT - Define in order to get a custom assert called when NaNs, divides by zero, and other invalid conditions are detected.
   - IKFAST_REAL - Use to force a custom real number type for IkReal.
   - IKFAST_NAMESPACE - Enclose all functions and classes in this namespace, the ``main`` function is excluded.

 */
#include <algorithm>
#include <array>
#include <vector>
#include <list>
#include <stdexcept>
#include <cmath>
#include <iostream>
#include <iomanip>

#ifndef IKFAST_HEADER_COMMON
#define IKFAST_HEADER_COMMON

/// should be the same as ikfast.__version__
/// if 0x10000000 bit is set, then the iksolver assumes 6D transforms are done without the manipulator offset taken into account (allows to reuse IK when manipulator offset changes)
#define IKFAST_VERSION 0x1000004c

#define IKSINGLEDOFSOLUTIONBASE_INDICES_SIZE 5

namespace ikfast {

/// \brief holds the solution for a single dof
template <typename T>
class IkSingleDOFSolutionBase
{
public:
    IkSingleDOFSolutionBase() {
        indices.fill(0xFF);
    }

    T fmul = 0.0, foffset = 0.0; ///< joint value is fmul*sol[freeind]+foffset
    signed char freeind = -1; ///< if >= 0, mimics another joint
    unsigned char jointtype = 0x01; ///< joint type, 0x01 is revolute, 0x11 is slider
    unsigned char maxsolutions = 0; ///< max possible indices, 0 if controlled by free index or a free joint itself
    std::array<unsigned char, IKSINGLEDOFSOLUTIONBASE_INDICES_SIZE> indices; ///< unique index of the solution used to keep track on what part it came from. sometimes a solution can be repeated for different indices. store at least another repeated root

    virtual void Print() const {
        std::cout << "(" << ((jointtype == 0x01) ? "R" : "P") << ", "
                  << (int)freeind << "), (" << foffset << ", "
                  << fmul << "), " << (unsigned int) maxsolutions << " (";
        for(unsigned int i = 0; i < indices.size(); i++) {
            std::cout << (unsigned int) indices[i] << ", ";
        }
        std::cout << ") " << std::endl;
    }
};

/// \brief The discrete solutions are returned in this structure.
///
/// Sometimes the joint axes of the robot can align allowing an infinite number of solutions.
/// Stores all these solutions in the form of free variables that the user has to set when querying the solution. Its prototype is:
template <typename T>
class IkSolutionBase
{
public:
    virtual ~IkSolutionBase() {
    }
    /// \brief gets a concrete solution
    ///
    /// \param[out] solution the result
    /// \param[in] freevalues values for the free parameters \se GetFree
    virtual void GetSolution(T* solution, const T* freevalues) const = 0;

    /// \brief std::vector version of \ref GetSolution
    virtual void GetSolution(std::vector<T>& solution, const std::vector<T>& freevalues) const {
        solution.resize(GetDOF());
        GetSolution(&solution.at(0), freevalues.size() > 0 ? &freevalues.at(0) : NULL);
    }

    /// \brief Gets the indices of the configuration space that have to be preset before a full solution can be returned
    ///
    /// 0 always points to the first value accepted by the ik function.
    /// \return vector of indices indicating the free parameters
    virtual const std::vector<int>& GetFree() const = 0;

    /// \brief the dof of the solution
    virtual const int GetDOF() const = 0;
};

/// \brief manages all the solutions
template <typename T>
class IkSolutionListBase
{
public:
    virtual ~IkSolutionListBase() {
    }

    /// \brief add one solution and return its index for later retrieval
    ///
    /// \param vinfos Solution data for each degree of freedom of the manipulator
    /// \param vfree If the solution represents an infinite space, holds free parameters of the solution that users can freely set. The indices are of the configuration that the IK solver accepts rather than the entire robot, ie 0 points to the first value accepted.
    virtual size_t AddSolution(const std::vector<IkSingleDOFSolutionBase<T> >& vinfos, const std::vector<int>& vfree) = 0;

    /// \brief returns the solution pointer
    virtual const IkSolutionBase<T>& GetSolution(size_t index) const = 0;

    /// \brief returns the number of solutions stored
    virtual size_t GetNumSolutions() const = 0;

    /// \brief clears all current solutions, note that any memory addresses returned from \ref GetSolution will be invalidated.
    virtual void Clear() = 0;
    virtual void Print() const = 0;
};

/// \brief holds function pointers for all the exported functions of ikfast
template <typename T>
class IkFastFunctions
{
public:
    IkFastFunctions() : _ComputeIk(NULL), _ComputeIk2(NULL), _ComputeFk(NULL), _GetNumFreeParameters(NULL), _GetFreeIndices(NULL), _GetNumJoints(NULL), _GetIkRealSize(NULL), _GetIkFastVersion(NULL), _GetIkType(NULL), _GetKinematicsHash(NULL) {
    }
    virtual ~IkFastFunctions() {
    }
    typedef bool (*ComputeIkFn)(const T*, const T*, const T*, IkSolutionListBase<T>&);
    ComputeIkFn _ComputeIk;
    typedef bool (*ComputeIk2Fn)(const T*, const T*, const T*, IkSolutionListBase<T>&, void*);
    ComputeIk2Fn _ComputeIk2;
    typedef void (*ComputeFkFn)(const T*, T*, T*);
    ComputeFkFn _ComputeFk;
    typedef int (*GetNumFreeParametersFn)();
    GetNumFreeParametersFn _GetNumFreeParameters;
    typedef const int* (*GetFreeIndicesFn)();
    GetFreeIndicesFn _GetFreeIndices;
    typedef int (*GetNumJointsFn)();
    GetNumJointsFn _GetNumJoints;
    typedef int (*GetIkRealSizeFn)();
    GetIkRealSizeFn _GetIkRealSize;
    typedef const char* (*GetIkFastVersionFn)();
    GetIkFastVersionFn _GetIkFastVersion;
    typedef int (*GetIkTypeFn)();
    GetIkTypeFn _GetIkType;
    typedef const char* (*GetKinematicsHashFn)();
    GetKinematicsHashFn _GetKinematicsHash;
};

template <typename T>
inline void ikfastfmodtwopi(T& c) {
    // put back to (-PI, PI]
    while (c > 3.1415926535897932384626) {
        c -= 6.2831853071795864769252;
    }
    while (c <= -3.1415926535897932384626) {
        c += 6.2831853071795864769252;
    }
}

// Implementations of the abstract classes, user doesn't need to use them

/// \brief Default implementation of \ref IkSolutionBase
template <typename T>
class IkSolution : public IkSolutionBase<T>
{
public:
    IkSolution(const std::vector<IkSingleDOFSolutionBase<T> >& vinfos, const std::vector<int>& vfree) {
        _vbasesol = vinfos;
        _vfree = vfree;
    }

    IkSolution() {
    }

    // IkSolution(const std::vector<T>& v, uint32_t nvars) {
    //   this->SetSolution(v, nvars);
    // }

    void SetSolution(const T v[], uint32_t nvars) {
        _vbasesol.clear();
        _vbasesol.resize(nvars);
        for(uint32_t i = 0; i < nvars; i++) {
            _vbasesol[i].foffset = v[i];
        }
    }

    virtual void GetSolution(T* solution, const T* freevalues) const {
        for(std::size_t i = 0; i < _vbasesol.size(); ++i) {
            if( _vbasesol[i].freeind < 0 )
                solution[i] = _vbasesol[i].foffset;
            else {
                solution[i] = freevalues[_vbasesol[i].freeind]*_vbasesol[i].fmul + _vbasesol[i].foffset;
                ikfastfmodtwopi(solution[i]);
            }
        }
    }

    virtual void GetSolution(std::vector<T>& solution, const std::vector<T>& freevalues) const {
        solution.resize(GetDOF());
        GetSolution(&solution.at(0), freevalues.size() > 0 ? &freevalues.at(0) : NULL);
    }

    virtual const std::vector<int>& GetFree() const {
        return _vfree;
    }
    virtual const int GetDOF() const {
        return static_cast<int>(_vbasesol.size());
    }

    virtual void Validate() const {
        for(size_t i = 0; i < _vbasesol.size(); ++i) {
            if( _vbasesol[i].maxsolutions == (unsigned char)-1) {
                throw std::runtime_error("max solutions for joint " + std::to_string(i) + "not initialized");
            }
            if( _vbasesol[i].maxsolutions > 0 ) {
                if( _vbasesol[i].indices[0] >= _vbasesol[i].maxsolutions ) {
                    throw std::runtime_error("index >= max solutions for joint " + std::to_string(i));
                }
                if( _vbasesol[i].indices[1] != (unsigned char)-1 && _vbasesol[i].indices[1] >= _vbasesol[i].maxsolutions ) {
                    throw std::runtime_error("2nd index >= max solutions for joint " + std::to_string(i));
                }
            }
            if( !std::isfinite(_vbasesol[i].foffset) ) {
                throw std::runtime_error("foffset for joint " + std::to_string(i) + " was not finite");
            }
        }
    }

    virtual void GetSolutionIndices(std::vector<unsigned int>& v) const {
        v.resize(0);
        v.push_back(0);
        for(int i = (int)_vbasesol.size()-1; i >= 0; --i) {
            if( _vbasesol[i].maxsolutions != (unsigned char)-1 && _vbasesol[i].maxsolutions > 1 ) {
                for(size_t j = 0; j < v.size(); ++j) {
                    v[j] *= _vbasesol[i].maxsolutions;
                }
                size_t orgsize=v.size();
                if( _vbasesol[i].indices[1] != (unsigned char)-1 ) {
                    for(size_t j = 0; j < orgsize; ++j) {
                        v.push_back(v[j]+_vbasesol[i].indices[1]);
                    }
                }
                if( _vbasesol[i].indices[0] != (unsigned char)-1 ) {
                    for(size_t j = 0; j < orgsize; ++j) {
                        v[j] += _vbasesol[i].indices[0];
                    }
                }
            }
        }
    }

    IkSingleDOFSolutionBase<T>& operator[](size_t i) {
        // assert(i < _vbasesol.size());
        return _vbasesol[i];
    }

    const IkSingleDOFSolutionBase<T>& get(size_t i) const {
        // assert(i < _vbasesol.size());
        return _vbasesol[i];
    }

    void SetFree(std::vector<int> vfree) {
        _vfree = std::move(vfree);
    }

    bool HasFreeIndices() const {
        return !_vfree.empty();
    }

    void ResetFreeIndices() {
        for (size_t i = 0; i < _vbasesol.size(); ++i) {
            _vbasesol[i].freeind = _vbasesol[i].indices[4] = -1;
        }
    }

    virtual void Print() const {
        std::cout << std::setprecision(16);
        for (size_t i = 0; i < _vbasesol.size(); ++i) {
            std::cout << i << ": ";
            _vbasesol[i].Print();
        }
        if(!_vfree.empty())
        {
            std::cout << "vfree = ";
            for (size_t i = 0; i < _vfree.size(); ++i) {
                std::cout << _vfree[i] << ", ";
            }
            std::cout << std::endl;
        }
    }

    std::vector< IkSingleDOFSolutionBase<T> > _vbasesol;       ///< solution and their offsets if joints are mimiced
    std::vector<int> _vfree;
};

/// \brief Default implementation of \ref IkSolutionListBase
template <typename T>
class IkSolutionList : public IkSolutionListBase<T>
{
public:
    virtual size_t AddSolution(const std::vector<IkSingleDOFSolutionBase<T> >& vinfos, const std::vector<int>& vfree)
    {
        size_t index = _listsolutions.size();
        _listsolutions.push_back(IkSolution<T>(vinfos,vfree));
        return index;
    }

    virtual const IkSolutionBase<T>& GetSolution(size_t index) const
    {
        if( index >= _listsolutions.size() ) {
            throw std::runtime_error("GetSolution index is invalid");
        }
        typename std::list< IkSolution<T> >::const_iterator it = _listsolutions.begin();
        std::advance(it,index);
        return *it;
    }

    virtual size_t GetNumSolutions() const {
        return _listsolutions.size();
    }

    virtual void Clear() {
        _listsolutions.clear();
    }

    IkSolution<T>& operator[](size_t i) {
        // assert(i < _listsolutions.size());
        return _listsolutions[i];
    }

    void SetSolutions(std::vector<IkSolution<T> > &vecsols) {
        _listsolutions.clear();
        std::move( std::make_move_iterator(vecsols.begin()),
                   std::make_move_iterator(vecsols.end()),
                   std::back_inserter(_listsolutions)
                   );
    }

    virtual void Print() const {
        unsigned int i = 0;
        for (typename std::list<IkSolution<T> >::const_iterator it=_listsolutions.cbegin();
             it != _listsolutions.cend(); ++it) {
            std::cout << "Solution " << i++ << ":" << std::endl;
            std::cout << "===========" << std::endl;
            it->Print();
        }
    }

protected:
    std::list< IkSolution<T> > _listsolutions;
};

/// \brief Contains information of a solution where two axes align.
///
/// \param freejoint  Index of the  free joint jy in SolutionArray = std::array<T, N>.
/// \param mimicjoint Index of the mimic joint jx in SolutionArray.
/// \param solutionindex Index of the IK solution in std::vector<SolutionArray>.
///        This is NOT the solution index associated with each joint in one IK solution.
/// \param bxpy If true, then jx + jy is the constant jxpy; otherwise jx - jy is the constant jxmy.
///        We store this constant in the foffset field of jx, and store 0 in that of jy.
///
/// In an aligned case, jy = c = 0.0 + 1.0*c, and
/// jx = jxpy - c = jxpy + (-1.0) * c (when bxpy is true), or
/// jx = jxmy + c = jxmy +   1.0  * c (when bxpy is false).

struct AlignedSolution {
    uint32_t freejoint = 0;     // jy
    uint32_t mimicjoint = 0;    // jx
    uint32_t solutionindex = 0;
    bool bxpy = true;           // jxpy = jx + jy
    // false means                 jxmy = jx - jy

    AlignedSolution(uint32_t freejoint_in,
                    uint32_t mimicjoint_in,
                    uint32_t solutionindex_in,
                    bool bxpy_in) {
        freejoint     = freejoint_in;
        mimicjoint    = mimicjoint_in;
        solutionindex = solutionindex_in;
        bxpy          = bxpy_in;
    }

    template <typename T>
    void SetIkSolution(ikfast::IkSolution<T>& solnobj, const T v[]) {
        const uint32_t dof = solnobj.GetDOF();
        // assert(freejoint < dof && mimicjoint < dof);
        ikfast::IkSingleDOFSolutionBase<T>& freejointsoln = solnobj[freejoint],
        &mimicjointsoln = solnobj[mimicjoint];

        // update _vbasesol
        freejointsoln.fmul = 1.0;
        freejointsoln.foffset = 0.0;
        freejointsoln.freeind = 0;
        freejointsoln.maxsolutions = 0;
        freejointsoln.indices[0] = (unsigned char) -1;

        mimicjointsoln.fmul = bxpy ? (-1.0) : (1.0);
        mimicjointsoln.foffset = v[mimicjoint] - v[freejoint] * mimicjointsoln.fmul;
        mimicjointsoln.freeind = 0;
        mimicjointsoln.maxsolutions = 0;
        mimicjointsoln.indices[0] = (unsigned char) -1;

        // update _vfree
        solnobj.SetFree({(int) freejoint});
    }
};

template <typename T, long unsigned int N>
struct IndicesCompare
{
    IndicesCompare(
        const std::vector<ikfast::IkSolution<T> >& vecsols,
        const std::array<uint32_t, N>& jointorder)
    : _vecsols(vecsols),
      _jointorder(jointorder) {}

    bool operator ()(uint32_t inda, uint32_t indb)
    {
        const ikfast::IkSolution<T> &sola = _vecsols[inda],
                                    &solb = _vecsols[indb];
        for (size_t i = 0; i < _jointorder.size(); ++i) {
            const T x = sola.get(_jointorder[i]).foffset,
                    y = solb.get(_jointorder[i]).foffset;
            if (x != y) { return x < y; }
        }
        return false;
    }

    const std::vector<ikfast::IkSolution<T> >& _vecsols;
    const std::array<uint32_t, N>& _jointorder;
};

template <typename T, long unsigned int N>
void DeriveSolutionIndices(std::vector<ikfast::IkSolution<T> >& vecsols,
                           const std::array<uint32_t, N>& jointorder) {
    if( vecsols.empty() ) {
        return;
    }
    const uint32_t nallvars = vecsols[0].GetDOF(), numsolns = vecsols.size();
    // assert(N <= nallvars);
    // for(ikfast::IkSolution<T>& vecsol : vecsols) {
    //   assert(vecsol.GetDOF() == nallvars);
    // }

    std::vector<uint32_t> vindices(numsolns);
    for (uint32_t i = 0; i < numsolns; i++) {
        vindices[i] = i;
    }

    std::sort(vindices.begin(), vindices.end(), IndicesCompare<T, N>(vecsols, jointorder));

    // initialize
    for (uint32_t i = 0; i < N; i++) {
        vecsols[vindices[0]][jointorder[i]].indices[0] = (unsigned char) 0;
    }
    std::array<uint32_t, N> count;
    count.fill(0);

    // derive solution indices for each joint and maxsolutions
    for(uint32_t si = 1; si < numsolns; si++) {
        bool index_determined = false;
        const ikfast::IkSolution<T> &prevsol = vecsols[vindices[si-1]];
        ikfast::IkSolution<T> &cursol = vecsols[vindices[si]];

        for (uint32_t varindex = 0; varindex < N; varindex++) {
            const uint32_t jointindex = jointorder[varindex];
            const uint32_t lastindex = prevsol.get(jointindex).indices[0];

            if( index_determined ) {
                const uint32_t oldsi = count[varindex], maxsolni = lastindex + 1;
                count[varindex] = si;
                cursol[jointindex].indices[0] = (unsigned char) 0;
                for(uint32_t sii = oldsi; sii < si; sii++) {
                    vecsols[vindices[sii]][jointindex].maxsolutions = (unsigned char) maxsolni;
                }
            }
            else if ( cursol[jointindex].foffset > prevsol.get(jointindex).foffset ) {
                cursol[jointindex].indices[0] = (unsigned char) (lastindex + 1);
                index_determined = true;
            }
            else {
                cursol[jointindex].indices[0] = (unsigned char) lastindex;
            }
        }
    }

    // last round of deriving maxsolutions
    const ikfast::IkSolution<T> &cursol = vecsols[vindices[numsolns-1]];
    for (uint32_t varindex = 0; varindex < N; varindex++) {
        const uint32_t jointindex = jointorder[varindex],
                       oldsi = count[varindex],
                       maxsolni = cursol.get(jointindex).indices[0] + 1;
        for(uint32_t sii = oldsi; sii < numsolns; sii++) {
            vecsols[vindices[sii]][jointindex].maxsolutions = (unsigned char) maxsolni;
        }
    }

    // set free & mimic joints to have 0 maxsolutions and index -1
    for (size_t a = 0; a < vecsols.size(); ++a) {
        ikfast::IkSolution<T>& vecsol = vecsols[a];
        if(!vecsol._vfree.empty()) {
            for(uint32_t i = 0; i < nallvars; i++) {
                if(vecsol[i].freeind != (signed char) -1) {
                    vecsol[i].indices[0] = (unsigned char) -1;
                    vecsol[i].maxsolutions = 0;
                }
            }
        }
    }
}
} // namespace ikfast

#endif // OPENRAVE_IKFAST_HEADER

// The following code is dependent on the C++ library linking with.
#ifdef IKFAST_HAS_LIBRARY

// defined when creating a shared object/dll
#ifdef IKFAST_CLIBRARY
#ifdef _MSC_VER
#define IKFAST_API extern "C" __declspec(dllexport)
#else
#define IKFAST_API extern "C"
#endif
#else
#define IKFAST_API
#endif

#ifdef IKFAST_NAMESPACE
namespace IKFAST_NAMESPACE {
#endif

#ifdef IKFAST_REAL
typedef IKFAST_REAL IkReal;
#else
typedef double IkReal;
#endif

/** \brief Computes all IK solutions given a end effector coordinates and the free joints.

   - ``eetrans`` - 3 translation values. For iktype **TranslationXYOrientation3D**, the z-axis is the orientation.
   - ``eerot``
   - For **Transform6D** it is 9 values for the 3x3 rotation matrix.
   - For **Direction3D**, **Ray4D**, and **TranslationDirection5D**, the first 3 values represent the target direction.
   - For **TranslationXAxisAngle4D**, **TranslationYAxisAngle4D**, and **TranslationZAxisAngle4D** the first value represents the angle.
   - For **TranslationLocalGlobal6D**, the diagonal elements ([0],[4],[8]) are the local translation inside the end effector coordinate system.
 */
IKFAST_API bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, ikfast::IkSolutionListBase<IkReal>& solutions);

/** \brief Similar to ComputeIk except takes OpenRAVE boost::shared_ptr<RobotBase::Manipulator>*
 */
IKFAST_API bool ComputeIk2(const IkReal* eetrans, const IkReal* eerot, const IkReal* pfree, ikfast::IkSolutionListBase<IkReal>& solutions, void* pOpenRAVEManip);

/// \brief Computes the end effector coordinates given the joint values. This function is used to double check ik.
IKFAST_API void ComputeFk(const IkReal* joints, IkReal* eetrans, IkReal* eerot);

/// \brief returns the number of free parameters users has to set apriori
IKFAST_API int GetNumFreeParameters();

/// \brief the indices of the free parameters indexed by the chain joints
IKFAST_API const int* GetFreeIndices();

/// \brief the total number of indices of the chain
IKFAST_API int GetNumJoints();

/// \brief the size in bytes of the configured number type
IKFAST_API int GetIkRealSize();

/// \brief the ikfast version used to generate this file
IKFAST_API const char* GetIkFastVersion();

/// \brief the ik type ID
IKFAST_API int GetIkType();

/// \brief a hash of all the chain values used for double checking that the correct IK is used.
IKFAST_API const char* GetKinematicsHash();

#ifdef IKFAST_NAMESPACE
}
#endif

#endif // IKFAST_HAS_LIBRARY
