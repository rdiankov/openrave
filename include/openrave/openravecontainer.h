// -*- coding: utf-8 -*-
// Copyright (C) 2024 OpenRAVE
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

/** \file openravecontainer.h
    \brief Collection of high performance containers
 */
#ifndef OPENRAVE_CONTAINER_H
#define OPENRAVE_CONTAINER_H

#include <openrave/config.h>
#include <openrave/openraveexception.h>

#include <stdint.h>
#include <vector>

namespace OpenRAVE {

namespace orcontainer {

typedef uint32_t VectorBackedNameId;
inline void ClearNameId(VectorBackedNameId& nameId)
{
    if (nameId == 0) {
        return;
    }
    nameId = 0;
}

inline bool IsValidNameId(VectorBackedNameId nameId)
{
    return nameId != 0;
}

/// \brief Associative data structure similar to map, optimized to avoid memory allocation. More suitable for relatively small number of elements.
/// Similar to map, nameId (key) is unique. However, there can be multiple invalid nameIds.
template<typename DataType> class VectorBackedMap
{
public:
    /// \brief gets number of valid entries
    /// \return number of valid entries
    inline size_t GetSize() const {
        return _numValidElements;
    }

    /// \brief invalidates all entries while preserving memory
    inline void Clear()
    {
        // Just invalidate each element, but keep memory allocated
        for (size_t index = _beginValidElementsIndex; index < _endValidElementsIndex; ++index) {
            OpenRAVE::orcontainer::ClearNameId(_vNameIds[index]);
        }
        _numValidElements = 0;
        _beginValidElementsIndex = 0;
        _endValidElementsIndex = 0;
    }

    /// \brief erases element for name id.
    /// internally invalidates entry so not observable from outside but memory is kept allocated.
    /// \param nameId key to erase value of.
    inline void Erase(VectorBackedNameId nameId)
    {
        for (int64_t index = 0; index < _vNameIds.size(); ++index) {
            VectorBackedNameId& _nameId =  _vNameIds[index];
            if (_nameId != nameId) {
                continue;
            }
            OpenRAVE::orcontainer::ClearNameId(_nameId);

            _numValidElements--;
            if (_numValidElements == 0) {
                _beginValidElementsIndex = 0;
                _endValidElementsIndex = 0;
            }
            else {
                // update begin and end index
                if (index == _beginValidElementsIndex) {
                    if (_numValidElements == 1) {
                        _beginValidElementsIndex = _endValidElementsIndex - 1;
                    }
                    else {
                        for (size_t newBeginIndex = index + 1; newBeginIndex < _endValidElementsIndex; ++newBeginIndex) {
                            if (!IsValidNameId(_vNameIds[newBeginIndex])) {
                                continue;
                            }
                            _beginValidElementsIndex = newBeginIndex;
                            break;
                        }
                    }
                }
                else if (index + 1 == _endValidElementsIndex) {
                    if (_numValidElements == 1) {
                        _endValidElementsIndex = _beginValidElementsIndex + 1;
                    }
                    else {
                        for (size_t newEndIndex = _endValidElementsIndex - 1; newEndIndex > _beginValidElementsIndex; --newEndIndex) {
                            if (!IsValidNameId(_vNameIds[newEndIndex - 1])) {
                                continue;
                            }
                            _endValidElementsIndex = newEndIndex;
                            break;
                        }
                    }
                }
            }
            OPENRAVE_ASSERT_OP(_numValidElements, <=,(_endValidElementsIndex - _beginValidElementsIndex));
            return;
        }
        OPENRAVE_ASSERT_FORMAT(false, "nameId=%d is not found", nameId, OpenRAVE::ORE_InvalidArguments);
    }

    /// \brief finds element for key
    /// \param nameId key to find value for
    /// \param value value for key.
    /// \return whether key is found.
    inline bool Find(VectorBackedNameId nameId, DataType& value) const
    {
        OPENRAVE_ASSERT_FORMAT0(nameId != 0, "nameId cannot be 0. 0 is reserved for invalid", OpenRAVE::ORE_InvalidArguments);

        for (size_t index = _beginValidElementsIndex; index < _endValidElementsIndex; ++index) {
            if (_vNameIds[index] == nameId) {
                value = _vDatas[index];
                return true;
            }
        }
        return false;
    }

    /// \brief inserts element
    /// \param nameId key to insert value for
    /// \param data value for key.
    /// \return true if nameId did not exist, false if nameId existed and its value is overwritten
    inline bool Insert(VectorBackedNameId nameId, const DataType& data)
    {
        OPENRAVE_ASSERT_FORMAT0(nameId != 0, "nameId cannot be 0. 0 is reserved for invalid", OpenRAVE::ORE_InvalidArguments);

        // if nameId currently does not exist, try to reclaim invalidated cache
        int64_t invalidIndex = -1;
        for (size_t index = _beginValidElementsIndex; index < _endValidElementsIndex; ++index) {
            if (_vNameIds[index] == nameId) {
                // overwrite existing
                _vDatas[index] = data;
                return false;
            }
            if (invalidIndex >= 0) {
                continue;
            }
            if (!IsValidNameId(_vNameIds[index])) {
                invalidIndex = index;
            }
        }

        // always check the beginning for an invalid index regardless of current invalidIndex value
        for (size_t index = 0; index < _beginValidElementsIndex; ++index) {
            if (!IsValidNameId(_vNameIds[index])) {
                invalidIndex = index;
                break;
            }
        }

        if (invalidIndex < 0) {
            // not found, now check the end
            for (size_t index = _endValidElementsIndex; index < _vNameIds.size(); ++index) {
                if (!IsValidNameId(_vNameIds[index])) {
                    invalidIndex = index;
                    break;
                }
            }
        }

        if (invalidIndex >= 0) {
            // reclaim existing invalid data
            _vNameIds[invalidIndex] = nameId;
            _vDatas[invalidIndex] = data;

            // update begin and end index
            if (invalidIndex < _beginValidElementsIndex) {
                _beginValidElementsIndex = invalidIndex;
            }
            if (invalidIndex + 1 > _endValidElementsIndex) {
                _endValidElementsIndex = invalidIndex + 1;
            }
        }
        else {
            // cannot reclaim, insert new data.
            _vNameIds.push_back(nameId);
            _vDatas.push_back(data);
            _endValidElementsIndex++;
            // at this point _endValidElementsIndex == _vDatas.size()
            //OPENRAVE_ASSERT_OP(_endValidElementsIndex, ==, _vNameIds.size());
        }
        _numValidElements++;
        OPENRAVE_ASSERT_OP_FORMAT(_numValidElements, <=,(_endValidElementsIndex - _beginValidElementsIndex), "Where nameId=%s, _numValidElements=%d, _beginValidElementsIndex=%u, _endValidElementsIndex=%u", _numValidElements%_beginValidElementsIndex%_endValidElementsIndex, ORE_InvalidState);
        return true;
    }

    /// \brief Iterator class to be used for Map class
    class Iterator
    {
public:
        Iterator() = delete;
        Iterator(const VectorBackedMap& map, size_t dataIndex, size_t endIndex)
            : _map(map), _dataIndex(dataIndex), _endIndex(endIndex)
        {
        }

        /// \brief Pre-increment
        /// iterate to next valid data or to end index. Even when iterator is beyond end index, iterates to the next index once.
        inline Iterator& operator++()
        {
            do {
                ++_dataIndex;
            } while (_dataIndex < _endIndex && !IsValidNameId(_map._vNameIds[_dataIndex]));

            return *this;
        }

        /// \brief compares for equality
        /// \param other the other object to compare against
        inline bool operator==(const Iterator& other) const {
            return _dataIndex == other._dataIndex && &(*this)._map == &(other._map);
        }

        /// \brief compares for inequality
        /// \param other the other object to compare against
        inline bool operator!=(const Iterator& other) const {
            return !(*this == other);
        }

        /// \brief dereference operator
        inline VectorBackedNameId GetNameId() const
        {
            return _map._vNameIds[_dataIndex];
        }

        inline const DataType& GetValue() const
        {
            return _map._vDatas[_dataIndex];
        }

private:
        const VectorBackedMap& _map; ///< Underlying data
        size_t _dataIndex; ///< current position
        const size_t _endIndex; ///< end position
    };

    /// \brief gets iterator to the first element
    /// \return iterator to first element
    inline Iterator GetBegin() const
    {
        return Iterator(*this, _beginValidElementsIndex, _endValidElementsIndex);
    }

    /// \brief gets iterator to the element after last element
    /// \return iterator to the element after last element
    inline Iterator GetEnd() const
    {
        return Iterator(*this, _endValidElementsIndex, _endValidElementsIndex);
    }

private:
    std::vector<VectorBackedNameId> _vNameIds; ///< Vector of name ids (keys). Vector is not sorted. For small size, faster to keep it unsorted and do brute-force search.
    std::vector<DataType> _vDatas; ///< Vector of values. Vector is not sorted. For small size, faster to keep it unsorted and do brute-force search.
    size_t _numValidElements = 0; ///< number of valid elements, at most _vNamedDatas.size()
    size_t _beginValidElementsIndex = 0; ///< index to the first element
    size_t _endValidElementsIndex = 0; ///< index to the end element (last + 1)
};

} // namespace orcontainer

} // namespace OpenRAVE

#endif // OPENRAVE_CONTAINER_H
