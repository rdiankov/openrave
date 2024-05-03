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
    \brief 
 */
#ifndef OPENRAVE_CONTAINER_H
#define OPENRAVE_CONTAINER_H

#include <openrave/config.h>
#include <openrave/openraveexception.h>

#include <stdint.h>
#include <vector>

namespace OpenRAVE {

namespace orcontainer {

/// \brief Represents Key-value pair.
template<typename DataType> struct NamedData
{
    NamedData(uint64_t nameId_, const DataType& data_) : nameId(nameId_), data(data_) {}

    ///\brief invalidates this object
    inline void Invalidate() { nameId = 0; }

    ///\brief Checks validity of this object
    ///\return whether valid
    inline bool IsValid() const { return nameId > 0; }

    ///\brief gets value
    inline const DataType& GetValue() const { return data; }

    ///\brief gets key
    inline uint64_t GetKey() const { return nameId; }

    ///\brief sets value for key
    ///\param nameId_ key for the value
    ///\param data_ value for the key
    inline void Set(uint64_t nameId_, const DataType& data_)
    {
        nameId = nameId_;
        data = data_;
    }

    uint64_t nameId = 0; ///< id for the semantic meaning of data. 0 means data is not valid.
    DataType data; ///< user-defined data. If nameId is 0, invalid
};

/// \brief Associative data structure similar to map, optimized to avoid memory allocation.
/// Similar to map, nameId(key) is unique. However, there can be multiple invalid nameIds.
template<typename DataType> struct Map
{
    /// \brief gets number of valid entries
    /// \return number of valid entries
    inline size_t GetSize() const { return _numValidElements; }

    /// \brief invalidates all entries while preserving memory
    void Clear()
    {
        // Just invalidate each element, but keep memory allocated
        for (NamedData<DataType>& customData : _vNamedDatas) {
            customData.Invalidate();
        }
        _numValidElements = 0;
        _beginValidElementsIndex = 0;
        _endValidElementsIndex = 0;
    }

    /// \brief erases element for name id.
    /// internally invalidates entry so not observable from outside but memory is kept allocated.
    /// \param nameId key to erase value of.
    void Erase(uint64_t nameId)
    {
        for (int64_t index = 0; index < _vNamedDatas.size(); ++index) {
            NamedData<DataType>& customData = _vNamedDatas[index];
            if (customData.nameId != nameId) {
                continue;
            }
            customData.Invalidate();

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
                            const NamedData<DataType>& data = _vNamedDatas[newBeginIndex];
                            if (!data.IsValid()) {
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
                            const NamedData<DataType>& data = _vNamedDatas[newEndIndex - 1];
                            if (!data.IsValid()) {
                                continue;
                            }
                            _endValidElementsIndex = newEndIndex;
                            break;
                        }
                    }
                }
            }
            OPENRAVE_ASSERT_OP(_numValidElements, <= ,(_endValidElementsIndex - _beginValidElementsIndex));
            return;
        }
        OPENRAVE_ASSERT_FORMAT(false, "nameId=%d is not found", nameId, OpenRAVE::ORE_InvalidArguments);
    }

    /// \brief finds element for key
    /// \param nameId key to find value for
    /// \param value value for key.
    /// \return whether key is found.
    bool Find(uint64_t nameId, DataType& value) const
    {
        OPENRAVE_ASSERT_FORMAT0(nameId != 0, "nameId cannot be 0. 0 is reserved for invalid", OpenRAVE::ORE_InvalidArguments);

        for (const NamedData<DataType>& customData : _vNamedDatas) {
            if (customData.nameId == nameId) {
                value = customData.data;
                return true;
            }
        }
        return false;
    }

    /// \brief inserts element
    /// \param nameId key to insert value for
    /// \param data value for key.
    void Insert(uint64_t nameId, const DataType& data)
    {
        OPENRAVE_ASSERT_FORMAT0(nameId != 0, "nameId cannot be 0. 0 is reserved for invalid", OpenRAVE::ORE_InvalidArguments);
        
        int64_t invalidIndex = -1;
        for (size_t index = 0; index < _vNamedDatas.size(); ++index) {
            NamedData<DataType>& customData = _vNamedDatas[index];
                
            if (customData.nameId == nameId) {
                // overwrite existing
                customData.data = data;
                return;
            }
            if (invalidIndex >= 0) {
                continue;
            }
            if (!customData.IsValid()) {
                invalidIndex = index;
            }
        }

        if (invalidIndex >= 0) {
            // reclaim existing invalid data
            _vNamedDatas[invalidIndex].Set(nameId, data);

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
            _vNamedDatas.push_back(NamedData<DataType>(nameId, data));
            _endValidElementsIndex++;
        }
        _numValidElements++;
        OPENRAVE_ASSERT_OP(_numValidElements, <= ,(_endValidElementsIndex - _beginValidElementsIndex));
    }

    /// \brief Iterator class to be used for Map class
    class Iterator
    {
    public:
        Iterator() = delete;
        Iterator(const std::vector<NamedData<DataType>>& data, size_t dataIndex, size_t endIndex)
            : _data(data), _dataIndex(dataIndex), _endIndex(endIndex)
        {
        }

        /// \brief Pre-increment
        /// iterate to next valid data or to end index. Even when iterator is beyond end index, iterates to the next index once.
        Iterator& operator++()
        {
            do {
                ++_dataIndex;
            } while (_dataIndex < _endIndex && !_data[_dataIndex].IsValid());
                
            return *this;
        }

        /// \brief compares for equality
        /// \param other the other object to compare against
        bool operator==(const Iterator& other) const { return _dataIndex == other._dataIndex && this == &other; }

        /// \brief compares for inequality
        /// \param other the other object to compare against
        bool operator!=(const Iterator& other) const { return !(*this == other); }

        /// \brief dereference operator
        const NamedData<DataType>& operator*() const
        {
            return _data[_dataIndex];
        }
    private:
        const std::vector<NamedData<DataType>>& _data; ///< underlying data to iterate over
        size_t _dataIndex; ///< current position
        const size_t _endIndex; ///< end position
    };

    /// \brief gets iterator to the first element
    /// \return iterator to first element
    Iterator GetBegin() const
    {
        return Iterator(_vNamedDatas, _beginValidElementsIndex, _endValidElementsIndex);
    }

    /// \brief gets iterator to the element after last element
    /// \return iterator to the element after last element
    Iterator GetEnd() const
    {
        return Iterator(_vNamedDatas, _endValidElementsIndex, _endValidElementsIndex);
    }

private:
    std::vector<NamedData<DataType>> _vNamedDatas; ///< Vector of elements. Vector is not sorted. For small size, faster to keep it unsorted and do brute-force search.
    size_t _numValidElements = 0; ///< number of valid elements, at most _vNamedDatas.size()
    size_t _beginValidElementsIndex = 0; ///< index to the first element
    size_t _endValidElementsIndex = 0; ///< index to the end element (last + 1)
};
    
} // namespace orcontainer

} // namespace OpenRAVE

#endif // OPENRAVE_CONTAINER_H
