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
#include <string>
#include <stdexcept>
#include <vector>

namespace OpenRAVE {

namespace orcontainer {

template<typename DataType> struct NamedData
{
    NamedData(uint64_t nameId_, const DataType& data_) : nameId(nameId_), data(data_) {}
    inline void Invalidate() { nameId = 0; }
    inline bool IsValid() const { return nameId > 0; }
    inline const DataType& Get() const { return data; }
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
template<typename DataType> struct NamedDatas
{
    inline size_t GetSize() const { return _numValidElements; }

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
        
    bool Find(uint64_t nameId, DataType& found) const
    {
        for (const NamedData<DataType>& customData : _vNamedDatas) {
            if (customData.nameId == nameId) {
                found = customData.data;
                return true;
            }
        }
        return false;
    }

    void Insert(uint64_t nameId, const DataType& data)
    {
        OPENRAVE_ASSERT_FORMAT0(nameId != 0, "nameId cannot be 0. 0 is reserved for invalid", OpenRAVE::ORE_InvalidArguments);
        
        int64_t invalidIndex = -1;
        for (int64_t index = 0; index < _vNamedDatas.size(); ++index) {
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

    class Iterator
    {
    public:
        Iterator() = delete;
        Iterator(const std::vector<NamedData<DataType>>& data, size_t dataIndex, size_t endIndex)
            : _data(data), _dataIndex(dataIndex), _endIndex(endIndex)
        {
        }

        // iterate to next valid data or to end index. Even when iterator is beyond end index, iterates to the next index once.
        Iterator& operator++()
        {
            do {
                _dataIndex++;
            } while (_dataIndex < _endIndex && !_data[_dataIndex].IsValid());
                
            return *this;
        }

        bool operator==(const Iterator& other) const { return _dataIndex == other._dataIndex; }
        bool operator!=(const Iterator& other) const { return !(*this == other); }

        const NamedData<DataType>& operator*() const
        {
            return _data[_dataIndex];
        }

        const std::vector<NamedData<DataType>>& _data;
        size_t _dataIndex;
        const size_t _endIndex;
    };

    Iterator GetBegin() const
    {
        return Iterator(_vNamedDatas, _beginValidElementsIndex, _endValidElementsIndex);
    }

    Iterator GetEnd() const
    {
        return Iterator(_vNamedDatas, _endValidElementsIndex, _endValidElementsIndex);
    }

private:
    std::vector<NamedData<DataType>> _vNamedDatas; ///< Vector of elements. Vector is not sorted. For small size, faster to keep it unsorted and do brute-force search.
    size_t _numValidElements = 0; ///< number of valid elements, at most _vNamedDatas.size()
    size_t _beginValidElementsIndex = 0;
    size_t _endValidElementsIndex = 0;
};
    
} // namespace orcontainer

} // namespace OpenRAVE

#endif // OPENRAVE_CONTAINER_H
