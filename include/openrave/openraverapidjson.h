// Tencent is pleased to support the open source community by making RapidJSON available.
// 
// Copyright (C) 2015 THL A29 Limited, a Tencent company, and Milo Yip. All rights reserved.
//
// Licensed under the MIT License (the "License"); you may not use this file except
// in compliance with the License. You may obtain a copy of the License at
//
// http://opensource.org/licenses/MIT
//
// Unless required by applicable law or agreed to in writing, software distributed 
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR 
// CONDITIONS OF ANY KIND, either express or implied. See the License for the 
// specific language governing permissions and limitations under the License.

#ifndef OPENRAVE_RAPIDJSON_H
#define OPENRAVE_RAPIDJSON_H

#include <rapidjson/allocators.h>

namespace OpenRAVE {
namespace orrapidjson {

///////////////////////////////////////////////////////////////////////////////
// MemoryPoolAllocator

//! Default memory allocator used by the parser and DOM.
/*! This allocator allocate memory blocks from pre-allocated memory chunks. 

    It does not free memory blocks. And Realloc() only allocate new memory.

    The memory chunks are allocated by BaseAllocator, which is CrtAllocator by default.

    User may also supply a buffer as the first chunk.

    If the user-buffer is full then additional chunks are allocated by BaseAllocator.

    The user-buffer is not deallocated by this allocator.

    \tparam BaseAllocator the allocator type for allocating memory chunks. Default is CrtAllocator.
    \note implements Allocator concept
*/
template <typename BaseAllocator = rapidjson::CrtAllocator>
class MemoryPoolAllocator {
public:
    static const bool kNeedFree = false;    //!< Tell users that no need to call Free() with this allocator. (concept Allocator)

    //! Constructor with chunkSize.
    /*! \param chunkSize The size of memory chunk. The default is kDefaultChunkSize.
        \param baseAllocator The allocator for allocating memory chunks.
    */
    MemoryPoolAllocator(size_t chunkSize = kDefaultChunkCapacity, BaseAllocator* baseAllocator = 0) : 
        chunkHead_(0), chunkNext_(0), chunkTail_(0), chunk_capacity_(chunkSize), userBuffer_(0), baseAllocator_(baseAllocator), ownBaseAllocator_(0)
    {
    }

    //! Constructor with user-supplied buffer.
    /*! The user buffer will be used firstly. When it is full, memory pool allocates new chunk with chunk size.

        The user buffer will not be deallocated when this allocator is destructed.

        \param buffer User supplied buffer.
        \param size Size of the buffer in bytes. It must at least larger than sizeof(ChunkHeader).
        \param chunkSize The size of memory chunk. The default is kDefaultChunkSize.
        \param baseAllocator The allocator for allocating memory chunks.
    */
    MemoryPoolAllocator(void *buffer, size_t size, size_t chunkSize = kDefaultChunkCapacity, BaseAllocator* baseAllocator = 0) :
        chunkHead_(0), chunkNext_(0), chunkTail_(0), chunk_capacity_(chunkSize), userBuffer_(buffer), baseAllocator_(baseAllocator), ownBaseAllocator_(0)
    {
        RAPIDJSON_ASSERT(buffer != 0);
        RAPIDJSON_ASSERT(size > sizeof(ChunkHeader));
        chunkHead_ = reinterpret_cast<ChunkHeader*>(buffer);
        chunkHead_->capacity = size - sizeof(ChunkHeader);
        chunkHead_->size = 0;
        chunkHead_->next = 0;
        chunkHead_->prev = 0;
        chunkTail_ = chunkHead_;
        chunkNext_ = chunkHead_;
    }

    //! Destructor.
    /*! This deallocates all memory chunks, excluding the user-supplied buffer.
    */
    ~MemoryPoolAllocator() {
        Release();
        RAPIDJSON_DELETE(ownBaseAllocator_);
    }

    //! Deallocates all memory chunks, excluding the user-supplied buffer.
    void Release() {
        while (chunkTail_ && chunkTail_ != userBuffer_) {
            ChunkHeader* prev = chunkTail_->prev;
            baseAllocator_->Free(chunkTail_);
            chunkTail_ = prev;
        }
        chunkNext_ = chunkHead_ = chunkTail_;
        if (chunkHead_) {
            RAPIDJSON_ASSERT(chunkHead_->prev == 0);
            chunkHead_->next = 0;
            chunkHead_->size = 0; // Clear user buffer
        }
    }

    //! Release all memory chunks, allowing them to be reused
    void Clear() {
        for (ChunkHeader* c = chunkHead_; c != 0; c = c->next) {
            c->size = 0;
        }
        chunkNext_ = chunkHead_;
    }

    //! Reserve enough memory
    void Reserve(size_t size) {
        size_t capacity = Capacity();
        if (size > capacity) {
            _AllocateChunk(RAPIDJSON_ALIGN(size - capacity));
        }
    }

    //! Computes the total capacity of allocated memory chunks.
    /*! \return total capacity in bytes.
    */
    size_t Capacity() const {
        size_t capacity = 0;
        for (ChunkHeader* c = chunkHead_; c != 0; c = c->next) {
            capacity += c->capacity;
        }
        return capacity;
    }

    //! Computes the memory blocks allocated.
    /*! \return total used bytes.
    */
    size_t Size() const {
        size_t size = 0;
        for (ChunkHeader* c = chunkHead_; c != 0; c = c->next) {
            size += c->size;
        }
        return size;
    }

    //! Allocates a memory block. (concept Allocator)
    void* Malloc(size_t size) {
        if (!size) {
            return NULL;
        }

        size = RAPIDJSON_ALIGN(size);
        if (!_EnsureChunk(size)) {
            return NULL;
        }

        void *buffer = reinterpret_cast<char *>(chunkNext_) + RAPIDJSON_ALIGN(sizeof(ChunkHeader)) + chunkNext_->size;
        chunkNext_->size += size;
        return buffer;
    }

    //! Resizes a memory block (concept Allocator)
    void* Realloc(void* originalPtr, size_t originalSize, size_t newSize) {
        if (originalPtr == 0) {
            return Malloc(newSize);
        }

        if (newSize == 0) {
            return NULL;
        }

        originalSize = RAPIDJSON_ALIGN(originalSize);
        newSize = RAPIDJSON_ALIGN(newSize);

        // Do not shrink if new size is smaller than original
        if (originalSize >= newSize) {
            return originalPtr;
        }

        // Simply expand it if it is the last allocation and there is sufficient space
        if (originalPtr == reinterpret_cast<char *>(chunkNext_) + RAPIDJSON_ALIGN(sizeof(ChunkHeader)) + chunkNext_->size - originalSize) {
            size_t increment = static_cast<size_t>(newSize - originalSize);
            if (chunkNext_->size + increment <= chunkNext_->capacity) {
                chunkNext_->size += increment;
                return originalPtr;
            }
        }

        // Realloc process: allocate and copy memory, do not free original buffer.
        if (void* newBuffer = Malloc(newSize)) {
            if (originalSize) {
                std::memcpy(newBuffer, originalPtr, originalSize);
            }
            return newBuffer;
        }
        else {
            return NULL;
        }
    }

    //! Frees a memory block (concept Allocator)
    static void Free(void *ptr) { (void)ptr; } // Do nothing

private:
    //! Copy constructor is not permitted.
    MemoryPoolAllocator(const MemoryPoolAllocator& rhs) /* = delete */;
    //! Copy assignment operator is not permitted.
    MemoryPoolAllocator& operator=(const MemoryPoolAllocator& rhs) /* = delete */;

    //! Creates a new chunk.
    /*! \param capacity Capacity of the chunk in bytes.
        \return true if success.
    */
    bool _EnsureChunk(size_t size) {
        if (chunkNext_ != 0) {
            // check if chunk is big enough
            if (chunkNext_->size + size <= chunkNext_->capacity) {
                return true;
            }
            // search for for a big enough chunk
            for (ChunkHeader* c = chunkNext_->next; c != 0; c = c->next) {
                if (c->size + size > c->capacity) {
                    // not big enough, continue
                    continue;
                }
                if (c == chunkNext_->next) {
                    chunkNext_ = c;
                    return true;
                }
                // remove chunk from link list
                RAPIDJSON_ASSERT(c != chunkHead_);
                RAPIDJSON_ASSERT(c->prev != 0);
                c->prev->next = c->next;
                if (c->next) {
                    RAPIDJSON_ASSERT(c != chunkTail_);
                    c->next->prev = c->prev;
                } else {
                    chunkTail_ = c->prev;
                    RAPIDJSON_ASSERT(c == chunkTail_);
                }
                c->prev = c->next = 0;
                // add chunk to link list after chunkNext_
                RAPIDJSON_ASSERT(chunkNext_->next != 0);
                c->next = chunkNext_->next;
                chunkNext_->next->prev = c;
                c->prev = chunkNext_;
                chunkNext_->next = c;
                chunkNext_ = c;
                return true;
            }
        }
        // if no existing chunk can satisfy, need to allocate a new chunk
        ChunkHeader* chunk = _AllocateChunk(size);
        if (!chunk) {
            return false;
        }
        chunkNext_ = chunk;
        return true;
    }

    static const int kDefaultChunkCapacity = RAPIDJSON_ALLOCATOR_DEFAULT_CHUNK_CAPACITY; //!< Default chunk capacity.

    //! Chunk header for perpending to each chunk.
    /*! Chunks are stored as a doubly linked list.
    */
    struct ChunkHeader {
        size_t capacity;    //!< Capacity of the chunk in bytes (excluding the header itself).
        size_t size;        //!< Current size of allocated memory in bytes.
        ChunkHeader *next;  //!< Next chunk in the linked list.
        ChunkHeader *prev;  //!< Prev chunk in the linked list.
    };

    //! Allocate new chunk, but do not change chunkNext_
    ChunkHeader* _AllocateChunk(size_t size) {
        if (!baseAllocator_) {
            ownBaseAllocator_ = baseAllocator_ = RAPIDJSON_NEW(BaseAllocator)();
        }
        size_t capacity = chunk_capacity_;
        if (size > capacity) {
            capacity = size;
        }
        ChunkHeader* chunk = reinterpret_cast<ChunkHeader*>(baseAllocator_->Malloc(RAPIDJSON_ALIGN(sizeof(ChunkHeader)) + capacity));
        if (!chunk) {
            return 0;
        }
        chunk->capacity = capacity;
        chunk->size = 0;
        chunk->next = 0;
        chunk->prev = 0;
        if (!chunkNext_) {
            // first chunk in the list
            RAPIDJSON_ASSERT(chunkHead_ == 0);
            RAPIDJSON_ASSERT(chunkTail_ == 0);
            chunkHead_ = chunkTail_ = chunkNext_ = chunk; // have to set chunkNext_ since it is the first chunk
            return chunk;
        }
        if (!(chunkNext_->next)) {
            // last chunk in the list
            RAPIDJSON_ASSERT(chunkNext_ == chunkTail_);
            chunk->prev = chunkTail_;
            chunkTail_->next = chunk;
            chunkTail_ = chunk;
            return chunk;
        }
        // insert chunk to link list after chunkNext_
        chunk->next = chunkNext_->next;
        chunkNext_->next->prev = chunk;
        chunk->prev = chunkNext_;
        chunkNext_->next = chunk;
        return chunk;
    }

    ChunkHeader *chunkHead_;    //!< Head of the chunk linked-list.
    ChunkHeader *chunkNext_;    //!< Next available chunk in the linked-list. Only the next chunk serves allocation.
    ChunkHeader *chunkTail_;    //!< Tail of the chunk linked-list.
    size_t chunk_capacity_;     //!< The minimum capacity of chunk when they are allocated.
    void *userBuffer_;          //!< User supplied buffer.
    BaseAllocator* baseAllocator_;  //!< base allocator for allocating memory chunks.
    BaseAllocator* ownBaseAllocator_;   //!< base allocator created by this object.
};

} // namespace orrapidjson
} // namespace OpenRAVE

// replace the default allocator for rapidjson::Document and rapidjson::Value with our MemoryPoolAllocator
#ifndef RAPIDJSON_DEFAULT_ALLOCATOR
#define RAPIDJSON_DEFAULT_ALLOCATOR OpenRAVE::orrapidjson::MemoryPoolAllocator<>
#endif

#endif // OPENRAVE_RAPIDJSON_H
