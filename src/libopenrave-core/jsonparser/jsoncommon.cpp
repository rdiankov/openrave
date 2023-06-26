// -*- coding: utf-8 -*-
// Copyright (C) 2013 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "jsoncommon.h"
#include "stringutils.h"

#if OPENRAVE_ENCRYPTION
#include <iostream>
#include <sys/mman.h>
#include <gpgme.h>
#include <gpgme++/context.h>
#include <gpgme++/data.h>
#include <gpgme++/decryptionresult.h>
#include <gpgme++/encryptionresult.h>
#include <gpgme++/engineinfo.h>
#include <gpgme++/global.h>
#include <gpgme++/keylistresult.h>
#include <gpgme++/interfaces/dataprovider.h>
#endif

namespace OpenRAVE {

#if OPENRAVE_ENCRYPTION

class IStreamProvider final : public GpgME::DataProvider {
public:
    IStreamProvider(std::istream& stream)
        : _stream(stream)
        , _size(_stream.seekg(0, std::ios::end).tellg()) {
        _stream.seekg(0, std::ios::beg);
    }
    IStreamProvider(const IStreamProvider&) = delete;
    IStreamProvider(IStreamProvider&& other) noexcept = delete;
    ~IStreamProvider() noexcept {}

    bool isSupported(GpgME::DataProvider::Operation op) const override {
        return (op == GpgME::DataProvider::Operation::Read)
            || (op == GpgME::DataProvider::Operation::Seek)
            || (op == GpgME::DataProvider::Operation::Release);
    }

    ssize_t read(void *buffer, size_t bufSize) override {
        const size_t to_read = std::min(_size - _stream.tellg(), bufSize);
        _stream.read(static_cast<char*>(buffer), to_read);
        return _stream.bad() ? -1 : to_read;
    }

    ssize_t write(const void *buffer, size_t bufSize) override {
        return -1;
    }

    off_t seek(off_t offset, int whence) override {
        switch (whence) {
        case SEEK_SET: {
            _stream.seekg(offset, std::ios::beg);
            break;
        }
        case SEEK_CUR: {
            _stream.seekg(offset, std::ios::cur);
            break;
        }
        case SEEK_END: {
            _stream.seekg(offset, std::ios::end);
            break;
        }
        }
        return _stream.bad() ? -1 : off_t(_stream.tellg());
    }

    void release() override {}

private:
    std::istream& _stream;
    size_t _size;
};

class StringBufferProvider final : public GpgME::DataProvider {
public:
    StringBufferProvider(std::string& buffer) : _buffer(buffer) {}
    StringBufferProvider(const StringBufferProvider&) = delete;
    StringBufferProvider(StringBufferProvider&& other) noexcept = delete;
    ~StringBufferProvider() noexcept {}

    bool isSupported(GpgME::DataProvider::Operation op) const override {
        return true;
    }

    ssize_t read(void *buffer, size_t bufSize) override {
        const size_t delta = std::min(_buffer.size() - _pos, bufSize);
        ::memcpy(buffer, &_buffer[0], delta);
        _pos += delta;
        return delta;
    }

    ssize_t write(const void *buffer, size_t bufSize) override {
        const size_t delta = _buffer.size() - _pos;
        if (delta < bufSize) {
            _buffer.resize(_buffer.size() + (bufSize - delta));
        }
        ::memcpy(&_buffer[0] + _pos, buffer, bufSize);
        _pos += bufSize;
        return bufSize;
    }

    off_t seek(off_t offset, int whence) override {
        switch (whence) {
        case SEEK_SET: {
            _pos = offset;
            break;
        }
        case SEEK_CUR: {
            _pos += offset;
            break;
        }
        case SEEK_END: {
            _pos = _buffer.size() - offset;
            break;
        }
        }
        return _pos;
    }

    void release() override { _pos = 0; }

private:
    std::string& _buffer;
    std::streampos _pos;
};

GpgME::Error FindGPGKeyByName(std::unique_ptr<GpgME::Context>& gpgCtx, const std::string& keyName, GpgME::Key& outKey) {
    GpgME::Error err = gpgCtx->startKeyListing();
    if (err.code() != GPG_ERR_NO_ERROR) {
        return err;
    }
    do {
        outKey = gpgCtx->nextKey(err);
        if (err.code() != GPG_ERR_NO_ERROR) {
            break;
        }
        if (keyName.empty()) {
            RAVELOG_WARN("No key encryption key; OpenRAVE will use the first key if available.");
            break;
        }
        if (StringEndsWith(keyName, outKey.shortKeyID())) {
            RAVELOG_INFO("Using GPG key from %s\n", outKey.userID(0).name());
            break;
        }
    } while (err.code() != GPG_ERR_NO_ERROR);
    GpgME::KeyListResult keylistresults = gpgCtx->endKeyListing();
    return keylistresults.error();
}

bool GpgDecrypt(std::istream& inputStream, std::string& outputBuffer)
{
    // gpgme_check_version must be executed at least once before we create a context.
    const char* gpg_version = gpgme_check_version(NULL);
    RAVELOG_DEBUG("GPG version: %s\n", gpg_version);
    std::unique_ptr<GpgME::Context> gpgCtx = GpgME::Context::create(GpgME::Protocol::OpenPGP);
    if (!gpgCtx) {
        RAVELOG_ERROR("Failed to initialize GPG context.");
        return false;
    }

    IStreamProvider cipherStream(inputStream);
    GpgME::Data cipherData(&cipherStream);
    StringBufferProvider plainStream(outputBuffer);
    GpgME::Data plainData(&plainStream);
    GpgME::DecryptionResult result = gpgCtx->decrypt(cipherData, plainData);
    if (result.error().code() == GPG_ERR_NO_ERROR) {
        return true;
    } else {
        RAVELOG_ERROR("%s\n", result.error().asString());
        return false;
    }
}

bool GpgEncrypt(std::istream& inputStream, std::string& outputBuffer, const std::string& keyName)
{
    // gpgme_check_version must be executed at least once before we create a context.
    const char* gpg_version = gpgme_check_version(NULL);
    RAVELOG_DEBUG("GPG version: %s\n", gpg_version);
    std::unique_ptr<GpgME::Context> gpgCtx = GpgME::Context::create(GpgME::Protocol::OpenPGP);
    if (!gpgCtx) {
        RAVELOG_ERROR("Failed to initialize GPG context.");
        return false;
    }

    GpgME::Error err;
    GpgME::Key key;
    err = FindGPGKeyByName(gpgCtx, keyName, key);
    if (err.code() != GPG_ERR_NO_ERROR) {
        RAVELOG_ERROR("Failed to find GPG key %s: %s\n", keyName.c_str(), err.asString());
        return false;
    }
    if (key.isNull() || key.isInvalid()) {
        RAVELOG_ERROR("Either the specified key '%s' was not found, or there are no keys available.", keyName.c_str());
        return false;
    }

    IStreamProvider plainStream(inputStream);
    GpgME::Data plainData(&plainStream);
    StringBufferProvider cipherStringBuffer(outputBuffer);
    GpgME::Data cipherData(&cipherStringBuffer);
    GpgME::EncryptionResult result = gpgCtx->encrypt({key}, plainData, cipherData, GpgME::Context::EncryptionFlags::AlwaysTrust);
    if (result.error().code() == GPG_ERR_NO_ERROR) {
        return true;
    } else {
        RAVELOG_ERROR("%s\n", result.error().asString());
        return false;
    }
}

#else

bool GpgDecrypt(std::istream& inputStream, std::string& outputBuffer) {
    throw OPENRAVE_EXCEPTION_FORMAT("Encryption not enabled in OpenRAVE, but decryption was requested.");
}

bool GpgEncrypt(std::istream& inputStream, std::string& outputBuffer, const std::string& keyName) {
    throw OPENRAVE_EXCEPTION_FORMAT("Encryption not enabled in OpenRAVE, but encryption was requested.");
}

#endif

} // namespace OpenRAVE
