// -*- coding: utf-8 -*-
// Copyright (C) 2023 Tan Li Boon <undisputed.seraphim@gmail.com>
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
#include <gpgme.h>
#include <gpgme++/context.h>
#include <gpgme++/data.h>
#include <gpgme++/decryptionresult.h>
#include <gpgme++/encryptionresult.h>
#include <gpgme++/keylistresult.h>
#include <gpgme++/interfaces/dataprovider.h>
#endif

namespace OpenRAVE {

#if OPENRAVE_ENCRYPTION

class IStreamProvider final : public GpgME::DataProvider {
public:
    explicit IStreamProvider(std::istream& stream) : _stream(stream)
    {
        _stream.seekg(0, std::ios::beg);
    }
    IStreamProvider(const IStreamProvider&) = delete;
    IStreamProvider(IStreamProvider&& other) = delete;
    ~IStreamProvider() override = default;

    bool isSupported(GpgME::DataProvider::Operation op) const override
    {
        return op == GpgME::DataProvider::Operation::Read || op == GpgME::DataProvider::Operation::Seek;
    }

    ssize_t read(void *buffer, size_t bufSize) override
    {
        return _stream.readsome(static_cast<char *>(buffer), bufSize);
    }

    ssize_t write(const void *, size_t) override
    {
        return -1;
    }

    off_t seek(off_t offset, int whence) override
    {
        switch (whence) {
        case SEEK_SET:
            return _stream.rdbuf()->pubseekoff(offset, std::ios::beg, std::ios::in);
        case SEEK_CUR:
            return _stream.rdbuf()->pubseekoff(offset, std::ios::cur, std::ios::in);
        case SEEK_END:
            return _stream.rdbuf()->pubseekoff(offset, std::ios::end, std::ios::in);
        default:
            return -1;
        }
    }

    void release() override {}

private:
    std::istream& _stream;
};

class OStreamProvider final : public GpgME::DataProvider {
public:
    explicit OStreamProvider(std::ostream& stream) : _stream(stream) {}
    OStreamProvider(const OStreamProvider&) = delete;
    OStreamProvider(OStreamProvider&& other) = delete;
    ~OStreamProvider() override = default;

    bool isSupported(GpgME::DataProvider::Operation op) const override
    {
        return op == GpgME::DataProvider::Operation::Write || op == GpgME::DataProvider::Operation::Seek;
    }

    ssize_t read(void *buffer, size_t bufSize) override
    {
        return -1;
    }

    ssize_t write(const void *buffer, size_t bufSize) override
    {
        return _stream.write(static_cast<const char *>(buffer), bufSize).good() ? bufSize : -1;
    }

    off_t seek(off_t offset, int whence) override
    {
        switch (whence) {
            case SEEK_SET:
                return _stream.rdbuf()->pubseekoff(offset, std::ios::beg, std::ios::out);
            case SEEK_CUR:
                return _stream.rdbuf()->pubseekoff(offset, std::ios::cur, std::ios::out);
            case SEEK_END:
                return _stream.rdbuf()->pubseekoff(offset, std::ios::end, std::ios::out);
            default:
                return -1;
        }
    }

    void release() override {}

private:
    std::ostream& _stream;
};

static GpgME::Error FindGPGKeyByName(std::unique_ptr<GpgME::Context>& gpgCtx, const std::unordered_set<std::string>& keyIds, std::vector<GpgME::Key>& outKeys)
{
    GpgME::Error err = gpgCtx->startKeyListing();
    if (err.code() != GPG_ERR_NO_ERROR) {
        return err;
    }
    for (;;) {
        GpgME::Key key = gpgCtx->nextKey(err);
        if (err.code() != GPG_ERR_NO_ERROR) {
            break;
        }
        if (!key.canEncrypt()) {
            continue;
        }
        if (keyIds.empty()) {
            RAVELOG_WARN("No key encryption key; OpenRAVE will use the first key if available.");
            outKeys.emplace_back(std::move(key));
            break;
        }
        for (const std::string& keyId : keyIds) {
            if (StringEndsWith(keyId, key.shortKeyID())) {
                outKeys.push_back(key);
                RAVELOG_INFO("Using GPG key from %s\n", key.userID(0).name());
                break;
            }
        }
    }
    GpgME::KeyListResult keylistresults = gpgCtx->endKeyListing();
    return keylistresults.error();
}

bool GpgDecrypt(std::istream& inputStream, std::ostream& outputStream)
{
    // gpgme_check_version must be executed at least once before we create a context.
    const char *gpg_version = gpgme_check_version(nullptr);
    RAVELOG_DEBUG("GPG version: %s\n", gpg_version);
    std::unique_ptr<GpgME::Context> gpgCtx = GpgME::Context::create(GpgME::Protocol::OpenPGP);
    if (!gpgCtx) {
        RAVELOG_ERROR("Failed to initialize GPG context.");
        return false;
    }

    IStreamProvider inputData(inputStream);
    GpgME::Data cipherData(&inputData);
    OStreamProvider outputData(outputStream);
    GpgME::Data plainData(&outputData);
    GpgME::DecryptionResult result = gpgCtx->decrypt(cipherData, plainData);
    if (result.error().code() == GPG_ERR_NO_ERROR) {
        return true;
    }
    RAVELOG_ERROR(result.error().asString());
    return false;
}

bool GpgEncrypt(std::istream& inputStream, std::ostream& outputStream, const std::unordered_set<std::string>& keyIds)
{
    // gpgme_check_version must be executed at least once before we create a context.
    const char *gpg_version = gpgme_check_version(nullptr);
    RAVELOG_DEBUG("GPG version: %s\n", gpg_version);
    std::unique_ptr<GpgME::Context> gpgCtx = GpgME::Context::create(GpgME::Protocol::OpenPGP);
    if (!gpgCtx) {
        RAVELOG_ERROR("Failed to initialize GPG context.");
        return false;
    }

    GpgME::Error err;
    std::vector<GpgME::Key> keys;
    err = FindGPGKeyByName(gpgCtx, keyIds, keys);
    if (err.code() != GPG_ERR_NO_ERROR) {
        RAVELOG_ERROR("Failed to find GPG keys: %s\n", err.asString());
        return false;
    }
    if (!keyIds.empty() && keys.size() < keyIds.size()) {
        RAVELOG_ERROR("Either the specified keys were not found, or there are no keys available.");
        return false;
    }

    IStreamProvider inputData(inputStream);
    GpgME::Data plainData(&inputData);
    OStreamProvider outputData(outputStream);
    GpgME::Data cipherData(&outputData);
    GpgME::EncryptionResult result = gpgCtx->encrypt(keys, plainData, cipherData, GpgME::Context::EncryptionFlags::AlwaysTrust);
    if (result.error().code() == GPG_ERR_NO_ERROR) {
        return true;
    }
    RAVELOG_ERROR("%s\n", result.error().asString());
    return false;
}

#else

bool GpgDecrypt(std::istream&, std::ostream&) {
    throw OPENRAVE_EXCEPTION_FORMAT0("Encryption not enabled in OpenRAVE, but decryption was requested.", ORE_NotImplemented);
}

bool GpgEncrypt(std::istream&, std::ostream&, const std::unordered_set<std::string>&) {
    throw OPENRAVE_EXCEPTION_FORMAT0("Encryption not enabled in OpenRAVE, but encryption was requested.", ORE_NotImplemented);
}

#endif

} // namespace OpenRAVE
