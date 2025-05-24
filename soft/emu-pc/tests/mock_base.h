/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo, Inc.
 * SPDX-License-Identifier: LicenseRef-QORVO-1
 */

#pragma once

#include <cassert>

template <typename T> class MockBase {
public:
    /* Retrieves the singleton pointer */
    static T *get_instance()
    {
        /* This brutal stop occurs when the mock has not been declared in the test suite
         * class. */
        assert(instance);
        return instance;
    }

protected:
    /* It must only be used by the destructor of the inherited class. */
    void clear_instance(T *self)
    {
        assert(instance == self);
        instance = nullptr;
    }
    /* It must only be used by the constructor of the inherited class. */
    void set_instance(T *self)
    {
        assert(instance == nullptr);
        instance = self;
    }

private:
    static inline T *instance = nullptr;
};
