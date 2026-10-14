#pragma once

#include <condition_variable>
#include <mutex>
#include <unordered_map>

/// Readers/writer lock of a map's navmesh: requests read it (shared, re-entrant per thread), replacing tiles writes
/// it (exclusive). A waiting writer holds back new requests but never a thread that already reads (a nested
/// request on the same thread can't deadlock against it).
class NavMeshLock
{
    std::mutex Mutex;
    std::condition_variable Cv;
    int Readers = 0;
    bool Writer = false;
    int WritersWaiting = 0;

    static int& Depth(const NavMeshLock* lock)
    {
        thread_local std::unordered_map<const NavMeshLock*, int> depth;
        return depth[lock];
    }

public:
    void LockShared()
    {
        int& depth = Depth(this);

        if (depth++ > 0)
        {
            return;
        }

        std::unique_lock lock(Mutex);
        Cv.wait(lock, [this] { return !Writer && WritersWaiting == 0; });
        Readers++;
    }

    void UnlockShared()
    {
        int& depth = Depth(this);

        if (--depth > 0)
        {
            return;
        }

        {
            const std::lock_guard lock(Mutex);
            Readers--;
        }

        Cv.notify_all();
    }

    void Lock()
    {
        std::unique_lock lock(Mutex);
        WritersWaiting++;
        Cv.wait(lock, [this] { return !Writer && Readers == 0; });
        WritersWaiting--;
        Writer = true;
    }

    void Unlock()
    {
        {
            const std::lock_guard lock(Mutex);
            Writer = false;
        }

        Cv.notify_all();
    }

    /// RAII shared hold (movable, empty when default constructed).
    class Shared
    {
        NavMeshLock* Owner = nullptr;

    public:
        Shared() noexcept = default;
        explicit Shared(NavMeshLock& owner) : Owner(&owner) { Owner->LockShared(); }
        Shared(Shared&& other) noexcept : Owner(other.Owner) { other.Owner = nullptr; }
        Shared& operator=(Shared&& other) noexcept
        {
            if (this != &other)
            {
                Release();
                Owner = other.Owner;
                other.Owner = nullptr;
            }

            return *this;
        }
        Shared(const Shared&) = delete;
        Shared& operator=(const Shared&) = delete;
        ~Shared() { Release(); }

        void Release() noexcept
        {
            if (Owner)
            {
                Owner->UnlockShared();
                Owner = nullptr;
            }
        }
    };
};
