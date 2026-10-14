#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "../Utils/Misc.hpp"
#include "../Utils/Vector3.hpp"

#pragma pack(push, 1)
struct MD20
{
    unsigned char magic[4];
    unsigned int size;
    unsigned int lengthModelName;
    unsigned int offsetName;
    unsigned int modelFlags;
    unsigned int countGlobalSequences;
    unsigned int offsetGlobalSequences;
    unsigned int countAnimations;
    unsigned int offsetAnimations;
    unsigned int countAnimationLookup;
    unsigned int offsetAnimationLookup;
    unsigned int countBones;
    unsigned int offsetBones;
    unsigned int countKeyBoneLookup;
    unsigned int offsetKeyBoneLookup;
    unsigned int countVertices;
    unsigned int offsetVertices;
    unsigned int countViews;
    unsigned int countColors;
    unsigned int offsetColors;
    unsigned int countTextures;
    unsigned int offsetTextures;
    unsigned int countTransparency;
    unsigned int offsetTransparency;
    unsigned int countUvAnimation;
    unsigned int offsetUvAnimation;
    unsigned int countTexReplace;
    unsigned int offsetTexReplace;
    unsigned int countRenderFlags;
    unsigned int offsetRenderFlags;
    unsigned int countBoneLookup;
    unsigned int offsetBoneLookup;
    unsigned int countTexLookup;
    unsigned int offsetTexLookup;
    unsigned int countTexUnits;
    unsigned int offsetTexUnits;
    unsigned int countTransLookup;
    unsigned int offsetTransLookup;
    unsigned int countUvAnimLookup;
    unsigned int offsetUvAnimLookup;
    Vector3 vertexBox[2];
    float vertexRadius;
    Vector3 boundingBox[2];
    float boundingRadius;
    unsigned int countBoundingTriangles;
    unsigned int offsetBoundingTriangles;
    unsigned int countBoundingVertices;
    unsigned int offsetBoundingVertices;
    unsigned int countBoundingNormals;
    unsigned int offsetBoundingNormals;
};

/// Collision part of the M2 header (from the vertex bounding box on). Its position depends on the version:
/// before WotLK (version < 264) the header has three more fields in front of it (playable animation lookup, skin
/// profiles as an array instead of a count, texture flipbooks: 20 bytes).
struct M2Collision
{
    Vector3 vertexBox[2];
    float vertexRadius;
    Vector3 boundingBox[2];
    float boundingRadius;
    unsigned int countBoundingTriangles; // indices (3 per triangle)
    unsigned int offsetBoundingTriangles;
    unsigned int countBoundingVertices;
    unsigned int offsetBoundingVertices;
    unsigned int countBoundingNormals;
    unsigned int offsetBoundingNormals;
};
#pragma pack(pop)

static_assert(sizeof(M2Collision) == 80);

constexpr unsigned int M2_VERSION_WOTLK = 264;
constexpr size_t M2_COLLISION_OFFSET_WOTLK = offsetof(MD20, vertexBox);
constexpr size_t M2_COLLISION_OFFSET_CLASSIC = M2_COLLISION_OFFSET_WOTLK + 20;

/// An M2 model file: MD20 (vanilla to Warlords) or the chunked MD21 format (Legion+, MD20 data in the MD21 chunk,
/// offsets relative to it). Only the collision mesh is used.
struct M2
{
    unsigned char* Data;
    unsigned int Size;

private:
    /// Start and size of the MD20 data.
    const unsigned char* Base() const noexcept { return IsChunked() ? Data + 8 : Data; }
    size_t BaseSize() const noexcept { return IsChunked() ? Size - 8 : Size; }

    bool IsChunked() const noexcept { return Size >= 8 && std::memcmp(Data, "MD21", 4) == 0; }

    size_t CollisionOffset() const noexcept
    {
        return Version() < M2_VERSION_WOTLK ? M2_COLLISION_OFFSET_CLASSIC : M2_COLLISION_OFFSET_WOTLK;
    }

public:
    unsigned int Version() const noexcept
    {
        unsigned int version = 0;
        std::memcpy(&version, Base() + 4, sizeof(version));
        return version;
    }

    /// The collision header (copied, the file data isn't aligned).
    M2Collision Collision() const noexcept
    {
        M2Collision collision{};
        std::memcpy(&collision, Base() + CollisionOffset(), sizeof(collision));
        return collision;
    }

    /// Header present and the collision vertex/index arrays inside the file.
    bool IsValid() const noexcept
    {
        if (!Data || Size < 8)
        {
            return false;
        }

        const size_t size = BaseSize();

        if (size < 8 || std::memcmp(Base(), "MD20", 4) != 0 || size < CollisionOffset() + sizeof(M2Collision))
        {
            return false;
        }

        const M2Collision c = Collision();
        const uint64_t vertsEnd =
            static_cast<uint64_t>(c.offsetBoundingVertices) + static_cast<uint64_t>(c.countBoundingVertices) * sizeof(Vector3);
        const uint64_t trisEnd = static_cast<uint64_t>(c.offsetBoundingTriangles)
                                 + static_cast<uint64_t>(c.countBoundingTriangles) * sizeof(unsigned short);
        return vertsEnd <= size && trisEnd <= size;
    }

    Vector3 Vertex(size_t i) const noexcept
    {
        Vector3 v;
        std::memcpy(&v, Base() + Collision().offsetBoundingVertices + sizeof(Vector3) * i, sizeof(v));
        return v;
    }

    unsigned short Index(size_t i) const noexcept
    {
        unsigned short index = 0;
        std::memcpy(&index, Base() + Collision().offsetBoundingTriangles + sizeof(unsigned short) * i, sizeof(index));
        return index;
    }

    /// Has a collision mesh.
    bool IsCollideable() const noexcept
    {
        const M2Collision c = Collision();
        return c.countBoundingVertices > 0 && c.countBoundingTriangles >= 3 && c.offsetBoundingVertices > 0
               && c.offsetBoundingTriangles > 0 && c.boundingRadius > 0.0f;
    }
};
