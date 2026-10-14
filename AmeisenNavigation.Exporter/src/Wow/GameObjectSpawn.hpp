#pragma once

#include <cmath>
#include <string>

#include "../Utils/Matrix4x4.hpp"
#include "../Utils/Vector3.hpp"

/// One spawned game object (WoW coordinates).
struct GameObjectSpawn
{
    int mapId = -1;
    unsigned int displayId = 0;
    std::string model; // model path, empty = from GameObjectDisplayInfo.dbc by displayId
    Vector3 position;
    float orientation = 0.0f; // radians around z, used when the rotation is zero
    float rotation[4]{};      // quaternion x, y, z, w (0, 0, 0, 0 = use the orientation)
    float scale = 1.0f;
    int type = -1; // GameObject type, -1 = unknown
};

namespace GameObjects {
/// Transform from the model's space to WoW coordinates: scale, rotation (quaternion, or the orientation around z),
/// translation.
inline Matrix4x4 SpawnTransform(const GameObjectSpawn& spawn) noexcept
{
    Matrix4x4 transform;
    transform.SetScale({spawn.scale, spawn.scale, spawn.scale});
    const float* q = spawn.rotation;

    if (q[0] != 0.0f || q[1] != 0.0f || q[2] != 0.0f || q[3] != 0.0f)
    {
        transform.SetRotation(q[0], q[1], q[2], q[3]);
    }
    else
    {
        transform.SetRotation(0.0f, 0.0f, std::sin(spawn.orientation * 0.5f), std::cos(spawn.orientation * 0.5f));
    }

    transform.SetTranslation(spawn.position);
    return transform;
}
} // namespace GameObjects
