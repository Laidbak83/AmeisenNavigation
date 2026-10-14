using System;
using System.Collections.Generic;

namespace AmeisenNavigation.Client
{
    /// <summary>Kinds of geometry a static world query ignores (protocol 4).</summary>
    [Flags]
    public enum WorldIgnore
    {
        None = 0,
        Terrain = 1 << 0,
        Wmos = 1 << 1,

        /// <summary>M2 models placed by the map: trees, rocks, fences, crates.</summary>
        Doodads = 1 << 2,

        /// <summary>Liquid surfaces (line of sight always ignores them).</summary>
        Liquids = 1 << 3,
        GameObjects = 1 << 4,
    }

    /// <summary>Where a piece of the static world comes from.</summary>
    public enum WorldSourceKind : byte
    {
        Terrain = 0,

        /// <summary>A WMO (building, cave, bridge...), the id is its placement's unique id.</summary>
        Wmo = 1,

        /// <summary>An M2 placed by the map, the id is its placement's unique id.</summary>
        Doodad = 2,
        Liquid = 3,

        /// <summary>A game object spawned by the server at export time, the id is its display id.</summary>
        GameObject = 4,
        Other = 15,
    }

    /// <summary>Provenance of a triangle of the static world: kind and id.</summary>
    public readonly record struct WorldSource(uint Raw)
    {
        public WorldSourceKind Kind => (WorldSourceKind)(Raw >> 28);
        public uint Id => Raw & 0x0FFFFFFFu;

        public override string ToString() => $"{Kind} {Id}";
    }

    /// <summary>Result of <see cref="AmeisenNavClient.LineOfSight"/> and <see cref="AmeisenNavClient.Sweep"/>.</summary>
    /// <param name="Known">False if the server has no static world there (or the request failed): nothing is
    /// known, Hit is false.</param>
    /// <param name="Hit">Something is in the way.</param>
    /// <param name="T">Fraction of start -> end before the hit (1 if clear).</param>
    /// <param name="Position">Hit position (line of sight) or where the body stops (sweep), end if clear.</param>
    /// <param name="Normal">Surface normal facing the ray.</param>
    /// <param name="Source">What was hit.</param>
    /// <param name="Area">Navmesh area id of the hit triangle (0 = too steep to stand on).</param>
    public readonly record struct WorldHit(bool Known, bool Hit, float T, Vector3 Position, Vector3 Normal,
                                           WorldSource Source, int Area);

    /// <summary>A surface under a point.</summary>
    /// <param name="Standable">Walkable ground (not too steep, not liquid).</param>
    public readonly record struct WorldSurface(float Z, Vector3 Normal, WorldSource Source, byte Area, bool Standable,
                                               bool Liquid);

    /// <summary>A game object the bot sees (from the object manager), for <see cref="AmeisenNavClient.SetObjects"/>.</summary>
    /// <param name="Guid">Object guid, the same object is updated in place.</param>
    /// <param name="DisplayId">GameObjectDisplayInfo id (the exporter wrote its collision model).</param>
    /// <param name="Rotation">Quaternion (x, y, z, w) of the object, all zero = none.</param>
    /// <param name="Blocking">Its collision is in the way (a closed door). False keeps it but out of the way.</param>
    public readonly record struct WorldObject(ulong Guid, uint DisplayId, Vector3 Position, Quaternion Rotation,
                                              float Scale = 1f, bool Blocking = true);

    /// <summary>A rotation quaternion.</summary>
    public readonly record struct Quaternion(float X, float Y, float Z, float W)
    {
        public static readonly Quaternion None = new(0f, 0f, 0f, 0f);

        /// <summary>Rotation by an orientation around the z axis (radians), like a game object's facing.</summary>
        public static Quaternion FromOrientation(float orientation)
            => new(0f, 0f, MathF.Sin(orientation * 0.5f), MathF.Cos(orientation * 0.5f));
    }

    /// <summary>Result of <see cref="AmeisenNavClient.SetObjects"/>.</summary>
    /// <param name="Accepted">Objects whose model the server knows, -1 if the map can't change (no static world).</param>
    /// <param name="PendingTiles">Navmesh tiles still being rebuilt: paths see the objects once this reaches 0.</param>
    public readonly record struct SetObjectsResult(int Accepted, int PendingTiles);

    /// <summary>Result of <see cref="AmeisenNavClient.GetSurfaces"/>: highest first, and the AreaTable id there
    /// (0 = unknown).</summary>
    public sealed record WorldSurfaces(IReadOnlyList<WorldSurface> Surfaces, uint AreaId);
}
