namespace AmeisenNavigation.Client
{
    /// <summary>A circle that costs more to cross (protocol 3). The search prefers other ways and paths are led
    /// around it.</summary>
    /// <param name="Center">WoW coordinates, the height picks the floor (bridges over the zone aren't affected).</param>
    /// <param name="Radius">Up to 100 yards.</param>
    /// <param name="Cost">Travel cost multiplier inside, at least 1 (e.g. 10 = ten times as expensive).</param>
    public readonly record struct AvoidZone(Vector3 Center, float Radius, float Cost = 10f);

    /// <summary>A unit near the steering client (WoW coordinates, velocity in yards per second).</summary>
    public readonly record struct SteerUnit(Vector3 Position, Vector3 Velocity, float Radius = 0.5f);

    /// <summary>Result of <see cref="AmeisenNavClient.Steer"/>.</summary>
    /// <param name="IsValid">False if the request failed (no navmesh, invalid input, connection error).</param>
    /// <param name="Velocity">Velocity for the next tick (horizontal).</param>
    /// <param name="Position">Where that velocity leads on the navmesh after the tick.</param>
    public readonly record struct SteerResult(bool IsValid, Vector3 Velocity, Vector3 Position);

    /// <summary>
    /// Surface categories for <see cref="AmeisenNavClient.SetTerrainCost"/>, translated to the area ids of the
    /// server's navmesh format (ANP, TrinityCore or SkyFire MMAPs).
    /// </summary>
    public enum TerrainType
    {
        Ground,
        Road,
        Water,

        /// <summary>Lava and slime.</summary>
        BadLiquid,

        /// <summary>Steep slopes, excluded by default on TrinityCore MMAPs (ANP navmeshes have none).</summary>
        Steep,

        /// <summary>Off-mesh jumps, excluded by default: enable them for a bot that jumps where
        /// <see cref="PathPointFlags.Jump"/> says.</summary>
        Jump,

        /// <summary>Off-mesh drops off ledges.</summary>
        Drop,

        /// <summary>Custom off-mesh connections.</summary>
        Link,
    }
}
