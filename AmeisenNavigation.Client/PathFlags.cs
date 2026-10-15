using System;

namespace AmeisenNavigation.Client
{
    /// <summary>
    /// Path request flags. Combine with bitwise OR.
    /// At most one smoothing flag and one validation flag should be set.
    /// </summary>
    [Flags]
    public enum PathFlags
    {
        None = 0,
        SmoothChaikin = 1 << 0,
        SmoothCatmullRom = 1 << 1,
        SmoothBezier = 1 << 2,
        ValidateClosestPointOnPoly = 1 << 3,
        ValidateMoveAlongSurface = 1 << 4,

        /// <summary>
        /// Fail (return null) instead of returning a path that ends as close as possible to an unreachable
        /// end position. Requires protocol version 2 (server 1.9+), older servers ignore it.
        /// </summary>
        RequireComplete = 1 << 5,

        /// <summary>
        /// Keep corners <see cref="PathTuning.WallMargin"/> away from the walls they bend around.
        /// Protocol 3 (server 1.10+), older servers ignore it.
        /// </summary>
        AvoidWalls = 1 << 6,

        /// <summary>Skip corners wherever the navmesh allows a straight line. Protocol 3.</summary>
        Shortcut = 1 << 7,

        /// <summary>
        /// Replace corners by arcs around them (<see cref="PathTuning.CornerRadius"/>,
        /// <see cref="PathTuning.CornerPoints"/>). Protocol 3.
        /// </summary>
        RoundCorners = 1 << 8,

        /// <summary>
        /// Reuse the corridor of the connection's previous path while walking along it: corners stay stable
        /// between requests and the request is cheap. GetPath/GetPathEx only. Protocol 3.
        /// </summary>
        Follow = 1 << 9,

        /// <summary>GetPathEx only: random offsets like <see cref="AmeisenNavClient.GetRandomPath"/>. Protocol 3.</summary>
        Randomize = 1 << 10,

        /// <summary>
        /// Sweep a body (server's fBodyRadius, fBodyHeight, fBodyStepHeight) along the path through the static world
        /// and game objects: goes around what the navmesh doesn't know (an object whose tile isn't rebuilt yet, a
        /// fence on navmeshes that walk over them), flags what it can't go around (<see cref="PathPointFlags.Blocked"/>).
        /// Protocol 4, needs a .anw file.
        /// </summary>
        ClearBody = 1 << 11,

        /// <summary>
        /// Dense waypoints, all on the navmesh surface: where the ground changes (water, road, building floor,
        /// faction border), where the height profile bends (crests, dips, stairs, bridge humps) and at most
        /// <see cref="PathTuning.WaypointSpacing"/> apart (server default fWaypointSpacing, 4 yards), kept away from
        /// the navmesh edge (fEdgeMargin: cliffs, docks) and on the true ground when the server has the static
        /// world. Corners and off-mesh connections stay as they are. Protocol 4.
        /// </summary>
        Detail = 1 << 12,

        /// <summary>
        /// <see cref="AmeisenNavClient.GetPathEx"/> only: also return the navmesh area under every point and what the
        /// ground does there (<see cref="PathResult.Areas"/>, <see cref="PathResult.Info"/>). Most useful with
        /// <see cref="Detail"/>. Protocol 4 (server 1.12+).
        /// </summary>
        SurfaceInfo = 1 << 13,

        /// <summary>Smooth natural movement: shortcuts, distance to walls and round corners.</summary>
        Fluid = AvoidWalls | Shortcut | RoundCorners,
    }

    /// <summary>What a point of a <see cref="PathResult"/> is.</summary>
    [Flags]
    public enum PathPointFlags : byte
    {
        None = 0,
        Start = 1 << 0,
        End = 1 << 1,

        /// <summary>Leave this point by an off-mesh connection, the next point is where it ends.</summary>
        OffMesh = 1 << 2,

        /// <summary>... by jumping (over a low obstacle or a gap).</summary>
        Jump = 1 << 3,

        /// <summary>... by walking off a ledge (the character falls).</summary>
        Drop = 1 << 4,

        /// <summary>... by a custom connection (server's off-mesh file or the navmesh file's connections).</summary>
        Link = 1 << 5,

        /// <summary>Added by <see cref="PathFlags.ClearBody"/> to go around something the body would run into.</summary>
        Bypass = 1 << 6,

        /// <summary><see cref="PathFlags.ClearBody"/>: something is in the way to the next point that the body can't
        /// walk over or around nearby (jump, or open the door).</summary>
        Blocked = 1 << 7,
    }

    /// <summary>What the ground does at a point of a <see cref="PathResult"/> (<see cref="PathFlags.SurfaceInfo"/>).</summary>
    [Flags]
    public enum PathPointInfo : byte
    {
        None = 0,

        /// <summary>The ground changes after this point: water, road, building floor, faction border (see
        /// <see cref="PathResult.Areas"/>).</summary>
        SurfaceChange = 1 << 0,

        /// <summary>Top of a rise: up before, down after.</summary>
        Crest = 1 << 1,

        /// <summary>Bottom of a dip.</summary>
        Dip = 1 << 2,

        /// <summary>The slope changes here (stairs, a ramp starts or ends).</summary>
        SlopeChange = 1 << 3,

        /// <summary>Added by <see cref="PathFlags.Detail"/> only to keep the waypoint spacing.</summary>
        Spacing = 1 << 4,
    }

    /// <summary>
    /// Per request tuning of the path refinement (protocol 3). Negative values use the server's defaults
    /// (fWallMargin, fCornerRadius, iCornerPoints).
    /// </summary>
    /// <param name="WallMargin">Distance of corners from the walls they bend around (<see cref="PathFlags.AvoidWalls"/>).</param>
    /// <param name="CornerRadius">Radius of the arcs (<see cref="PathFlags.RoundCorners"/>).</param>
    /// <param name="CornerPoints">Points per arc.</param>
    /// <param name="StopDistance">End the path this far before the target, at a point that sees it (melee or casting
    /// range). 0 = at the target.</param>
    /// <param name="WaypointSpacing"><see cref="PathFlags.Detail"/>: longest distance between two waypoints, 0 =
    /// server default (protocol 4).</param>
    /// <param name="WaypointHeightError"><see cref="PathFlags.Detail"/>: how far the surface may leave the line
    /// between two waypoints, 0 = server default (protocol 4).</param>
    public readonly record struct PathTuning(float WallMargin = -1f, float CornerRadius = -1f, int CornerPoints = -1,
                                             float StopDistance = 0f, float WaypointSpacing = 0f,
                                             float WaypointHeightError = 0f);

    /// <summary>Result of <see cref="AmeisenNavClient.GetPathEx"/>.</summary>
    /// <param name="Points">Path points (WoW coordinates).</param>
    /// <param name="Flags">One entry per point.</param>
    /// <param name="IsPartial">The end isn't reachable, the path leads as close as possible.</param>
    /// <param name="Areas"><see cref="PathFlags.SurfaceInfo"/>: navmesh area id under every point (0 = none), null
    /// if not requested or the path was smoothed with Chaikin / Catmull-Rom / Bezier.</param>
    /// <param name="Info"><see cref="PathFlags.SurfaceInfo"/>: what the ground does at every point, null like
    /// <paramref name="Areas"/>.</param>
    public sealed record PathResult(Vector3[] Points, PathPointFlags[] Flags, bool IsPartial, byte[]? Areas = null,
                                    PathPointInfo[]? Info = null);
}
