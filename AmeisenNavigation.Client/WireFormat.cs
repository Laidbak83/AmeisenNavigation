using System.Runtime.InteropServices;

namespace AmeisenNavigation.Client
{
    [StructLayout(LayoutKind.Sequential)]
    internal struct PathRequestData
    {
        public int MapId;
        public int Flags;
        public Vector3 Start;
        public Vector3 End;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct MoveRequestData
    {
        public int MapId;
        public Vector3 Start;
        public Vector3 End;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct CastRayData
    {
        public int MapId;
        public Vector3 Start;
        public Vector3 End;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct CastRayExResponse
    {
        public int Hit;
        public float Fraction;
        public Vector3 Position;
        public Vector3 Normal;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct RandomPointAroundData
    {
        public int MapId;
        public Vector3 Start;
        public float Radius;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct ExplorePolyRequestHeader
    {
        public int MapId;
        public int Flags;
        public Vector3 Start;
        public float Spacing;
        public int PointCount;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct GetHeightData
    {
        public int MapId;
        public Vector3 Position;
    }

    // Protocol 3: must match AmeisenNavigation.Server/src/Protocol.hpp (sizes checked by ClientIntegration).

    [StructLayout(LayoutKind.Sequential)]
    internal struct PathTuningData
    {
        public float WallMargin;
        public float CornerRadius;
        public int CornerPoints;
        public float StopDistance;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct PathRequestWithTuning
    {
        public PathRequestData Request;
        public PathTuningData Tuning;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct PathExResponseHeader
    {
        public int PointCount;
        public int ResultFlags;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct AvoidZonesHeader
    {
        public int MapId;
        public int ZoneCount;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct AvoidZoneData
    {
        public Vector3 Center;
        public float Radius;
        public float Cost;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct SteerRequestHeader
    {
        public int MapId;
        public Vector3 Position;
        public Vector3 Velocity;
        public Vector3 Target;
        public float MaxSpeed;
        public float Radius;
        public float DeltaTime;
        public int UnitCount;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct SteerUnitData
    {
        public Vector3 Position;
        public Vector3 Velocity;
        public float Radius;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct SteerResponse
    {
        public int Ok;
        public Vector3 Velocity;
        public Vector3 Position;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct ReportStuckData
    {
        public int MapId;
        public Vector3 Position;
        public float Radius;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct LineOfSightData
    {
        public int MapId;
        public int Flags;
        public Vector3 Start;
        public Vector3 End;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct SweepData
    {
        public int MapId;
        public int Flags;
        public Vector3 Start;
        public Vector3 End;
        public float Radius;
        public float Height;
        public float StepHeight;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct WorldHitResponse
    {
        public int Result;
        public float T;
        public Vector3 Position;
        public Vector3 Normal;
        public uint Source;
        public int Area;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct SurfacesRequestData
    {
        public int MapId;
        public int Flags;
        public float X;
        public float Y;
        public float ZTop;
        public float ZBottom;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct SurfacesResponseHeader
    {
        public int Count;
        public uint AreaId;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct SurfaceData
    {
        public float Z;
        public Vector3 Normal;
        public uint Source;
        public byte Area;
        public byte Flags;
        public ushort Reserved;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct SetObjectsHeader
    {
        public int MapId;
        public int Count;
        public int RemoveCount;
        public int Reserved;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct ObjectData
    {
        public ulong Guid;
        public uint DisplayId;
        public uint Flags;
        public Vector3 Position;
        public float RotationX;
        public float RotationY;
        public float RotationZ;
        public float RotationW;
        public float Scale;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal struct SetObjectsResponse
    {
        public int Accepted;
        public int PendingTiles;
    }

    /// <summary>Wire struct sizes, compared with the server's by the integration test.</summary>
    internal static class WireSizes
    {
        public static readonly (string Name, int Size, int Expected)[] All =
        [
            (nameof(PathRequestData), Marshal.SizeOf<PathRequestData>(), 32),
            (nameof(PathTuningData), Marshal.SizeOf<PathTuningData>(), 16),
            (nameof(PathExResponseHeader), Marshal.SizeOf<PathExResponseHeader>(), 8),
            (nameof(AvoidZonesHeader), Marshal.SizeOf<AvoidZonesHeader>(), 8),
            (nameof(AvoidZoneData), Marshal.SizeOf<AvoidZoneData>(), 20),
            (nameof(SteerRequestHeader), Marshal.SizeOf<SteerRequestHeader>(), 56),
            (nameof(SteerUnitData), Marshal.SizeOf<SteerUnitData>(), 28),
            (nameof(SteerResponse), Marshal.SizeOf<SteerResponse>(), 28),
            (nameof(ReportStuckData), Marshal.SizeOf<ReportStuckData>(), 20),
            (nameof(LineOfSightData), Marshal.SizeOf<LineOfSightData>(), 32),
            (nameof(SweepData), Marshal.SizeOf<SweepData>(), 44),
            (nameof(WorldHitResponse), Marshal.SizeOf<WorldHitResponse>(), 40),
            (nameof(SurfacesRequestData), Marshal.SizeOf<SurfacesRequestData>(), 24),
            (nameof(SurfacesResponseHeader), Marshal.SizeOf<SurfacesResponseHeader>(), 8),
            (nameof(SurfaceData), Marshal.SizeOf<SurfaceData>(), 24),
            (nameof(SetObjectsHeader), Marshal.SizeOf<SetObjectsHeader>(), 16),
            (nameof(ObjectData), Marshal.SizeOf<ObjectData>(), 48),
            (nameof(SetObjectsResponse), Marshal.SizeOf<SetObjectsResponse>(), 8),
        ];
    }
}
