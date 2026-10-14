using System;
using System.Linq;
using AmeisenNavigation.Client;

// Runs the C# client against a server serving the synthetic test world (tests/TestWorld.hpp, map 1).
// Usage: ClientIntegration <port>

if (args.Length < 1 || !int.TryParse(args[0], out int port))
{
    Console.WriteLine("Usage: ClientIntegration <port>");
    return 2;
}

const int map = 1;
int failures = 0;

// RD -> WoW: (x, y, z)_wow = (rdZ, rdX, rdY), same helper as the C++ tests.
static Vector3 Wow(float x, float y, float z) => new(z, x, y);

void Check(bool ok, string what)
{
    Console.WriteLine($"{(ok ? "OK  " : "FAIL")} {what}");
    if (!ok) failures++;
}

using var nav = new AmeisenNavClient("127.0.0.1", port) { TimeoutMs = 10000 };

if (!nav.TryConnect())
{
    Console.WriteLine("connect failed");
    return 1;
}

var cfg = nav.GetConfig();
Check(cfg is { UseAnpFileFormat: true, ProtocolVersion: >= 2, MaxPointPath: > 0 } && cfg.ServerVersion.Length > 0,
      $"GetConfig {cfg}");

nav.SetClientState(ClientState.NormalHorde);
nav.SetAreaCosts(1f, 0.75f, 1.6f, 4f, allyMult: 3f, hordeMult: 1f);
Check(nav.ApplyFilter() && !nav.IsFilterDirty, "ApplyFilter (ANP costs)");

var start = Wow(-700, 0, -300);
var end = Wow(-200, 0, -300);
var path = nav.GetPath(map, start, end);
Check(path is { Length: >= 3 } && path[^1].DistanceTo(end) < 1.5f, $"GetPath points={path?.Length}");

foreach (var flags in new[] { PathFlags.SmoothChaikin, PathFlags.SmoothCatmullRom | PathFlags.ValidateMoveAlongSurface,
                              PathFlags.SmoothBezier | PathFlags.ValidateClosestPointOnPoly, PathFlags.RequireComplete })
{
    var p = nav.GetPath(map, start, end, flags);
    Check(p is { Length: >= 2 }, $"GetPath {flags} points={p?.Length}");
}

Check(nav.GetRandomPath(map, start, end) is { Length: >= 2 }, "GetRandomPath");

// The platform deck is an island: partial path by default, null with RequireComplete.
var ground = Wow(-600, 0, -425);
var deck = Wow(-425, 5, -425);
Check(nav.GetPath(map, ground, deck) is { Length: >= 2 }, "GetPath to unreachable deck (partial)");
Check(nav.GetPath(map, ground, deck, PathFlags.RequireComplete) == null, "GetPath RequireComplete -> null");

bool clear = nav.CastRay(map, Wow(-400, 0, -300), Wow(-200, 0, -300), out var hitPoint);
Check(!clear && hitPoint.IsZero, "CastRay blocked (legacy)");
var hit = nav.CastRayEx(map, Wow(-400, 0, -300), Wow(-200, 0, -300));
Check(hit.IsValid && hit.IsHit && Math.Abs(hit.Position.Y - (-300)) < 4, $"CastRayEx {hit}");
var open = nav.CastRayEx(map, Wow(-400, 0, -50), Wow(-200, 0, -50));
Check(open.IsClear && open.Fraction == 1f, $"CastRayEx clear {open}");

var h = nav.GetHeight(map, Wow(-425, 6, -425));
Check(Math.Abs(h.Z - 5) < 0.6, $"GetHeight deck {h}");
Check(!nav.GetRandomPoint(map).IsZero, "GetRandomPoint");
Check(nav.GetRandomPointAround(map, start, 15f).DistanceTo(start) < 60f, "GetRandomPointAround");
Check(nav.MoveAlongSurface(map, Wow(-320, 0, -300), Wow(-280, 0, -300)).Y < -300, "MoveAlongSurface blocked");

var outline = new[] { Wow(-390, 0, -450), Wow(-210, 0, -450), Wow(-210, 0, -150), Wow(-390, 0, -150) };
var route = nav.ExplorePolygon(map, Wow(-380, 0, -440), outline, 40f);
Check(route is { Length: > 20 } && route.Length <= cfg!.MaxPointPath && route.All(p => Math.Abs(p.Z) < 0.6f),
      $"ExplorePolygon points={route?.Length}");
Check(nav.ExplorePolygon(map, start, outline.Take(2).ToArray(), 40f) == null, "ExplorePolygon invalid -> null");

Check(nav.GetPath(4242, Wow(0, 0, 0), Wow(1, 0, 1)) == null, "unknown map -> null");

// ── Protocol 3 ──

foreach (var (name, size, expected) in WireSizes.All)
    Check(size == expected, $"wire size {name} {size} == {expected}");

Check(cfg is { ProtocolVersion: >= 3 }, $"protocol {cfg?.ProtocolVersion}");

var fluid = nav.GetPath(map, start, end, PathFlags.Fluid);
Check(fluid is { Length: >= 3 } && fluid[^1].DistanceTo(end) < 1.5f, $"GetPath Fluid points={fluid?.Length}");
var stop = nav.GetPath(map, start, end, PathFlags.None, new PathTuning(StopDistance: 10f));
Check(stop != null && stop[^1].DistanceTo(end) <= 10.1f && stop[^1].DistanceTo(end) > 8f, "GetPath StopDistance");
Check(nav.GetRandomPath(map, start, end, PathFlags.Fluid, new PathTuning(WallMargin: 2f)) is { Length: >= 2 },
      "GetRandomPath tuned");
var tunedRoute = nav.ExplorePolygon(map, Wow(-380, 0, -440), outline, 40f, PathFlags.RoundCorners, new PathTuning(CornerRadius: 3f));
Check(tunedRoute is { Length: > 20 }, $"ExplorePolygon tuned points={tunedRoute?.Length}");

var follow1 = nav.GetPath(map, start, end, PathFlags.Follow);
var follow2 = follow1 == null ? null : nav.GetPath(map, follow1[0] + (follow1[1] - follow1[0]) * (2f / follow1[0].DistanceTo(follow1[1])), end, PathFlags.Follow);
Check(follow2 != null && follow2.Length == follow1!.Length && follow2[1].DistanceTo(follow1[1]) < 0.01f, "GetPath Follow keeps corners");

// The server adds drop links (fAutoDropHeight): off the deck with a drop.
var ex = nav.GetPathEx(map, deck, ground);
Check(ex != null && !ex.IsPartial && ex.Flags[0].HasFlag(PathPointFlags.Start) && ex.Flags[^1].HasFlag(PathPointFlags.End)
      && ex.Flags.Count(f => f.HasFlag(PathPointFlags.Drop)) == 1, $"GetPathEx drop {ex?.Points.Length}");
var exUp = nav.GetPathEx(map, ground, deck);
Check(exUp is { IsPartial: true }, "GetPathEx partial");
Check(nav.GetPathEx(map, ground, deck, PathFlags.RequireComplete) == null, "GetPathEx RequireComplete -> null");

// Avoid zones: a zone in the open, gone around.
var a = Wow(-700, 0, -50);
var b = Wow(-400, 0, -50);
var center = Wow(-550, 0, -50);
Check(nav.SetAvoidZones(map, [new AvoidZone(center, 15f, 20f)]), "SetAvoidZones");
var around = nav.GetPath(map, a, b);
bool clearOfZone = around != null;

for (int i = 1; around != null && i < around.Length; i++)
{
    for (int k = 0; k <= 20; k++)
    {
        var p = around[i - 1] + (around[i] - around[i - 1]) * (k / 20f);
        clearOfZone &= p.DistanceTo2D(center) >= 14.9f;
    }
}

Check(clearOfZone, "path avoids the zone");
Check(nav.ClearAvoidZones(map), "ClearAvoidZones");

// Terrain: no water.
nav.SetCanSwim(false);
Check(nav.ApplyFilter(), "ApplyFilter with terrain costs");
var dry = nav.GetPath(map, Wow(-140, 0, -430), Wow(-40, 0, -430));
bool wet = false;

for (int i = 1; dry != null && i < dry.Length; i++)
{
    for (int k = 0; k <= 20; k++)
    {
        var p = dry[i - 1] + (dry[i] - dry[i - 1]) * (k / 20f);
        wet |= p.Y > -118 && p.Y < -62 && p.X > -458 && p.X < -402;
    }
}

Check(dry != null && !wet, "SetCanSwim(false) avoids the pool");
nav.SetCanSwim(true);
Check(nav.ApplyFilter(), "ApplyFilter back to defaults");

var steer = nav.Steer(map, Wow(-600, 0, -100), Wow(7, 0, 0), Wow(-500, 0, -100), 7f, 0.6f, 0.1f,
                      [new SteerUnit(Wow(-596, 0, -100), Wow(-7, 0, 0), 0.6f)]);
Check(steer.IsValid && Math.Abs(steer.Velocity.X) > 1f, $"Steer sidesteps {steer}");
Check(nav.ReportStuck(map, Wow(-650, 0, -150), 5f), "ReportStuck");

Console.WriteLine(failures == 0 ? "ALL OK" : $"{failures} FAILED");
return failures == 0 ? 0 : 1;
