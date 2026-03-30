using System.Collections.Generic;
using Model;
using Model.Ops;
using Track;
using Autopilot.Model;
using Autopilot.Services;
using Autopilot.Patches;

namespace Autopilot.Planning
{
    public class DestinationSelector
    {
        private const int MaxSpanWalkSegments = 20;

        private readonly TrainService _trainService;
        private readonly RouteChecker _routeChecker;

        public DestinationSelector(TrainService trainService, RouteChecker routeChecker)
        {
            _trainService = trainService;
            _routeChecker = routeChecker;
        }

        /// <summary>
        /// Get candidate delivery locations for a destination, prioritized:
        ///   1. Spans with completed-waybill cars (couple next to them)
        ///   2. Empty spans (push to buffer end)
        ///   3. Spans with pending-waybill cars (avoid blocking their exit)
        /// Within each tier, sorted by distance (closest first).
        /// </summary>
        public List<(DirectedPosition loc, Car coupleTo)> GetDestinationCandidates(OpsCarPosition destination, BaseLocomotive loco)
        {
            var graph = Graph.Shared;
            var coupled = new HashSet<Car>(_trainService.GetCoupled(loco));

            // priority: 0 = completed cars (best), 1 = empty, 2 = pending cars (avoid)
            var candidates = new List<(DirectedPosition loc, float dist, int priority, Car coupleTo)>();

            Loader.Mod.Logger.Log($"Autopilot GetDestinationCandidates: {destination.DisplayName} has {destination.Spans.Length} span(s)");
            foreach (var span in destination.Spans)
            {
                var lower = span.lower;
                var upper = span.upper;

                Loader.Mod.Logger.Log($"  span: lower={lower?.segment?.id}|{lower?.distance:F1}, upper={upper?.segment?.id}|{upper?.distance:F1}");

                if (lower == null && upper == null)
                    continue;

                // Collect all segments within this span for the car search
                var spanSegIds = GetSpanSegmentIds(lower, upper, graph);
                if (Loader.Settings?.verboseLogging == true)
                    Loader.Mod.Logger.Log($"  span segments: [{string.Join(", ", spanSegIds)}]");

                // Find the nearest existing car anywhere within this span
                Car nearestCar = null;
                float nearestCarDist = float.MaxValue;
                bool hasPendingWaybill = false;

                foreach (var car in TrainController.Shared.Cars)
                {
                    if (coupled.Contains(car)) continue;

                    var segA = car.LocationA.segment?.id;
                    var segB = car.LocationB.segment?.id;
                    if ((segA == null || !spanSegIds.Contains(segA)) &&
                        (segB == null || !spanSegIds.Contains(segB)))
                        continue;

                    bool completed = car.Waybill.HasValue && car.Waybill.Value.Completed;
                    // Only cars with an actual pending waybill (not yet completed)
                    // count as pending. Cars with no waybill (tenders, locos for
                    // sale) are just parked — they don't block the span.
                    if (car.Waybill.HasValue && !car.Waybill.Value.Completed)
                        hasPendingWaybill = true;

                    if (Loader.Settings?.verboseLogging == true)
                        Loader.Mod.Logger.Log($"  found car {car.DisplayName} on seg={segA ?? segB}, " +
                            $"waybill={car.Waybill.HasValue}, completed={completed}");

                    var carPosA = DirectedPosition.FromLocation(car.LocationA);
                    var carPosB = DirectedPosition.FromLocation(car.LocationB);
                    var resultA = _routeChecker.GraphDistanceToLoco(loco, carPosA);
                    var resultB = _routeChecker.GraphDistanceToLoco(loco, carPosB);
                    float dA = resultA?.Distance ?? float.MaxValue;
                    float dB = resultB?.Distance ?? float.MaxValue;
                    float d = System.Math.Min(dA, dB);
                    if (d < nearestCarDist)
                    {
                        nearestCarDist = d;
                        nearestCar = car;
                    }
                }

                // If there's an existing car, offer a coupling candidate
                if (nearestCar != null)
                {
                    var carPosA = DirectedPosition.FromLocation(nearestCar.LocationA);
                    var carPosB = DirectedPosition.FromLocation(nearestCar.LocationB);
                    var resultA2 = _routeChecker.GraphDistanceToLoco(loco, carPosA);
                    var resultB2 = _routeChecker.GraphDistanceToLoco(loco, carPosB);
                    float dA2 = resultA2?.Distance ?? float.MaxValue;
                    float dB2 = resultB2?.Distance ?? float.MaxValue;
                    var nearCarEndLoc = dA2 <= dB2 ? nearestCar.LocationA : nearestCar.LocationB;
                    var coupleLoc = graph.LocationByMoving(nearCarEndLoc, -1f, false, true).Flipped();
                    var couplePos = DirectedPosition.FromLocation(coupleLoc);
                    int priority = hasPendingWaybill ? 2 : 0;
                    float dist = _routeChecker.GraphDistanceToLoco(loco, couplePos)?.Distance ?? float.MaxValue;
                    candidates.Add((couplePos, dist, priority, nearestCar));
                }

                // Also offer span endpoints as empty-span candidates
                var endpoints = new List<DirectedPosition>();
                if (lower != null) endpoints.Add(DirectedPosition.FromLocation(lower.Value));
                if (upper != null && (lower == null || upper.Value.segment != lower.Value.segment))
                    endpoints.Add(DirectedPosition.FromLocation(upper.Value));

                foreach (var endpoint in endpoints)
                {
                    float dist = _routeChecker.GraphDistanceToLoco(loco, endpoint)?.Distance ?? float.MaxValue;
                    candidates.Add((endpoint, dist, 1, null));
                }
            }

            // Sort by priority first, then distance within each tier
            candidates.Sort((a, b) =>
            {
                if (a.priority != b.priority)
                    return a.priority.CompareTo(b.priority);
                return a.dist.CompareTo(b.dist);
            });
            return candidates.ConvertAll(c => (c.loc, c.coupleTo));
        }

        public DirectedPosition GetDestinationLocation(OpsCarPosition destination, BaseLocomotive loco)
        {
            var candidates = GetDestinationCandidates(destination, loco);
            if (candidates.Count > 0)
                return candidates[0].loc;
            return DirectedPosition.FromLocation((Location)destination);
        }

        /// <summary>
        /// Walk the graph from the lower to upper span bound, collecting all
        /// segment IDs in between. Handles switches and multi-segment spans.
        /// </summary>
        private static HashSet<string> GetSpanSegmentIds(Location? lower, Location? upper, Graph graph)
        {
            var ids = new HashSet<string>();
            if (lower?.segment == null) return ids;

            ids.Add(lower.Value.segment.id);

            if (upper?.segment == null || upper.Value.segment == lower.Value.segment)
                return ids;

            var targetId = upper.Value.segment.id;

            // BFS from lower segment toward upper segment
            var queue = new Queue<TrackSegment>();
            queue.Enqueue(lower.Value.segment);

            while (queue.Count > 0 && ids.Count < MaxSpanWalkSegments)
            {
                var seg = queue.Dequeue();
                foreach (var end in new[] { TrackSegment.End.A, TrackSegment.End.B })
                {
                    var node = seg.NodeForEnd(end);
                    if (node == null) continue;

                    var neighbors = new List<TrackSegment>();
                    if (graph.IsSwitch(node))
                    {
                        graph.DecodeSwitchAt(node, out var e, out var n, out var r);
                        if (e != null) neighbors.Add(e);
                        if (n != null) neighbors.Add(n);
                        if (r != null) neighbors.Add(r);
                    }
                    else
                    {
                        GraphPatches.SegmentsReachableFrom(graph, seg, end, out var next, out _);
                        if (next != null) neighbors.Add(next);
                    }

                    foreach (var neighbor in neighbors)
                    {
                        if (neighbor == seg || ids.Contains(neighbor.id)) continue;
                        ids.Add(neighbor.id);
                        if (neighbor.id == targetId) return ids;
                        queue.Enqueue(neighbor);
                    }
                }
            }

            return ids;
        }
    }
}
