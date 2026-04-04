using System.Collections.Generic;
using System.Linq;
using Model;
using Model.AI;
using Track;
using Autopilot.Model;
using Autopilot.Services;

namespace Autopilot.Planning
{
    public class PickupPlanner
    {
        private readonly TrainService _trainService;

        public PickupPlanner(TrainService trainService)
        {
            _trainService = trainService;
        }

        private void Log(string msg) => Loader.Mod.Logger.Log($"Autopilot Pickup: {msg}");

        /// <summary>
        /// Given a chain of cars ordered from approach end to buffer end,
        /// return consecutive target cars from the approach end.
        /// If the approach-end car is NOT a target, returns empty (blocked).
        /// </summary>
        public static List<ICar> FilterAccessibleTargets(
            List<ICar> chainFromApproach, HashSet<string> targetCarIds)
        {
            var targets = new List<ICar>();
            if (chainFromApproach.Count == 0)
                return targets;

            if (!targetCarIds.Contains(chainFromApproach[0].id))
                return targets;

            foreach (var car in chainFromApproach)
            {
                if (!targetCarIds.Contains(car.id))
                    break;
                targets.Add(car);
            }

            return targets;
        }

        /// <summary>
        /// Walk a car's coupled chain. Returns all cars ordered from
        /// the end with a free LogicalEnd.A to the end with a free LogicalEnd.B.
        /// </summary>
        public static List<ICar> GetCoupledChain(ICar startCar)
        {
            var current = startCar;
            while (current.CoupledTo(Car.LogicalEnd.A) != null)
                current = current.CoupledTo(Car.LogicalEnd.A)!;

            var chain = new List<ICar>();
            var visited = new HashSet<string>();
            while (current != null && !visited.Contains(current.id))
            {
                visited.Add(current.id);
                chain.Add(current);
                current = current.CoupledTo(Car.LogicalEnd.B);
            }

            return chain;
        }

        /// <summary>
        /// Scan uncoupled cars reachable from the loco. Return distinct
        /// destination names (sorted) for cars with waybills.
        /// </summary>
        public List<string> GetReachableDestinations(BaseLocomotive loco)
        {
            var nearbyCars = _trainService.GetNearbyCars(loco);
            var destinations = new HashSet<string>();

            foreach (var car in nearbyCars)
            {
                var waybill = _trainService.GetWaybill(car);
                if (waybill == null) continue;
                if (waybill.Value.Completed) continue;

                var destName = waybill.Value.Destination.DisplayName;
                if (string.IsNullOrEmpty(destName)) continue;

                destinations.Add(destName);
            }

            var result = destinations.ToList();
            result.Sort();
            Log($"Found {result.Count} reachable destination(s)");
            return result;
        }

        /// <summary>
        /// Find the nearest accessible car (or group) bound for the given
        /// destination. Returns null if no more targets are reachable.
        /// </summary>
        public PickupTarget? FindNextPickup(BaseLocomotive loco, string destinationName,
            HashSet<Car>? skippedCars = null)
        {
            _trainService.ClearPlanCaches();
            var nearbyCars = _trainService.GetNearbyCars(loco);
            Log($"GetNearbyCars returned {nearbyCars.Count} cars");
            var graph = Graph.Shared;

            // Find all uncoupled cars with matching waybill destination
            var matchingCarIds = new HashSet<string>();
            var matchingCars = new Dictionary<string, Car>();
            foreach (var car in nearbyCars)
            {
                var waybill = _trainService.GetWaybill(car);
                if (waybill == null) continue;
                if (waybill.Value.Completed) continue;
                if (skippedCars != null && skippedCars.Contains(car)) continue;
                if (waybill.Value.Destination.DisplayName != destinationName) continue;
                matchingCarIds.Add(car.id);
                matchingCars[car.id] = car;
                Log($"Matching car: {car.DisplayName} on seg={car.LocationA.segment?.id}, visible={car.IsVisible}");
            }

            if (matchingCarIds.Count == 0)
            {
                Log($"No cars found for destination '{destinationName}'");
                return null;
            }

            Log($"Found {matchingCarIds.Count} car(s) for '{destinationName}'");

            var candidates = new List<(PickupTarget target, float distance)>();
            var processedChains = new HashSet<string>();

            // Build segment→cars map for blocking checks.
            // RouteSearch ignores coupled groups (only uncoupled cars are obstacles),
            // so we must independently check route segments for any non-target cars.
            var segmentToCars = new Dictionary<string, List<Car>>();
            foreach (var car in nearbyCars)
            {
                var segA = car.LocationA.segment;
                var segB = car.LocationB.segment;
                if (segA != null)
                {
                    if (!segmentToCars.TryGetValue(segA.id, out var listA))
                    {
                        listA = new List<Car>();
                        segmentToCars[segA.id] = listA;
                    }
                    listA.Add(car);
                }
                if (segB != null && segB != segA)
                {
                    if (!segmentToCars.TryGetValue(segB.id, out var listB))
                    {
                        listB = new List<Car>();
                        segmentToCars[segB.id] = listB;
                    }
                    listB.Add(car);
                }
            }

            foreach (var carId in matchingCarIds)
            {
                var car = matchingCars[carId];
                var wrapped = (ICar)new CarAdapter(car);
                var chain = GetCoupledChain(wrapped);
                var chainIds = new HashSet<string>();
                foreach (var c in chain) chainIds.Add(c.id);

                var chainKey = chain[0].id;
                if (processedChains.Contains(chainKey))
                    continue;
                processedChains.Add(chainKey);

                // Determine approach end: which end of the chain is closer to the loco?
                var firstCar = chain[0];
                var lastCar = chain[chain.Count - 1];

                float distToFirst = _trainService.GraphDistanceToLoco(loco, firstCar.EndA)?.Distance ?? float.MaxValue;
                float distToLast = _trainService.GraphDistanceToLoco(loco, lastCar.EndB)?.Distance ?? float.MaxValue;

                // Try both orientations — the closer end might face a buffer stop
                var chainOrientations = new List<List<ICar>>();
                if (distToFirst <= distToLast)
                {
                    chainOrientations.Add(chain);
                    chainOrientations.Add(Enumerable.Reverse(chain).ToList());
                }
                else
                {
                    chainOrientations.Add(Enumerable.Reverse(chain).ToList());
                    chainOrientations.Add(chain);
                }

                // Try both orientations — each end of the chain is a
                // separate candidate.  When one end is blocked the other
                // may be reachable from a different approach direction.
                var coupled = _trainService.GetCoupled(loco);
                float trainLen = _trainService.GetTrainLength(loco);

                foreach (var orientation in chainOrientations)
                {
                    var targets = FilterAccessibleTargets(orientation, matchingCarIds);
                    if (targets.Count == 0)
                        continue;

                    var coupleTarget = (orientation[0] as CarAdapter)?.Car;
                    if (coupleTarget == null)
                        continue;

                    var target = orientation[0];

                    var endsToTry = new[] { Car.LogicalEnd.A, Car.LogicalEnd.B };
                    if (orientation.Count > 1)
                    {
                        var nextInChain = orientation[1];
                        var freeEnd = target.CoupledTo(Car.LogicalEnd.A)?.id == nextInChain.id
                            ? Car.LogicalEnd.B : Car.LogicalEnd.A;
                        endsToTry = new[] { freeEnd };
                    }

                    CoupleWaypoint coupleLocation = default;
                    float distance = float.MaxValue;
                    bool reachable = false;

                    foreach (var logicalEnd in endsToTry)
                    {
                        var coupleWp = Services.CoupleLocationCalculator.GetCoupleLocationForEnd(target, logicalEnd, graph);
                        var testLoc = coupleWp.ToLocation();

                        var ignored = new System.Collections.Generic.HashSet<Car>(coupled);
                        float routeDist = float.MaxValue;
                        foreach (var locoLoc in new[] { loco.LocationF, loco.LocationR })
                        {
                            var impasse = new System.Collections.Generic.HashSet<Car>();
                            var routeSteps = new System.Collections.Generic.List<Track.Search.RouteSearch.Step>();
                            bool routeFound = Track.Search.RouteSearch.FindRoute(
                                graph, locoLoc, testLoc,
                                RouteChecker.DefaultHeuristic, routeSteps,
                                out Track.Search.RouteSearch.Metrics routeMetrics,
                                checkForCars: true, trainLength: trainLen,
                                trainMomentum: 0f, maxIterations: 5000,
                                checkForCarsIgnored: ignored,
                                checkForCarsImpasse: impasse);
                            // RouteSearch with checkForCars only detects uncoupled
                            // cars; coupled groups are invisible. Check the actual
                            // route segments for any non-target car that would block.
                            if (routeFound)
                            {
                                var routeSegs = ReversalCounter.DeduplicateSegments(routeSteps);
                                var terminalSegId = testLoc.segment?.id;

                                // Determine approach direction on the terminal segment
                                // from the couple point vs target car center.
                                // The couple point is offset past the car's free end
                                // in the approach direction, so cpDist < carCenter
                                // means approach from End.A (low distance).
                                bool? approachFromEndA = null;
                                if (terminalSegId != null && coupleTarget.LocationA.segment?.id == terminalSegId
                                    && coupleTarget.LocationB.segment?.id == terminalSegId)
                                {
                                    float carCenter = (coupleTarget.LocationA.distance + coupleTarget.LocationB.distance) / 2f;
                                    approachFromEndA = testLoc.distance < carCenter;
                                }

                                // Verify the route enters the terminal segment from
                                // the correct side. RouteSearch ignores coupled cars,
                                // so it may find a route that enters from the far end —
                                // requiring the loco to pass through the chain itself.
                                if (approachFromEndA.HasValue && routeSegs.Count >= 2)
                                {
                                    var prevSeg = routeSegs[routeSegs.Count - 2];
                                    var termSeg = routeSegs[routeSegs.Count - 1];
                                    if (termSeg.id == terminalSegId)
                                    {
                                        var sharedNode = Services.TrackWalker.FindSharedNode(prevSeg, termSeg);
                                        if (sharedNode != null)
                                        {
                                            bool entersFromEndA = (termSeg.NodeForEnd(TrackSegment.End.A) == sharedNode);
                                            if (entersFromEndA != approachFromEndA.Value)
                                            {
                                                Log($"  Route to {target.DisplayName} enters {terminalSegId} from wrong end");
                                                routeFound = false;
                                            }
                                        }
                                    }
                                }

                                foreach (var seg in routeSegs)
                                {
                                    if (!routeFound) break;
                                    if (segmentToCars.TryGetValue(seg.id, out var carsOnSeg))
                                    {
                                        foreach (var blockCar in carsOnSeg)
                                        {
                                            if (chainIds.Contains(blockCar.id))
                                                continue;

                                            // On the terminal segment, only block if the car
                                            // is between the approach end and the couple point.
                                            if (seg.id == terminalSegId && approachFromEndA.HasValue)
                                            {
                                                float cpDist = testLoc.distance;
                                                float carMinDist = float.MaxValue;
                                                float carMaxDist = float.MinValue;
                                                if (blockCar.LocationA.segment?.id == seg.id)
                                                {
                                                    carMinDist = blockCar.LocationA.distance;
                                                    carMaxDist = blockCar.LocationA.distance;
                                                }
                                                if (blockCar.LocationB.segment?.id == seg.id)
                                                {
                                                    carMinDist = System.Math.Min(carMinDist, blockCar.LocationB.distance);
                                                    carMaxDist = System.Math.Max(carMaxDist, blockCar.LocationB.distance);
                                                }
                                                if (carMinDist == float.MaxValue)
                                                    continue;

                                                bool inPath = approachFromEndA.Value
                                                    ? carMinDist < cpDist
                                                    : carMaxDist > cpDist;

                                                if (!inPath)
                                                    continue;
                                            }

                                            Log($"  Route to {target.DisplayName} blocked by {blockCar.DisplayName} on {seg.id}");
                                            routeFound = false;
                                            break;
                                        }
                                    }
                                    if (!routeFound) break;
                                }
                            }
                            if (routeFound && routeMetrics.Distance < routeDist)
                                routeDist = routeMetrics.Distance;
                        }

                        Log($"Trying {target.DisplayName} end {logicalEnd}: waypoint on {testLoc.segment?.id}|{testLoc.distance:F1}, dist={routeDist:F0}");
                        if (routeDist < distance)
                        {
                            coupleLocation = coupleWp;
                            distance = routeDist;
                            reachable = true;
                        }
                    }

                    if (!reachable)
                        continue;

                    var targetCars = targets.Select(c => (c as CarAdapter)?.Car).Where(c => c != null).ToList()!;
                    candidates.Add((new PickupTarget(
                        coupleTarget, coupleLocation, targetCars!, destinationName), distance));
                }
            }

            if (candidates.Count == 0)
            {
                Log($"No accessible targets for '{destinationName}'");
                return null;
            }

            candidates.Sort((a, b) => a.distance.CompareTo(b.distance));

            var best = candidates[0];
            Log($"Next pickup: {best.target.CoupleTarget.DisplayName} at distance {best.distance:F0}m " +
                $"({best.target.TargetCars.Count} target car(s))");
            return best.target;
        }

    }
}
