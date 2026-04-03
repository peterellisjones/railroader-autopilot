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

            foreach (var carId in matchingCarIds)
            {
                var car = matchingCars[carId];
                var wrapped = (ICar)new CarAdapter(car);
                var chain = GetCoupledChain(wrapped);

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

                List<ICar> orderedChain = null;
                List<ICar> targets = null;
                foreach (var orientation in chainOrientations)
                {
                    var t = FilterAccessibleTargets(orientation, matchingCarIds);
                    if (t.Count > 0)
                    {
                        orderedChain = orientation;
                        targets = t;
                        break;
                    }
                }
                if (orderedChain == null || targets == null || targets.Count == 0)
                    continue;

                var coupleTarget = (orderedChain[0] as CarAdapter)?.Car;
                if (coupleTarget == null)
                    continue;

                // Couple location: 0.5m past the couple target on the accessible side.
                var target = orderedChain[0];
                var coupled = _trainService.GetCoupled(loco);
                float trainLen = _trainService.GetTrainLength(loco);

                var endsToTry = new[] { Car.LogicalEnd.A, Car.LogicalEnd.B };
                // For coupled chains, only try the free end
                if (orderedChain.Count > 1)
                {
                    var nextInChain = orderedChain[1];
                    var freeEnd = target.CoupledTo(Car.LogicalEnd.A)?.id == nextInChain.id
                        ? Car.LogicalEnd.B : Car.LogicalEnd.A;
                    endsToTry = new[] { freeEnd };
                }

                DirectedPosition coupleLocation = default;
                float distance = float.MaxValue;
                bool reachable = false;

                foreach (var logicalEnd in endsToTry)
                {
                    var coupleLoc = Services.CoupleLocationCalculator.GetCoupleLocationForEnd(target, logicalEnd, graph);
                    var testLoc = coupleLoc.ToLocation();

                    var routeResult = _trainService.GraphDistanceToLoco(loco, coupleLoc);
                    float routeDist = routeResult?.Distance ?? float.MaxValue;
                    if (routeResult?.BlockedByCars == true)
                        routeDist = float.MaxValue;

                    // Penalize waypoints that crossed onto a different segment —
                    // they may land on a dead-end siding the AE can't reach.
                    bool crossedSegment = testLoc.segment != null && target.EndA.Segment != null
                        && testLoc.segment.id != target.EndA.Segment.id
                        && testLoc.segment.id != target.EndB.Segment.id;
                    if (crossedSegment)
                        routeDist += 1000f;

                    Log($"Trying {target.DisplayName} end {logicalEnd}: waypoint on {testLoc.segment?.id}|{testLoc.distance:F1}, dist={routeDist:F0}{(crossedSegment ? " (penalized)" : "")}");
                    if (routeDist < distance)
                    {
                        coupleLocation = DirectedPosition.FromLocation(testLoc);
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
