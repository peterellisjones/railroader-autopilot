using System.Collections.Generic;
using Model;
using Model.AI;
using Track;
using Track.Search;
using Autopilot.Model;
using Autopilot.Services;

namespace Autopilot.Planning
{
    public class RouteChecker
    {
        private readonly TrainService _trainService;
        private readonly PlanningLogger _logger;
        private Dictionary<string, RouteResult?>? _distanceCache;

        public void ClearCache()
        {
            _distanceCache = null;
        }

        public static HeuristicCosts DefaultHeuristic => new HeuristicCosts
        {
            DivergingRoute = 100,
            ThrowSwitch = 50,
            ThrowSwitchCTCLocked = 1000,
            CarBlockingRoute = 500
        };

        public RouteChecker(TrainService trainService, PlanningLogger logger)
        {
            _trainService = trainService;
            _logger = logger;
        }

        /// <summary>
        /// Check if the loco can route to a destination with its current consist.
        /// </summary>
        public bool CanRouteTo(BaseLocomotive loco, DirectedPosition destination)
        {
            return CanRouteTo(loco, destination, _trainService.GetTrainLength(loco),
                _trainService.GetCoupled(loco));
        }

        /// <summary>
        /// Check if the loco can route to a destination with a specific train length,
        /// ignoring specific cars (e.g., only the cars that will remain attached after a split).
        /// Cars NOT in ignoredCars will be treated as blocking obstacles.
        /// Checks from both LocationF and LocationR since RouteSearch only exits
        /// the starting segment in one direction.
        /// </summary>
        public bool CanRouteTo(BaseLocomotive loco, DirectedPosition destination,
            float trainLength, List<Car> ignoredCars)
        {
            try
            {
                var graph = Graph.Shared;
                var heuristic = DefaultHeuristic;
                var destLoc = destination.ToLocation();

                if (destLoc.segment == null || loco.LocationF.segment == null)
                    return true;

                var ignored = new System.Collections.Generic.HashSet<Car>(ignoredCars);

                // Check from both loco ends — RouteSearch only exits the
                // starting segment in one direction, so after a push delivery
                // + back-away, LocationF may face away from the siding.
                bool foundF = RouteSearch.FindRoute(
                    graph,
                    loco.LocationF,
                    destLoc,
                    heuristic,
                    new List<RouteSearch.Step>(),
                    out _,
                    checkForCars: true,
                    trainLength: trainLength,
                    trainMomentum: 0f,
                    maxIterations: 10000,
                    checkForCarsIgnored: ignored,
                    checkForCarsImpasse: new System.Collections.Generic.HashSet<Car>(),
                    limitSwitchIds: null,
                    enableLogging: false
                );
                if (foundF) return true;

                bool foundR = RouteSearch.FindRoute(
                    graph,
                    loco.LocationR,
                    destLoc,
                    heuristic,
                    new List<RouteSearch.Step>(),
                    out _,
                    checkForCars: true,
                    trainLength: trainLength,
                    trainMomentum: 0f,
                    maxIterations: 10000,
                    checkForCarsIgnored: ignored,
                    checkForCarsImpasse: new System.Collections.Generic.HashSet<Car>(),
                    limitSwitchIds: null,
                    enableLogging: false
                );
                return foundR;
            }
            catch (System.Exception ex)
            {
                _logger.Log("RouteChecker", $"CanRouteTo: exception {ex.GetType().Name}: {ex.Message}");
                _logger.Log("RouteChecker", $"CanRouteTo: stack: {ex.StackTrace}");
                bool nearDest = loco.LocationF.segment == destination.Segment
                             || loco.LocationR.segment == destination.Segment;
                _logger.Log("RouteChecker", $"CanRouteTo: fallback nearDest={nearDest}");
                return nearDest;
            }
        }

        /// <summary>
        /// Like CanRouteTo, but also returns the route steps for safety checking.
        /// Returns (found, routeSteps). If not found, routeSteps is empty.
        /// Checks from both LocationF and LocationR.
        /// </summary>
        public (bool found, List<RouteSearch.Step> steps) CanRouteToWithSteps(
            BaseLocomotive loco, DirectedPosition destination,
            float trainLength, List<Car> ignoredCars)
        {
            try
            {
                var graph = Graph.Shared;
                var heuristic = DefaultHeuristic;
                var destLoc = destination.ToLocation();

                if (destLoc.segment == null || loco.LocationF.segment == null)
                    return (true, new List<RouteSearch.Step>());

                var ignored = new System.Collections.Generic.HashSet<Car>(ignoredCars);

                var stepsF = new List<RouteSearch.Step>();
                bool foundF = RouteSearch.FindRoute(
                    graph,
                    loco.LocationF,
                    destLoc,
                    heuristic,
                    stepsF,
                    out RouteSearch.Metrics metricsF,
                    checkForCars: true,
                    trainLength: trainLength,
                    trainMomentum: 0f,
                    maxIterations: 10000,
                    checkForCarsIgnored: ignored,
                    checkForCarsImpasse: new System.Collections.Generic.HashSet<Car>(),
                    limitSwitchIds: null,
                    enableLogging: false
                );

                var stepsR = new List<RouteSearch.Step>();
                bool foundR = RouteSearch.FindRoute(
                    graph,
                    loco.LocationR,
                    destLoc,
                    heuristic,
                    stepsR,
                    out RouteSearch.Metrics metricsR,
                    checkForCars: true,
                    trainLength: trainLength,
                    trainMomentum: 0f,
                    maxIterations: 10000,
                    checkForCarsIgnored: ignored,
                    checkForCarsImpasse: new System.Collections.Generic.HashSet<Car>(),
                    limitSwitchIds: null,
                    enableLogging: false
                );

                if (foundF && foundR)
                    return metricsF.Distance <= metricsR.Distance ? (true, stepsF) : (true, stepsR);
                if (foundF) return (true, stepsF);
                if (foundR) return (true, stepsR);
                return (false, new List<RouteSearch.Step>());
            }
            catch (System.Exception ex)
            {
                _logger.Log("RouteChecker", $"CanRouteToWithSteps: exception {ex.GetType().Name}: {ex.Message}");
                bool nearDest = loco.LocationF.segment == destination.Segment
                             || loco.LocationR.segment == destination.Segment;
                return (nearDest, new List<RouteSearch.Step>());
            }
        }

        /// <summary>
        /// Route distance from the loco to a location, checking both directions
        /// and accounting for blocking cars. Checks from both LocationF and
        /// LocationR since RouteSearch exits the starting segment in one direction only.
        /// Returns null if no route found in either direction.
        /// </summary>
        public RouteResult? GraphDistanceToLoco(BaseLocomotive loco, DirectedPosition target)
        {
            var targetLoc = target.ToLocation();
            if (targetLoc.segment == null) return null;

            var cacheKey = $"{targetLoc.segment.id}|{targetLoc.distance:F2}";
            _distanceCache ??= new Dictionary<string, RouteResult?>();
            if (_distanceCache.TryGetValue(cacheKey, out var cached))
                return cached;

            var coupled = _trainService.GetCoupled(loco);
            var resultF = RouteDistanceWithCars(loco.LocationF, targetLoc, coupled);
            var resultR = RouteDistanceWithCars(loco.LocationR, targetLoc, coupled);

            RouteResult? result;
            if (resultF == null) result = resultR;
            else if (resultR == null) result = resultF;
            else result = resultF.Value.Distance <= resultR.Value.Distance ? resultF : resultR;

            _distanceCache[cacheKey] = result;
            return result;
        }

        /// <summary>
        /// Find the route distance between two locations with checkForCars enabled.
        /// Returns null if the route is blocked or not found.
        /// Note: takes raw Location for game API boundary calls.
        /// </summary>
        public static RouteResult? RouteDistanceWithCars(Location start, Location end, List<Car> ignoredCars)
        {
            try
            {
                if (start.segment == null || end.segment == null)
                    return null;

                var graph = Graph.Shared;
                var heuristic = DefaultHeuristic;

                var ignored = new System.Collections.Generic.HashSet<Car>(ignoredCars);
                var impasse = new System.Collections.Generic.HashSet<Car>();
                var steps = new List<RouteSearch.Step>();

                bool found = RouteSearch.FindRoute(
                    graph, start, end, heuristic,
                    steps, out RouteSearch.Metrics metrics,
                    checkForCars: true, trainLength: 0f,
                    trainMomentum: 0f, maxIterations: 5000,
                    checkForCarsIgnored: ignored,
                    checkForCarsImpasse: impasse);

                if (!found)
                {
                    if (impasse.Count > 0)
                        return new RouteResult(float.MaxValue, 0, BlockedByCars: true);
                    return null;
                }

                return new RouteResult(metrics.Distance, 0, BlockedByCars: false);
            }
            catch { return null; }
        }
    }
}
