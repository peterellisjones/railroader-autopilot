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
                var impasse = new System.Collections.Generic.HashSet<Car>();

                var routeSteps = new List<RouteSearch.Step>();

                bool found = RouteSearch.FindRoute(
                    graph,
                    loco.LocationF,
                    destLoc,
                    heuristic,
                    routeSteps,
                    out RouteSearch.Metrics metrics,
                    checkForCars: true,
                    trainLength: trainLength,
                    trainMomentum: 0f,
                    maxIterations: 10000,
                    checkForCarsIgnored: ignored,
                    checkForCarsImpasse: impasse,
                    limitSwitchIds: null,
                    enableLogging: false
                );

                return found;
            }
            catch (System.Exception ex)
            {
                _logger.Log("RouteChecker", $"CanRouteTo: exception {ex.GetType().Name}: {ex.Message}");
                _logger.Log("RouteChecker", $"CanRouteTo: stack: {ex.StackTrace}");
                // Route search crashed — if the loco is on or adjacent to the
                // destination segment, assume reachable (already there). Otherwise
                // treat as unreachable.
                bool nearDest = loco.LocationF.segment == destination.Segment
                             || loco.LocationR.segment == destination.Segment;
                _logger.Log("RouteChecker", $"CanRouteTo: fallback nearDest={nearDest}");
                return nearDest;
            }
        }

        /// <summary>
        /// Like CanRouteTo, but also returns the route steps for safety checking.
        /// Returns (found, routeSteps). If not found, routeSteps is empty.
        /// </summary>
        public (bool found, List<RouteSearch.Step> steps) CanRouteToWithSteps(
            BaseLocomotive loco, DirectedPosition destination,
            float trainLength, List<Car> ignoredCars)
        {
            var steps = new List<RouteSearch.Step>();
            try
            {
                var graph = Graph.Shared;
                var heuristic = DefaultHeuristic;
                var destLoc = destination.ToLocation();

                if (destLoc.segment == null || loco.LocationF.segment == null)
                    return (true, steps);

                var ignored = new System.Collections.Generic.HashSet<Car>(ignoredCars);
                var impasse = new System.Collections.Generic.HashSet<Car>();

                bool found = RouteSearch.FindRoute(
                    graph,
                    loco.LocationF,
                    destLoc,
                    heuristic,
                    steps,
                    out RouteSearch.Metrics metrics,
                    checkForCars: true,
                    trainLength: trainLength,
                    trainMomentum: 0f,
                    maxIterations: 10000,
                    checkForCarsIgnored: ignored,
                    checkForCarsImpasse: impasse,
                    limitSwitchIds: null,
                    enableLogging: false
                );

                return (found, steps);
            }
            catch (System.Exception ex)
            {
                _logger.Log("RouteChecker", $"CanRouteToWithSteps: exception {ex.GetType().Name}: {ex.Message}");
                bool nearDest = loco.LocationF.segment == destination.Segment
                             || loco.LocationR.segment == destination.Segment;
                return (nearDest, steps);
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
            var coupled = _trainService.GetCoupled(loco);
            var targetLoc = target.ToLocation();
            var resultF = RouteDistanceWithCars(loco.LocationF, targetLoc, coupled);
            var resultR = RouteDistanceWithCars(loco.LocationR, targetLoc, coupled);

            if (resultF == null) return resultR;
            if (resultR == null) return resultF;
            return resultF.Value.Distance <= resultR.Value.Distance ? resultF : resultR;
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
