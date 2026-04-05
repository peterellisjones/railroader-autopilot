using System.Collections.Generic;
using Model;
using Track;
using Track.Search;
using Autopilot.Model;
using Autopilot.Services;
using Autopilot.TrackGraph;

namespace Autopilot.Planning
{
    public class FeasibilityChecker
    {
        private readonly TrainService _trainService;
        private readonly PlanningLogger _logger;
        public RouteChecker RouteChecker { get; }
        public ApproachAnalyzer ApproachAnalyzer { get; }
        public LoopValidator LoopValidator { get; }

        // Testable planning fields (used by interface-based constructor path)
        private readonly ITrainService? _iTrainService;
        private readonly IGraphAdapter? _iGraph;

        // Cached per plan cycle — GetLoopStatus is expensive (up to 5 loop searches)
        private LoopStatus? _cachedLoopStatus;
        private string? _cachedLoopStatusLocoId;

        public FeasibilityChecker(TrainService trainService)
        {
            _trainService = trainService;
            _logger = new PlanningLogger();
            RouteChecker = new RouteChecker(trainService, _logger);
            ApproachAnalyzer = new ApproachAnalyzer(RouteChecker, _logger);
            LoopValidator = new LoopValidator(RouteChecker, trainService, _logger);
        }

        /// <summary>Constructor for testable planning.</summary>
        public FeasibilityChecker(ITrainService trainService, IGraphAdapter graph)
        {
            _iTrainService = trainService;
            _iGraph = graph;
            _trainService = null!;
            _logger = new PlanningLogger();
            RouteChecker = new RouteChecker(trainService, graph, NullPlanningLogger.Instance);
            ApproachAnalyzer = new ApproachAnalyzer(RouteChecker, graph, NullPlanningLogger.Instance);
            LoopValidator = null!; // Loop queries go through ITrainService
        }

        public void ClearCache()
        {
            _cachedLoopStatus = null;
            _cachedLoopStatusLocoId = null;
        }

        private void Log(string msg) => Loader.Mod.Logger.Log($"Autopilot Feasibility: {msg}");

        // --- Main API ---

        public bool CanDeliver(BaseLocomotive loco, CarGroup group, SpanBoundary destLocation)
        {
            if (group.IsEmpty) return false;

            if (!ApproachAnalyzer.CheckApproachDirection(loco, group, destLocation))
                return false;

            // Convert SpanBoundary to DirectedPosition for route checking
            var adapter = new GameGraphAdapter();
            RegisterSegmentById(adapter, destLocation.SegmentId);
            if (!CanRouteTo(loco, destLocation.ToDirectedPosition(adapter)))
                return false;

            return true;
        }

        private static void RegisterSegmentById(GameGraphAdapter adapter, string segmentId)
        {
            if (segmentId == null) return;
            var graph = Track.Graph.Shared;
            foreach (var seg in graph.Segments)
            {
                if (seg.id == segmentId)
                {
                    adapter.RegisterSegment(seg);
                    return;
                }
            }
        }

        public LoopStatus GetLoopStatus(BaseLocomotive loco)
        {
            if (_cachedLoopStatus != null && _cachedLoopStatusLocoId == loco.id)
                return _cachedLoopStatus;
            _cachedLoopStatus = LoopValidator.GetLoopStatus(loco);
            _cachedLoopStatusLocoId = loco.id;
            return _cachedLoopStatus;
        }

        public (DirectedPosition? location, string? loopKey) GetRepositionLocation(
            BaseLocomotive loco, IEnumerable<string>? visitedSwitches,
            IEnumerable<string>? visitedLoopKeys = null,
            List<SpanBoundary>? deliveryDestinations = null)
            => LoopValidator.GetRepositionLocation(loco, visitedSwitches, visitedLoopKeys, deliveryDestinations);

        public bool CanRouteTo(BaseLocomotive loco, DirectedPosition destination)
            => RouteChecker.CanRouteTo(loco, destination);

        public bool CanRouteTo(BaseLocomotive loco, DirectedPosition destination,
            float trainLength, List<Car> ignoredCars)
            => RouteChecker.CanRouteTo(loco, destination, trainLength, ignoredCars);

        public (bool found, List<RouteSearch.Step> steps) CanRouteToWithSteps(
            BaseLocomotive loco, DirectedPosition destination,
            float trainLength, List<Car> ignoredCars)
            => RouteChecker.CanRouteToWithSteps(loco, destination, trainLength, ignoredCars);

        // =================================================================
        // Testable overloads using ITrainService + IGraphAdapter
        // =================================================================

        /// <summary>
        /// Check if delivery is feasible using abstract positions.
        /// Checks approach direction and route feasibility.
        /// </summary>
        public bool CanDeliver(GraphPosition tailOutward, GraphPosition tailInward,
            GraphPosition destLocation)
        {
            if (!ApproachAnalyzer.CheckApproachDirection(tailOutward, tailInward, destLocation))
                return false;
            if (!RouteChecker.CanRouteTo(destLocation))
                return false;
            return true;
        }
    }
}
