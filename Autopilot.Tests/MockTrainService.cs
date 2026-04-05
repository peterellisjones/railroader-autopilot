using System.Collections.Generic;
using System.Linq;
using Autopilot.Model;
using Autopilot.Services;

namespace Autopilot.Tests
{
    /// <summary>
    /// Mock ITrainService for unit tests. All state is set by ScenarioBuilder.
    /// Route queries delegate to MockGraphAdapter.
    /// </summary>
    public class MockTrainService : ITrainService
    {
        private readonly MockGraphAdapter _graph;
        private readonly List<ICar> _coupled;
        private readonly ICar _locoCar;
        private readonly float _trainLength;

        // Position lookup: carId -> (endA, endB)
        private readonly Dictionary<string, (GraphPosition endA, GraphPosition endB)> _carPositions = new();
        private GraphPosition _locoFront;
        private GraphPosition _locoRear;

        // Car data
        private readonly Dictionary<string, string?> _destinationTrackIds = new();
        private readonly Dictionary<string, string?> _destinationNames = new();
        private readonly HashSet<string> _waybilledCarIds = new();
        private readonly HashSet<string> _completedWaybillCarIds = new();

        // Destination candidates per destination track ID
        private readonly Dictionary<string, List<DestinationCandidate>> _destCandidates = new();
        private readonly Dictionary<string, float> _availableSpace = new();

        // Loop/runaround
        private LoopStatus _loopStatus = LoopStatus.NotOnLoop();
        private (GraphPosition? location, string? loopKey) _repositionResult = (null, null);

        public string LocoId => _locoCar.id;

        public MockTrainService(MockGraphAdapter graph, List<ICar> coupled, ICar locoCar,
            GraphPosition locoFront, GraphPosition locoRear, float trainLength)
        {
            _graph = graph;
            _coupled = coupled;
            _locoCar = locoCar;
            _locoFront = locoFront;
            _locoRear = locoRear;
            _trainLength = trainLength;
        }

        // --- Setup methods (called by ScenarioBuilder) ---

        public void SetCarPosition(string carId, GraphPosition endA, GraphPosition endB)
        {
            _carPositions[carId] = (endA, endB);
        }

        public void SetWaybill(string carId, string destinationTrackId, string destinationName,
            bool completed = false)
        {
            _waybilledCarIds.Add(carId);
            _destinationTrackIds[carId] = destinationTrackId;
            _destinationNames[carId] = destinationName;
            if (completed) _completedWaybillCarIds.Add(carId);
        }

        public void SetDestinationCandidates(string destinationTrackId,
            List<DestinationCandidate> candidates, float availableSpace)
        {
            _destCandidates[destinationTrackId] = candidates;
            _availableSpace[destinationTrackId] = availableSpace;
        }

        public void SetLoopStatus(LoopStatus status) => _loopStatus = status;

        public void SetRepositionResult(GraphPosition? location, string? loopKey)
            => _repositionResult = (location, loopKey);

        // --- ITrainService implementation ---

        public IReadOnlyList<ICar> GetCoupled() => _coupled;
        public float GetTrainLength() => _trainLength;
        public ICar GetLocoCar() => _locoCar;
        public GraphPosition GetLocoFront() => _locoFront;
        public GraphPosition GetLocoRear() => _locoRear;

        public GraphPosition GetCarEndA(ICar car) =>
            _carPositions.TryGetValue(car.id, out var pos) ? pos.endA : default;

        public GraphPosition GetCarEndB(ICar car) =>
            _carPositions.TryGetValue(car.id, out var pos) ? pos.endB : default;

        public GraphPosition GetCarFront(ICar car) => GetCarEndA(car);
        public GraphPosition GetCarRear(ICar car) => GetCarEndB(car);

        public bool HasWaybill(ICar car) => _waybilledCarIds.Contains(car.id);
        public bool IsWaybillCompleted(ICar car) => _completedWaybillCarIds.Contains(car.id);

        public string? GetDestinationTrackId(ICar car) =>
            _destinationTrackIds.TryGetValue(car.id, out var id) ? id : null;

        public string? GetDestinationName(ICar car) =>
            _destinationNames.TryGetValue(car.id, out var name) ? name : null;

        public IReadOnlyList<DestinationCandidate> GetDestinationCandidates(ICar car)
        {
            var destId = GetDestinationTrackId(car);
            if (destId != null && _destCandidates.TryGetValue(destId, out var candidates))
                return candidates;
            return new List<DestinationCandidate>();
        }

        public float GetAvailableSpace(ICar car)
        {
            var destId = GetDestinationTrackId(car);
            if (destId != null && _availableSpace.TryGetValue(destId, out var space))
                return space;
            return 0f;
        }

        public LoopStatus GetLoopStatus() => _loopStatus;

        public (GraphPosition? location, string? loopKey) GetRepositionLocation(
            IEnumerable<string>? visitedSwitches,
            IEnumerable<string>? visitedLoopKeys,
            IReadOnlyList<GraphPosition>? deliveryDestinations)
            => _repositionResult;

        public void ClearPlanCaches() { }
    }
}
