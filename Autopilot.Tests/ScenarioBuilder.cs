using System;
using System.Collections.Generic;
using System.Linq;
using Autopilot.Model;
using Autopilot.TrackGraph;
using Model;

namespace Autopilot.Tests
{
    /// <summary>
    /// Fluent builder that constructs a Scenario with MockGraphAdapter, MockCar instances,
    /// and MockTrainService wired into a coherent test scenario.
    /// </summary>
    public class ScenarioBuilder
    {
        // --- Segment/switch definitions (forwarded to MockGraphAdapter) ---
        private readonly List<(string id, string nodeA, string nodeB, float length)> _segments = new();
        private readonly List<(string nodeId, string enter, string exitNormal, string exitReverse, float fouling)> _switches = new();

        // --- Car definitions ---
        private readonly List<CarDef> _carDefs = new();
        private LocoDef? _locoDef;

        // --- Destination definitions ---
        private readonly Dictionary<string, DestDef> _destDefs = new();

        // --- Optional overrides ---
        private LoopStatus? _loopStatus;
        private (GraphPosition? location, string? loopKey)? _repositionResult;

        private class CarDef
        {
            public string Id;
            public string SegmentId;
            public float Distance;
            public Direction Facing;
            public string? Destination;
            public string? CoupledTo;
            public float Length;
        }

        private class LocoDef
        {
            public string Id;
            public string SegmentId;
            public float Distance;
            public Direction Facing;
            public string? CoupledTo;
            public float Length;
        }

        private class DestDef
        {
            public string TrackId;
            public string Name;
            public float AvailableSpace;
            public GraphPosition ApproachTarget;
        }

        // --- Fluent API ---

        public ScenarioBuilder AddSegment(string segId, string nodeA, string nodeB, float length)
        {
            _segments.Add((segId, nodeA, nodeB, length));
            return this;
        }

        public ScenarioBuilder AddSwitch(string nodeId, string enter, string exitNormal, string exitReverse, float fouling = 10f)
        {
            _switches.Add((nodeId, enter, exitNormal, exitReverse, fouling));
            return this;
        }

        public ScenarioBuilder PlaceCar(string id, string on, float distance,
            Direction facing = Direction.TowardEndB, string? destination = null,
            string? coupledTo = null, float length = 10f)
        {
            _carDefs.Add(new CarDef
            {
                Id = id,
                SegmentId = on,
                Distance = distance,
                Facing = facing,
                Destination = destination,
                CoupledTo = coupledTo,
                Length = length
            });
            return this;
        }

        public ScenarioBuilder PlaceLoco(string id, string on, float distance,
            Direction facing = Direction.TowardEndB, string? coupledTo = null,
            float length = 15f)
        {
            _locoDef = new LocoDef
            {
                Id = id,
                SegmentId = on,
                Distance = distance,
                Facing = facing,
                CoupledTo = coupledTo,
                Length = length
            };
            return this;
        }

        public ScenarioBuilder DefineDestination(string trackId, string name,
            float availableSpace, GraphPosition approachTarget)
        {
            _destDefs[trackId] = new DestDef
            {
                TrackId = trackId,
                Name = name,
                AvailableSpace = availableSpace,
                ApproachTarget = approachTarget
            };
            return this;
        }

        public ScenarioBuilder SetLoopStatus(LoopStatus status)
        {
            _loopStatus = status;
            return this;
        }

        public ScenarioBuilder SetRepositionResult(GraphPosition? location, string? loopKey)
        {
            _repositionResult = (location, loopKey);
            return this;
        }

        // --- Build ---

        public Scenario Build()
        {
            if (_locoDef == null)
                throw new InvalidOperationException("PlaceLoco must be called before Build().");

            // 1. Create MockGraphAdapter with all segments and switches
            var graph = new MockGraphAdapter();
            var segmentLengths = new Dictionary<string, float>();
            foreach (var (id, nodeA, nodeB, length) in _segments)
            {
                graph.AddSegment(id, nodeA, nodeB, length);
                segmentLengths[id] = length;
            }
            foreach (var (nodeId, enter, exitNormal, exitReverse, fouling) in _switches)
            {
                graph.AddSwitch(nodeId, enter, exitNormal, exitReverse, fouling);
            }

            // 2. Create MockCar instances
            var cars = new Dictionary<string, MockCar>();

            foreach (var carDef in _carDefs)
            {
                var mockCar = new MockCar
                {
                    id = carDef.Id,
                    DisplayName = carDef.Id,
                    CarLength = carDef.Length,
                    IsLocomotive = false,
                    IsLocoOrTender = false
                };
                cars[carDef.Id] = mockCar;
            }

            var loco = new MockCar
            {
                id = _locoDef.Id,
                DisplayName = _locoDef.Id,
                CarLength = _locoDef.Length,
                IsLocomotive = true,
                IsLocoOrTender = true
            };
            cars[_locoDef.Id] = loco;

            // 3. Wire coupling chains
            foreach (var carDef in _carDefs)
            {
                if (carDef.CoupledTo != null)
                {
                    if (!cars.TryGetValue(carDef.CoupledTo, out var targetCar))
                        throw new InvalidOperationException(
                            $"Car '{carDef.Id}' coupledTo '{carDef.CoupledTo}' which doesn't exist. " +
                            "Place the target car before the car that references it.");
                    // carDef car's B end -> target car's A end
                    MockCar.Couple(cars[carDef.Id], targetCar);
                }
            }

            if (_locoDef.CoupledTo != null)
            {
                if (!cars.TryGetValue(_locoDef.CoupledTo, out var targetCar))
                    throw new InvalidOperationException(
                        $"Loco '{_locoDef.Id}' coupledTo '{_locoDef.CoupledTo}' which doesn't exist.");
                // loco's B end -> target car's A end
                MockCar.Couple(loco, targetCar);
            }

            // 4. Walk coupling chain from loco to build coupled list
            //    Walk A side: loco -> CoupledTo(A) -> ...
            //    Walk B side: loco -> CoupledTo(B) -> ...
            var sideA = new List<ICar>();
            WalkCoupling(loco, Car.LogicalEnd.A, sideA);
            sideA.Reverse(); // reverse so furthest-from-loco is first

            var sideB = new List<ICar>();
            WalkCoupling(loco, Car.LogicalEnd.B, sideB);

            var coupledList = new List<ICar>();
            coupledList.AddRange(sideA);
            coupledList.Add(loco);
            coupledList.AddRange(sideB);

            // 5. Compute train length
            float trainLength = coupledList.Sum(c => c.CarLength);
            if (coupledList.Count > 1)
                trainLength += (coupledList.Count - 1) * AutopilotConstants.ConsistGapPerCar;

            // 6. Compute loco positions
            var locoFront = new GraphPosition(_locoDef.SegmentId, _locoDef.Distance, _locoDef.Facing);
            var locoRear = new GraphPosition(_locoDef.SegmentId, _locoDef.Distance, _locoDef.Facing.Opposite());

            // 7. Create MockTrainService
            var trainService = new MockTrainService(graph, coupledList, loco,
                locoFront, locoRear, trainLength);

            // 8. Set car positions on the service
            foreach (var carDef in _carDefs)
            {
                var endA = new GraphPosition(carDef.SegmentId, carDef.Distance, Direction.TowardEndA);
                var endB = new GraphPosition(carDef.SegmentId, carDef.Distance, Direction.TowardEndB);
                trainService.SetCarPosition(carDef.Id, endA, endB);
            }
            // Set loco position too
            {
                var endA = new GraphPosition(_locoDef.SegmentId, _locoDef.Distance, Direction.TowardEndA);
                var endB = new GraphPosition(_locoDef.SegmentId, _locoDef.Distance, Direction.TowardEndB);
                trainService.SetCarPosition(_locoDef.Id, endA, endB);
            }

            // 9. Set waybills from car destinations
            foreach (var carDef in _carDefs)
            {
                if (carDef.Destination != null)
                {
                    // Use destination name from DefineDestination if available, else use trackId
                    string destName = _destDefs.TryGetValue(carDef.Destination, out var dd)
                        ? dd.Name
                        : carDef.Destination;
                    trainService.SetWaybill(carDef.Id, carDef.Destination, destName);
                }
            }

            // 10. Set destination candidates
            //     Collect all unique destination track IDs from waybilled cars
            var destinationTrackIds = _carDefs
                .Where(c => c.Destination != null)
                .Select(c => c.Destination!)
                .Distinct()
                .ToList();

            foreach (var destTrackId in destinationTrackIds)
            {
                DestDef destDef;
                if (_destDefs.TryGetValue(destTrackId, out var explicitDef))
                {
                    destDef = explicitDef;
                }
                else
                {
                    // Auto-create from segment info
                    float segLength = segmentLengths.TryGetValue(destTrackId, out var len) ? len : 200f;
                    destDef = new DestDef
                    {
                        TrackId = destTrackId,
                        Name = destTrackId,
                        AvailableSpace = segLength,
                        ApproachTarget = new GraphPosition(destTrackId, 0, Direction.TowardEndB)
                    };
                }

                var candidate = new DestinationCandidate(
                    Location: destDef.ApproachTarget,
                    CoupleTarget: null,
                    AvailableSpace: destDef.AvailableSpace,
                    SpanIndex: 0,
                    ApproachTarget: destDef.ApproachTarget,
                    DestinationTrackId: destDef.TrackId,
                    DestinationName: destDef.Name
                );

                trainService.SetDestinationCandidates(
                    destDef.TrackId,
                    new List<DestinationCandidate> { candidate },
                    destDef.AvailableSpace);
            }

            // Apply optional overrides
            if (_loopStatus != null)
                trainService.SetLoopStatus(_loopStatus);
            if (_repositionResult.HasValue)
                trainService.SetRepositionResult(_repositionResult.Value.location, _repositionResult.Value.loopKey);

            // Build ICar dictionary for Scenario.GetCar
            var icarDict = new Dictionary<string, ICar>();
            foreach (var kv in cars)
                icarDict[kv.Key] = kv.Value;

            return new Scenario(trainService, graph, coupledList, loco, icarDict);
        }

        /// <summary>
        /// Walk from the given car through the coupling chain in one direction,
        /// adding each encountered car to the list (not including the starting car).
        /// </summary>
        private static void WalkCoupling(MockCar start, Car.LogicalEnd direction, List<ICar> result)
        {
            var current = start.CoupledTo(direction);
            var visited = new HashSet<string> { start.id };
            while (current != null && !visited.Contains(current.id))
            {
                visited.Add(current.id);
                result.Add(current);
                // Continue walking: if we arrived via A end, exit via B end and vice versa
                // We entered through one end, so we exit through the other
                // When we couple a.B -> b.A, walking from a's B side reaches b at b's A end
                // So from b we should continue via b's B end
                // Conversely, if we entered b at b's B end, continue via b's A end
                // Since Couple(a,b) sets a.CoupledAtB=b and b.CoupledAtA=a:
                //   Walking from a via B: reaches b. b.CoupledAtA == a, so we entered at A. Continue via B.
                //   Walking from b via A: reaches a. a.CoupledAtB == b, so we entered at B. Continue via A.
                // In general: if current.CoupledAtA == prev, we entered at A, continue via B
                //             if current.CoupledAtB == prev, we entered at B, continue via A
                var prev = result.Count >= 2 ? result[result.Count - 2] : (ICar)start;
                if (current.CoupledTo(Car.LogicalEnd.A) != null && current.CoupledTo(Car.LogicalEnd.A)!.id == prev.id)
                {
                    // Entered at A, continue via B
                    current = current.CoupledTo(Car.LogicalEnd.B);
                }
                else if (current.CoupledTo(Car.LogicalEnd.B) != null && current.CoupledTo(Car.LogicalEnd.B)!.id == prev.id)
                {
                    // Entered at B, continue via A
                    current = current.CoupledTo(Car.LogicalEnd.A);
                }
                else
                {
                    break; // shouldn't happen but guard against infinite loops
                }
            }
        }
    }
}
