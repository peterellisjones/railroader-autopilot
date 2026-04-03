using System;
using System.Collections.Generic;
using System.Linq;
using Model;
using Track;
using Track.Search;
using Autopilot.Model;
using Autopilot.Planning;
using Autopilot.Services;

namespace Autopilot.Execution
{
    public class DeliveryAction : IAction
    {
        private enum Phase { MovingToSiding, WaitingForStop, Uncoupling, BackingAway }

        private const float BackingDistanceMeters = 7f;

        private readonly DeliveryStep _step;
        private Phase _phase;
        private float _waitTimer;
        private float _stuckTimer;
        private string _statusMessage;
        private List<Car> _completedCars;

        public string StatusMessage => _statusMessage;

        public DeliveryAction(DeliveryStep step, BaseLocomotive loco, TrainService trainService)
        {
            _step = step;
            _step.Status = StepStatus.InProgress;
            _statusMessage = $"Moving to {step.DestinationName}...";
            _phase = Phase.MovingToSiding;
            _waitTimer = 0f;

            if (step.CoupleTarget != null)
            {
                // Couple to existing car on the span. We're pushing INTO the car
                // from the approach side, so the waypoint goes on the NEAR end
                // (the end facing our loco), not the far end.
                var graph = Graph.Shared;
                var nearEnd = step.CoupleTarget.ClosestLogicalEndTo(loco.LocationF, graph);
                var coupleLoc = CoupleLocationCalculator.GetCoupleLocationForEnd(
                    new CarAdapter(step.CoupleTarget), nearEnd, graph);
                Loader.Mod.Logger.Log($"Autopilot DeliveryAction: coupling to {step.CoupleTarget.DisplayName} at end {nearEnd}");
                trainService.SetWaypointWithCouple(loco, coupleLoc, step.CoupleTarget.id);
            }
            else
            {
                // Empty span — push waypoint deeper by trainLength so the
                // entire train fits past the span lower bound.
                var waypoint = AdjustWaypointForConsistLength(loco, step.DestinationLocation, step.Cars, trainService);
                trainService.SetWaypoint(loco, waypoint);
            }
        }

        public ActionOutcome Tick(BaseLocomotive loco, TrainService trainService)
        {
            switch (_phase)
            {
                case Phase.MovingToSiding:
                    return TickMovingToSiding(loco, trainService);
                case Phase.WaitingForStop:
                    return TickWaitingForStop(loco, trainService);
                case Phase.Uncoupling:
                    return TickUncoupling(loco, trainService);
                case Phase.BackingAway:
                    return TickBackingAway(loco, trainService);
                default:
                    return new ActionFailed("DeliveryAction: unknown phase.");
            }
        }

        private ActionOutcome TickMovingToSiding(BaseLocomotive loco, TrainService trainService)
        {
            if (!trainService.IsWaypointMode(loco))
                return new ActionFailed("Player took manual control — autopilot paused.");

            if (trainService.IsWaypointSatisfied(loco) && trainService.IsStopped(loco))
            {
                _phase = Phase.WaitingForStop;
                _waitTimer = 0f;
            }
            else if (trainService.IsStopped(loco))
            {
                _stuckTimer += AutopilotController.TickInterval;
                if (_stuckTimer > Loader.Settings.stuckTimeoutSeconds)
                    return new ActionFailed($"Train stuck for {Loader.Settings.stuckTimeoutSeconds:0}s moving to {_step.DestinationName}. Is the route blocked?");
            }
            else
            {
                _stuckTimer = 0f;
            }

            return new InProgress();
        }

        private ActionOutcome TickWaitingForStop(BaseLocomotive loco, TrainService trainService)
        {
            if (!trainService.IsStoppedForDuration(loco, 1.0f))
                return new InProgress();

            _statusMessage = $"Uncoupling {_step.Cars.Count} car(s)...";
            return PerformUncouple(loco, trainService);
        }

        private ActionOutcome PerformUncouple(BaseLocomotive loco, TrainService trainService)
        {
            var layout = ConsistLayout.Create(loco, trainService);

            Loader.Mod.Logger.Log($"Autopilot Uncouple: dropping {_step.Cars.Count} car(s) [{string.Join(", ", _step.Cars.ConvertAll(c => c.DisplayName))}]");
            Loader.Mod.Logger.Log($"Autopilot Uncouple: sideA={layout.SideA.Count} cars [{string.Join(", ", layout.SideA.Cars.Select(c => c.DisplayName))}]");
            Loader.Mod.Logger.Log($"Autopilot Uncouple: sideB={layout.SideB.Count} cars [{string.Join(", ", layout.SideB.Cars.Select(c => c.DisplayName))}]");

            _completedCars = new List<Car>(_step.Cars);

            var cutPoint = CutPointFinder.FindCutPoint(layout.SideA, _step.Cars);
            if (cutPoint.car == null)
                cutPoint = CutPointFinder.FindCutPoint(layout.SideB, _step.Cars);

            if (cutPoint.car == null)
                return new ActionFailed("Cannot find cut point — cars may have been rearranged.");

            Loader.Mod.Logger.Log($"Autopilot Uncouple: uncoupling {cutPoint.car.DisplayName} at end {cutPoint.end}");

            trainService.Uncouple(cutPoint.car, cutPoint.end);
            trainService.UpdateCarsForAE(loco);

            DisconnectHelper.DisconnectCars(_step.Cars, trainService);

            _phase = Phase.Uncoupling;
            _waitTimer = 0f;
            return new InProgress();
        }

        private ActionOutcome TickUncoupling(BaseLocomotive loco, TrainService trainService)
        {
            _waitTimer += AutopilotController.TickInterval;

            var consist = trainService.GetCoupled(loco);
            var carsToCheck = _completedCars ?? _step.Cars;
            bool stillCoupled = carsToCheck.Any(c => consist.Contains(c));

            if (!stillCoupled)
            {
                // Back away half a car length to clear the uncoupled cars.
                // Uses raw game Location for TryFindDistance — game API boundary
                var droppedCar = _step.Cars[0];
                var graph = Track.Graph.Shared;
                graph.TryFindDistance(loco.LocationF, droppedCar.LocationA, out float distF, out _);
                graph.TryFindDistance(loco.LocationR, droppedCar.LocationA, out float distR, out _);
                bool goForward = distR < distF;

                _statusMessage = "Backing away...";
                trainService.MoveDistance(loco, BackingDistanceMeters, goForward);
                _phase = Phase.BackingAway;
                _waitTimer = 0f;
                return new InProgress();
            }

            if (_waitTimer > AutopilotConstants.DecoupleWaitSeconds)
                return new ActionFailed("Cars did not decouple. Try uncoupling manually.");

            return new InProgress();
        }

        private ActionOutcome TickBackingAway(BaseLocomotive loco, TrainService trainService)
        {
            if (trainService.IsStoppedForDuration(loco, 0.5f))
            {
                // Yard mode persists after MoveDistance completes — switch back
                // to waypoint mode so the AE is ready for the next action.
                trainService.StopAE(loco);
                return new ActionReplan();
            }
            return new InProgress();
        }

        /// <summary>
        /// Push the waypoint deeper so the entire train fits past the
        /// destination (span lower bound). The AE positions the front of
        /// the train at the waypoint; the loco trails behind by trainLength.
        /// Offset by trainLength so the trailing end clears the span start.
        /// </summary>
        private static DirectedPosition AdjustWaypointForConsistLength(
            BaseLocomotive loco, DirectedPosition dest, List<Car> cars, TrainService trainService)
        {
            if (cars.Count == 0) return dest;

            var graph = Graph.Shared;
            if (dest.Segment == null) return dest;

            float offset = trainService.GetTrainLength(loco);
            var destLoc = dest.ToLocation();

            // Move dest DEEPER into the siding by the offset.
            // Try both directions since we don't know which way dest faces.
            // Pick the one farther from the loco (deeper).
            var optA = graph.LocationByMoving(destLoc, offset, false, true);
            var optB = graph.LocationByMoving(destLoc.Flipped(), offset, false, true).Flipped();

            float dA = MinDistToLoco(graph, loco, optA);
            float dB = MinDistToLoco(graph, loco, optB);
            var adjusted = dA >= dB ? optA : optB;

            // Sanity: adjusted must be farther from loco than dest (deeper).
            float destDist = MinDistToLoco(graph, loco, destLoc);
            if (Math.Max(dA, dB) <= destDist)
                return dest;

            Loader.Mod.Logger.Log($"Autopilot DeliveryAction: waypoint pushed {offset:F1}m deeper: " +
                $"{destLoc.segment?.id}|{destLoc.distance:F1} → {adjusted.segment?.id}|{adjusted.distance:F1}");

            return DirectedPosition.FromLocation(adjusted);
        }

        private static float SafeDist(Graph graph, Location a, Location b)
        {
            return graph.TryFindDistance(a, b, out float d, out _) ? d : float.MaxValue;
        }

        private static float MinDistToLoco(Graph graph, BaseLocomotive loco, Location loc)
        {
            float dF = SafeDist(graph, loco.LocationF, loc);
            float dR = SafeDist(graph, loco.LocationR, loc);
            return Math.Min(dF, dR);
        }
    }
}
