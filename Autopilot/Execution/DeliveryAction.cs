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
        private string _initError;
        private List<Car> _completedCars;
        private bool _coupleVerified;
        private bool _coupleRetried;

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
                // Couple to existing car on the span. Use the free (uncoupled)
                // end — that's the end facing the approach. For a lone car
                // (both ends free), pick the end closer to the loco by graph
                // distance — LogicalEnd.A is arbitrary and may face the buffer.
                var graph = Graph.Shared;
                var target = step.CoupleTarget;
                bool aFree = target.CoupledTo(Car.LogicalEnd.A) == null;
                bool bFree = target.CoupledTo(Car.LogicalEnd.B) == null;
                Car.LogicalEnd nearEnd;
                if (aFree && bFree)
                {
                    float dA = LocoDistanceTo(graph, loco, target.LocationA);
                    float dB = LocoDistanceTo(graph, loco, target.LocationB);
                    nearEnd = dA <= dB ? Car.LogicalEnd.A : Car.LogicalEnd.B;
                    Loader.Mod.Logger.Log($"Autopilot DeliveryAction: lone couple target, dA={dA:F1} dB={dB:F1} → end {nearEnd}");
                }
                else
                {
                    nearEnd = aFree ? Car.LogicalEnd.A : Car.LogicalEnd.B;
                }
                var coupleLoc = CoupleLocationCalculator.GetCoupleLocationForEnd(
                    new CarAdapter(target), nearEnd, graph);
                var coupleLocStr = Graph.Shared.LocationToString(coupleLoc.ToLocation());
                Loader.Mod.Logger.Log($"Autopilot DeliveryAction: coupling to {step.CoupleTarget.DisplayName} " +
                    $"at end {nearEnd}, waypoint={coupleLocStr}");
                trainService.SetWaypointWithCouple(loco, coupleLoc, step.CoupleTarget.id);
            }
            else
            {
                // Empty span — push waypoint to the far end of the span so the
                // train goes as deep as possible into the siding.
                var waypoint = GetDeepestSpanLocation(loco, step);
                trainService.SetWaypoint(loco, waypoint);
            }
        }

        public ActionOutcome Tick(BaseLocomotive loco, TrainService trainService)
        {
            if (_initError != null)
                return new ActionFailed(_initError);

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

            // If a couple target was set, verify coupling actually happened.
            // The AE waypoint can be satisfied before physical coupling occurs
            // (same pattern as PickupAction's explicit coupling check).
            if (_step.CoupleTarget != null && !_coupleVerified)
            {
                var consist = trainService.GetCoupled(loco);
                if (consist.Contains(_step.CoupleTarget))
                {
                    Loader.Mod.Logger.Log("Autopilot DeliveryAction: couple target verified in consist.");
                    _coupleVerified = true;
                }
                else if (!_coupleRetried)
                {
                    // Coupling didn't trigger — nudge toward the target to
                    // make physical contact and trigger auto-coupling.
                    _coupleRetried = true;
                    var graph = Track.Graph.Shared;
                    graph.TryFindDistance(loco.LocationF, _step.CoupleTarget.LocationA, out float dF, out _);
                    graph.TryFindDistance(loco.LocationR, _step.CoupleTarget.LocationA, out float dR, out _);
                    bool pushForward = dF < dR;
                    Loader.Mod.Logger.Log($"Autopilot DeliveryAction: coupling not detected, nudging {(pushForward ? "forward" : "backward")}...");
                    trainService.MoveDistance(loco, 2f, pushForward);
                    return new InProgress();
                }
                else
                {
                    Loader.Mod.Logger.Log("Autopilot DeliveryAction: coupling retry failed — delivering uncoupled.");
                    _coupleVerified = true;
                }
            }

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
        /// Shortest graph distance from either loco end to a target location.
        /// Checks both LocationF and LocationR since RouteSearch only exits
        /// the starting segment in one direction.
        /// </summary>
        private static float LocoDistanceTo(Graph graph, BaseLocomotive loco, Location target)
        {
            float best = float.MaxValue;
            if (graph.TryFindDistance(loco.LocationF, target, out float dF, out _))
                best = System.Math.Min(best, dF);
            if (graph.TryFindDistance(loco.LocationR, target, out float dR, out _))
                best = System.Math.Min(best, dR);
            return best;
        }

        /// <summary>
        /// Find the deepest point within the destination span — the span
        /// endpoint farthest from the loco. This ensures the train pushes
        /// all the way into the siding. Stays within span bounds so the
        /// waypoint is always on valid siding track.
        /// </summary>
        private static DirectedPosition GetDeepestSpanLocation(BaseLocomotive loco, DeliveryStep step)
        {
            var graph = Graph.Shared;
            DirectedPosition best = step.DestinationLocation;
            float bestDist = 0f;

            var span = step.Destination.Spans[step.SpanIndex];
            foreach (var ep in new[] { span.lower, span.upper })
            {
                if (!ep.HasValue || ep.Value.segment == null) continue;

                graph.TryFindDistance(loco.LocationF, ep.Value, out float dF, out _);
                graph.TryFindDistance(loco.LocationR, ep.Value, out float dR, out _);
                float dist = Math.Max(dF, dR);
                if (dist > bestDist)
                {
                    bestDist = dist;
                    best = DirectedPosition.FromLocation(ep.Value);
                }
            }

            Loader.Mod.Logger.Log($"Autopilot DeliveryAction: waypoint at span end: " +
                $"{best.Segment?.id}|{best.ToLocation().distance:F1}");
            return best;
        }
    }
}
