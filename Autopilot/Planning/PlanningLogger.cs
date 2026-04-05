using System.Collections.Generic;
using Model;
using Track;
using Autopilot.Model;

namespace Autopilot.Planning
{
    public class PlanningLogger : IPlanningLogger
    {
        public void Log(string prefix, string msg)
        {
            Loader.Mod.Logger.Log($"Autopilot {prefix}: {msg}");
        }

        public void LogDebug(string prefix, string msg)
        {
            if (Loader.Settings?.verboseLogging == true)
                Log(prefix, msg);
        }

        public void LogRoute(string prefix, List<TrackSegment> route)
        {
            LogDebug(prefix, $"route ({route.Count} segs): [{string.Join(" → ", route.ConvertAll(s => s.id))}]");
        }

        public void LogApproachDetails(string prefix, CarGroup group, BaseLocomotive loco,
            DirectedPosition tailOutward, List<TrackSegment> route)
        {
            var tailSeg = tailOutward.Segment;
            if (tailSeg == null) return;

            var graph = Graph.Shared;
            var nodeA = tailSeg.NodeForEnd(TrackSegment.End.A);
            var nodeB = tailSeg.NodeForEnd(TrackSegment.End.B);
            LogDebug(prefix, $"tailOutward seg={tailSeg.id}, facing={tailOutward.Facing}, " +
                $"distFromA={tailOutward.DistanceFromA:F1}, segLen={tailSeg.GetLength():F1}");
            LogDebug(prefix, $"seg nodeA={nodeA?.id}, nodeB={nodeB?.id}, " +
                $"nodeA_isSwitch={nodeA != null && graph.IsSwitch(nodeA)}, " +
                $"nodeB_isSwitch={nodeB != null && graph.IsSwitch(nodeB)}");
            if (group.TailInwardEnd.HasValue)
            {
                var tailInward = group.TailInwardEnd.Value;
                LogDebug(prefix, $"tailInward seg={tailInward.Segment?.id}, " +
                    $"facing={tailInward.Facing}, distFromA={tailInward.DistanceFromA:F1}");
            }
            LogDebug(prefix, $"tailCar={group.TailCar?.DisplayName}, " +
                $"coupledA={group.TailCar?.CoupledTo(Car.LogicalEnd.A)?.DisplayName ?? "null"}, " +
                $"coupledB={group.TailCar?.CoupledTo(Car.LogicalEnd.B)?.DisplayName ?? "null"}");

            LogDebug(prefix, $"loco seg={loco.LocationF.segment?.id} (F), {loco.LocationR.segment?.id} (R)");
            foreach (var car in group.Cars)
            {
                LogDebug(prefix, $"car {car.DisplayName}: " +
                    $"endA seg={car.EndA.Segment?.id} facing={car.EndA.Facing} distA={car.EndA.DistanceFromA:F1}, " +
                    $"endB seg={car.EndB.Segment?.id} facing={car.EndB.Facing} distA={car.EndB.DistanceFromA:F1}");
            }
            LogRoute(prefix, route);

            if (route.Count >= 2)
            {
                var firstDiffSeg = route.Count > 1 && route[0] != tailSeg ? route[0] : (route.Count > 1 ? route[1] : null);
                if (firstDiffSeg != null)
                {
                    var sharedNode = Services.TrackWalker.FindSharedNode(tailSeg, firstDiffSeg);
                    var exitEnd = sharedNode != null ? tailSeg.EndForNode(sharedNode) : (TrackSegment.End?)null;
                    // tailOutward facing direction determines which end of the segment the outward end faces
                    var facingEnd = tailOutward.Facing == Direction.TowardEndA ? TrackSegment.End.A : TrackSegment.End.B;
                    LogDebug(prefix, $"route exits via node={sharedNode?.id} at end={exitEnd}, " +
                        $"tailOutward faces={facingEnd}, match={exitEnd == facingEnd}");
                }
            }
        }
    }
}
