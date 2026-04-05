using System.Collections.Generic;
using System.Linq;
using Model;
using Track;
using Autopilot.Model;
using Autopilot.TrackGraph;

namespace Autopilot.Execution
{
    /// <summary>
    /// Converts abstract model types (ICar, GraphPosition, GraphCoupleWaypoint)
    /// back to game types (Car, DirectedPosition, CoupleWaypoint) at the
    /// execution layer boundary.
    /// </summary>
    internal static class PlanUnwrapper
    {
        public static Car UnwrapCar(ICar car) => ((CarAdapter)car).Car;

        public static List<Car> UnwrapCars(IEnumerable<ICar> cars)
            => cars.Select(c => UnwrapCar(c)).ToList();

        public static DirectedPosition ToDirectedPosition(GraphPosition gp, GameGraphAdapter adapter)
            => adapter.ToDirectedPosition(gp);

        public static CoupleWaypoint ToCoupleWaypoint(GraphCoupleWaypoint gcw, GameGraphAdapter adapter)
        {
            var seg = adapter.GetSegment(gcw.SegmentId);
            return new CoupleWaypoint(seg, gcw.DistanceFromA, gcw.Facing);
        }
    }
}
