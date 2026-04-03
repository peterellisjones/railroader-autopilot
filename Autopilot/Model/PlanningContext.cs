using System.Collections.Generic;
using System.Linq;
using Model;

namespace Autopilot.Model
{
    public record PlanningContext(
        HashSet<string> VisitedSwitches,
        HashSet<string> VisitedLoopKeys,
        HashSet<Car> SkippedCars,
        SplitInfo? PendingSplit
    )
    {
        public static PlanningContext Empty => new(
            new HashSet<string>(),
            new HashSet<string>(),
            new HashSet<Car>(),
            null
        );

        public PlanningContext WithVisitedSwitch(string switchId) =>
            this with { VisitedSwitches = new HashSet<string>(VisitedSwitches) { switchId } };

        public PlanningContext WithVisitedLoop(string key) =>
            this with { VisitedLoopKeys = new HashSet<string>(VisitedLoopKeys) { key } };

        public PlanningContext WithSkippedCar(Car car) =>
            this with { SkippedCars = new HashSet<Car>(SkippedCars) { car } };

        public PlanningContext WithSkippedCars(IEnumerable<Car> cars) =>
            this with { SkippedCars = new HashSet<Car>(SkippedCars.Concat(cars)) };

        public PlanningContext WithPendingSplit(SplitInfo? split) =>
            this with { PendingSplit = split };

        public PlanningContext WithClearedSwitches() =>
            this with { VisitedSwitches = new HashSet<string>() };
    }
}
