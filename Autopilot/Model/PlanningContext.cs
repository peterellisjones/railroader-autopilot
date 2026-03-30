using System.Collections.Generic;
using System.Collections.Immutable;
using Model;

namespace Autopilot.Model
{
    public record PlanningContext(
        ImmutableHashSet<string> VisitedSwitches,
        ImmutableHashSet<string> VisitedLoopKeys,
        ImmutableHashSet<Car> SkippedCars,
        SplitInfo? PendingSplit
    )
    {
        public static PlanningContext Empty => new(
            ImmutableHashSet<string>.Empty,
            ImmutableHashSet<string>.Empty,
            ImmutableHashSet<Car>.Empty,
            null
        );

        public PlanningContext WithVisitedSwitch(string switchId) =>
            this with { VisitedSwitches = VisitedSwitches.Add(switchId) };

        public PlanningContext WithVisitedLoop(string key) =>
            this with { VisitedLoopKeys = VisitedLoopKeys.Add(key) };

        public PlanningContext WithSkippedCar(Car car) =>
            this with { SkippedCars = SkippedCars.Add(car) };

        public PlanningContext WithSkippedCars(IEnumerable<Car> cars) =>
            this with { SkippedCars = SkippedCars.Union(cars) };

        public PlanningContext WithPendingSplit(SplitInfo? split) =>
            this with { PendingSplit = split };

        public PlanningContext WithClearedSwitches() =>
            this with { VisitedSwitches = ImmutableHashSet<string>.Empty };
    }
}
