using System.Collections.Generic;
using System.Linq;
using Game.Messages;
using Model;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

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

        public string Serialize()
        {
            var obj = new JObject
            {
                ["visitedSwitches"] = new JArray(VisitedSwitches.ToArray()),
                ["visitedLoopKeys"] = new JArray(VisitedLoopKeys.ToArray()),
                ["skippedCarIds"] = new JArray(SkippedCars.Select(c => c.id).ToArray())
            };
            return obj.ToString(Formatting.None);
        }

        public static PlanningContext Deserialize(string json)
        {
            var obj = JObject.Parse(json);

            var visitedSwitches = new HashSet<string>(
                (obj["visitedSwitches"] as JArray)?.Select(t => t.Value<string>())
                ?? Enumerable.Empty<string>());

            var visitedLoopKeys = new HashSet<string>(
                (obj["visitedLoopKeys"] as JArray)?.Select(t => t.Value<string>())
                ?? Enumerable.Empty<string>());

            var skippedCars = new HashSet<Car>();
            var skippedIds = (obj["skippedCarIds"] as JArray)?.Select(t => t.Value<string>())
                ?? Enumerable.Empty<string>();
            foreach (var id in skippedIds)
            {
                if (TrainController.Shared.TryGetCarForId(id, out Car car))
                    skippedCars.Add(car);
            }

            return new PlanningContext(visitedSwitches, visitedLoopKeys, skippedCars, null);
        }
    }
}
