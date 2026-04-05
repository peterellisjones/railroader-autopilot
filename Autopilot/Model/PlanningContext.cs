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
        HashSet<string> SkippedCarIds,
        SplitInfo? PendingSplit
    )
    {
        public static PlanningContext Empty => new(
            new HashSet<string>(),
            new HashSet<string>(),
            new HashSet<string>(),
            null
        );

        public PlanningContext WithVisitedSwitch(string switchId) =>
            this with { VisitedSwitches = new HashSet<string>(VisitedSwitches) { switchId } };

        public PlanningContext WithVisitedLoop(string key) =>
            this with { VisitedLoopKeys = new HashSet<string>(VisitedLoopKeys) { key } };

        public PlanningContext WithSkippedCar(ICar car) =>
            this with { SkippedCarIds = new HashSet<string>(SkippedCarIds) { car.id } };

        public PlanningContext WithSkippedCars(IEnumerable<ICar> cars) =>
            this with { SkippedCarIds = new HashSet<string>(SkippedCarIds.Concat(cars.Select(c => c.id))) };

        public PlanningContext WithPendingSplit(SplitInfo? split) =>
            this with { PendingSplit = split };

        public PlanningContext WithClearedSwitches() =>
            this with { VisitedSwitches = new HashSet<string>() };

        public bool IsCarSkipped(ICar car) => SkippedCarIds.Contains(car.id);

        public string Serialize()
        {
            var obj = new JObject
            {
                ["visitedSwitches"] = new JArray(VisitedSwitches.ToArray()),
                ["visitedLoopKeys"] = new JArray(VisitedLoopKeys.ToArray()),
                ["skippedCarIds"] = new JArray(SkippedCarIds.ToArray())
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

            var skippedCarIds = new HashSet<string>(
                (obj["skippedCarIds"] as JArray)?.Select(t => t.Value<string>())
                ?? Enumerable.Empty<string>());

            return new PlanningContext(visitedSwitches, visitedLoopKeys, skippedCarIds, null);
        }
    }
}
