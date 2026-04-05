using NUnit.Framework;
using System.Collections.Generic;
using Autopilot.Model;
using Autopilot.Planning;

namespace Autopilot.Tests
{
    [TestFixture]
    public class RouteCheckerTests
    {
        [Test]
        public void CanRouteTo_direct_route_returns_true()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding1", "sw1", "n1", 200)
                .AddSegment("siding2", "sw1", "n2", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding2")
                .PlaceLoco("loco", "lead", 250, Direction.TowardEndB)
                .Build();

            var checker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var dest = new GraphPosition("siding1", 100, Direction.TowardEndA);

            Assert.That(checker.CanRouteTo(dest), Is.True);
        }

        [Test]
        public void CanRouteTo_blocked_segment_returns_false()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding1", "sw1", "n1", 200)
                .AddSegment("siding2", "sw1", "n2", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding2")
                .PlaceLoco("loco", "lead", 250, Direction.TowardEndB)
                .Build();

            scenario.GraphAdapter.BlockSegment("siding1");

            var checker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var dest = new GraphPosition("siding1", 100, Direction.TowardEndA);

            Assert.That(checker.CanRouteTo(dest), Is.False);
        }

        [Test]
        public void CanRouteTo_tries_both_loco_directions()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding1", "sw1", "n1", 200)
                .AddSegment("siding2", "sw1", "n2", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding2")
                .PlaceLoco("loco", "lead", 250, Direction.TowardEndA) // facing away from switch
                .Build();

            var checker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var dest = new GraphPosition("siding1", 100, Direction.TowardEndA);

            // Rear faces toward switch, should still find route
            Assert.That(checker.CanRouteTo(dest), Is.True);
        }

        [Test]
        public void GraphDistanceToLoco_returns_distance()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding1", "sw1", "n1", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding1")
                .PlaceLoco("loco", "lead", 100, Direction.TowardEndB)
                .Build();

            var checker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var target = new GraphPosition("siding1", 50, Direction.TowardEndA);

            var result = checker.GraphDistanceToLoco(target);
            Assert.That(result, Is.Not.Null);
            Assert.That(result.Value.Distance, Is.GreaterThan(0));
        }

        [Test]
        public void GraphDistanceToLoco_unreachable_returns_null()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("seg1", "n0", "n1", 200)
                .AddSegment("seg2", "n2", "n3", 200) // disconnected
                .PlaceLoco("loco", "seg1", 100)
                .Build();

            var checker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var target = new GraphPosition("seg2", 100, Direction.TowardEndA);

            Assert.That(checker.GraphDistanceToLoco(target), Is.Null);
        }

        [Test]
        public void CanRouteTo_with_ignored_cars_bypasses_block()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding1", "sw1", "n1", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding1")
                .PlaceLoco("loco", "lead", 250, Direction.TowardEndB)
                .Build();

            scenario.GraphAdapter.BlockSegment("siding1");

            var checker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var dest = new GraphPosition("siding1", 100, Direction.TowardEndA);

            // Blocked without ignoring
            Assert.That(checker.CanRouteTo(dest), Is.False);
            // Unblocked when ignoring
            Assert.That(checker.CanRouteTo(dest, 0, new List<string> { "siding1" }), Is.True);
        }
    }
}
