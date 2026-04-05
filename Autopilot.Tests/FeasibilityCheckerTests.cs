using NUnit.Framework;
using Autopilot.Model;
using Autopilot.Planning;

namespace Autopilot.Tests
{
    [TestFixture]
    public class FeasibilityCheckerTests
    {
        [Test]
        public void CanDeliver_reachable_correct_approach_returns_true()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding1", "sw1", "n1", 200)
                .AddSegment("siding2", "sw1", "n2", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding2")
                .PlaceLoco("loco", "lead", 300, Direction.TowardEndB)
                .Build();

            var checker = new FeasibilityChecker(scenario.TrainService, scenario.GraphAdapter);

            // Tail outward faces EndB (toward sw1/destination)
            // Outward end is at higher distance (closer to EndB) than inward end
            var tailOutward = new GraphPosition("lead", 210, Direction.TowardEndB);
            var tailInward = new GraphPosition("lead", 200, Direction.TowardEndA);
            var dest = new GraphPosition("siding2", 100, Direction.TowardEndA);

            Assert.That(checker.CanDeliver(tailOutward, tailInward, dest), Is.True);
        }

        [Test]
        public void CanDeliver_unreachable_returns_false()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("seg1", "n0", "n1", 200)
                .AddSegment("seg2", "n2", "n3", 200) // disconnected
                .PlaceLoco("loco", "seg1", 100)
                .Build();

            var checker = new FeasibilityChecker(scenario.TrainService, scenario.GraphAdapter);

            var tailOutward = new GraphPosition("seg1", 50, Direction.TowardEndB);
            var tailInward = new GraphPosition("seg1", 60, Direction.TowardEndA);
            var dest = new GraphPosition("seg2", 100, Direction.TowardEndA);

            Assert.That(checker.CanDeliver(tailOutward, tailInward, dest), Is.False);
        }

        [Test]
        public void CanDeliver_wrong_approach_returns_false()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding", "sw1", "n1", 200)
                .AddSwitch("sw1", "lead", "siding", "siding")
                .PlaceLoco("loco", "lead", 300)
                .Build();

            var checker = new FeasibilityChecker(scenario.TrainService, scenario.GraphAdapter);

            // Tail faces EndA (away from switch/dest) -> approach wrong
            var tailOutward = new GraphPosition("lead", 200, Direction.TowardEndA);
            var tailInward = new GraphPosition("lead", 210, Direction.TowardEndB);
            var dest = new GraphPosition("siding", 100, Direction.TowardEndA);

            Assert.That(checker.CanDeliver(tailOutward, tailInward, dest), Is.False);
        }

        [Test]
        public void CanDeliver_blocked_route_returns_false()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding", "sw1", "n1", 200)
                .AddSwitch("sw1", "lead", "siding", "siding")
                .PlaceLoco("loco", "lead", 300, Direction.TowardEndB)
                .Build();

            scenario.GraphAdapter.BlockSegment("siding");

            var checker = new FeasibilityChecker(scenario.TrainService, scenario.GraphAdapter);

            // Outward end faces EndB (toward sw1/dest) — correct approach
            // Outward end is at higher distance (closer to EndB) than inward end
            var tailOutward = new GraphPosition("lead", 210, Direction.TowardEndB);
            var tailInward = new GraphPosition("lead", 200, Direction.TowardEndA);
            var dest = new GraphPosition("siding", 100, Direction.TowardEndA);

            // Approach is correct, but route is blocked → should return false
            Assert.That(checker.CanDeliver(tailOutward, tailInward, dest), Is.False);
        }
    }
}
