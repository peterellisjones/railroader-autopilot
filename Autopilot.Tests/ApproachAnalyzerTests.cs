using NUnit.Framework;
using System.Collections.Generic;
using Autopilot.Model;
using Autopilot.Planning;

namespace Autopilot.Tests
{
    [TestFixture]
    public class ApproachAnalyzerTests
    {
        // Test 2.1: Direct route, 0 reversals — tail leads
        [Test]
        public void Direct_route_zero_reversals_tail_leads()
        {
            // lead(n0--sw1) -- siding1(sw1--n1)
            //                \- siding2(sw1--n2)
            // Tail outward at lead:400 facing EndB (toward sw1)
            // Destination at siding2:100
            // BFS route: lead→siding2, 0 reversals
            // tailFacesDest=true (outward end faces toward switch/dest)
            // tailLeads = true XOR false(0 reversals) = true
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding1", "sw1", "n1", 200)
                .AddSegment("siding2", "sw1", "n2", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding2")
                .PlaceLoco("loco", "lead", 300, Direction.TowardEndB)
                .Build();

            var routeChecker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var analyzer = new ApproachAnalyzer(routeChecker, scenario.GraphAdapter,
                NullPlanningLogger.Instance);

            // Tail outward at 400 (toward EndB/sw1), inward at 390 (toward EndA/n0)
            var tailOutward = new GraphPosition("lead", 400, Direction.TowardEndB);
            var tailInward = new GraphPosition("lead", 390, Direction.TowardEndA);
            var dest = new GraphPosition("siding2", 100, Direction.TowardEndA);

            var result = analyzer.CheckApproachDirection(tailOutward, tailInward, dest);
            Assert.That(result, Is.True, "Tail should lead with 0 reversals when facing dest");
        }

        // Test 2.2: Route with 1 reversal — loco leads
        [Test]
        public void One_reversal_loco_leads()
        {
            // seg1(n0--sw1) -- seg2(sw1--sw2) -- seg3(sw2--n1)
            //                                  \- seg4(sw2--n2)
            // Tail outward at seg1:150 facing EndB (toward sw1)
            // Destination at seg4:100
            // Registered route with 1 reversal: seg1→seg2→seg4
            // tailFacesDest=true, 1 reversal (odd) → tailLeads = true XOR true = false
            var scenario = new ScenarioBuilder()
                .AddSegment("seg1", "n0", "sw1", 200)
                .AddSegment("seg2", "sw1", "sw2", 200)
                .AddSegment("seg3", "sw2", "n1", 200)
                .AddSegment("seg4", "sw2", "n2", 200)
                .AddSwitch("sw1", "seg1", "seg2", "seg2")
                .AddSwitch("sw2", "seg2", "seg3", "seg4")
                .PlaceLoco("loco", "seg1", 190)
                .Build();

            // Register route with 1 reversal (BFS wouldn't produce this naturally)
            scenario.GraphAdapter.RegisterRoute(
                new GraphPosition("seg1", 150, Direction.TowardEndB),
                new GraphPosition("seg4", 100, Direction.TowardEndA),
                new RouteResult(500f, 1, false, new List<string> { "seg1", "seg2", "seg4" }));
            // Also register for the flipped direction (same segment key)
            scenario.GraphAdapter.RegisterRoute(
                new GraphPosition("seg1", 150, Direction.TowardEndA),
                new GraphPosition("seg4", 100, Direction.TowardEndA),
                new RouteResult(700f, 1, false, new List<string> { "seg1", "seg2", "seg4" }));

            var routeChecker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var analyzer = new ApproachAnalyzer(routeChecker, scenario.GraphAdapter,
                NullPlanningLogger.Instance);

            // Tail outward at 150 (toward EndB/sw1), inward at 140 (toward n0)
            var tailOutward = new GraphPosition("seg1", 150, Direction.TowardEndB);
            var tailInward = new GraphPosition("seg1", 140, Direction.TowardEndA);
            var dest = new GraphPosition("seg4", 100, Direction.TowardEndA);

            var result = analyzer.CheckApproachDirection(tailOutward, tailInward, dest);
            // tailFacesDest=true, 1 reversal (odd) → tailLeads = true XOR true = false
            Assert.That(result, Is.False, "Loco should lead with 1 reversal");
        }

        // Test 2.3: Route with 2 reversals — tail leads
        [Test]
        public void Two_reversals_tail_leads()
        {
            // seg1(n0--sw1) -- seg2(sw1--n1)
            // Tail outward at seg1:150 facing EndB (toward sw1)
            // Destination at seg2:100
            // Registered route with 2 reversals
            // tailFacesDest=true, 2 reversals (even) → tailLeads = true XOR false = true
            var scenario = new ScenarioBuilder()
                .AddSegment("seg1", "n0", "sw1", 200)
                .AddSegment("seg2", "sw1", "n1", 200)
                .AddSwitch("sw1", "seg1", "seg2", "seg2")
                .PlaceLoco("loco", "seg1", 190)
                .Build();

            scenario.GraphAdapter.RegisterRoute(
                new GraphPosition("seg1", 150, Direction.TowardEndB),
                new GraphPosition("seg2", 100, Direction.TowardEndA),
                new RouteResult(400f, 2, false, new List<string> { "seg1", "seg2" }));
            scenario.GraphAdapter.RegisterRoute(
                new GraphPosition("seg1", 150, Direction.TowardEndA),
                new GraphPosition("seg2", 100, Direction.TowardEndA),
                new RouteResult(600f, 2, false, new List<string> { "seg1", "seg2" }));

            var routeChecker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var analyzer = new ApproachAnalyzer(routeChecker, scenario.GraphAdapter,
                NullPlanningLogger.Instance);

            // Tail outward at 150 (toward EndB/sw1), inward at 140
            var tailOutward = new GraphPosition("seg1", 150, Direction.TowardEndB);
            var tailInward = new GraphPosition("seg1", 140, Direction.TowardEndA);
            var dest = new GraphPosition("seg2", 100, Direction.TowardEndA);

            var result = analyzer.CheckApproachDirection(tailOutward, tailInward, dest);
            // tailFacesDest=true, 2 reversals (even) → tailLeads = true XOR false = true
            Assert.That(result, Is.True, "Tail should lead with 2 reversals (even parity)");
        }

        // Test 2.4: Tail faces away from destination
        [Test]
        public void Tail_faces_away_from_dest()
        {
            // lead(n0--sw1) -- siding(sw1--n1)
            // Tail outward at lead:100 facing EndA (away from sw1/siding)
            // Destination at siding:100
            // BFS route: lead→siding, 0 reversals
            // tailFacesDest=false (outward end faces away from dest)
            // tailLeads = false XOR false(0 reversals) = false
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding", "sw1", "n1", 200)
                .AddSwitch("sw1", "lead", "siding", "siding")
                .PlaceLoco("loco", "lead", 300)
                .Build();

            var routeChecker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var analyzer = new ApproachAnalyzer(routeChecker, scenario.GraphAdapter,
                NullPlanningLogger.Instance);

            // Tail outward at 100 facing EndA (toward n0, away from sw1)
            // Inward at 110 (toward loco at higher distance)
            var tailOutward = new GraphPosition("lead", 100, Direction.TowardEndA);
            var tailInward = new GraphPosition("lead", 110, Direction.TowardEndB);
            var dest = new GraphPosition("siding", 100, Direction.TowardEndA);

            var result = analyzer.CheckApproachDirection(tailOutward, tailInward, dest);
            // tailFacesDest=false (facing away), 0 reversals → tailLeads = false XOR false = false
            Assert.That(result, Is.False, "Tail faces away with 0 reversals — loco should lead");
        }
    }
}
