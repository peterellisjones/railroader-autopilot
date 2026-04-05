using NUnit.Framework;
using System.Collections.Generic;
using Autopilot.Model;
using Autopilot.Planning;

namespace Autopilot.Tests
{
    [TestFixture]
    public class EdgeCaseTests
    {
        // --- Test 1: Dead-end siding ---
        // A segment with only one connection (dead end) should still be routable.
        [Test]
        public void RouteChecker_dead_end_siding_is_routable()
        {
            // lead(n0--sw1) -- siding(sw1--deadEnd)
            // "deadEnd" is a non-switch node with no other segments attached,
            // so "siding" is a dead-end. The loco should still be able to route to it.
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding", "sw1", "deadEnd", 200)
                .AddSwitch("sw1", "lead", "siding", "siding")
                .PlaceLoco("loco", "lead", 250, Direction.TowardEndB)
                .Build();

            var checker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var dest = new GraphPosition("siding", 150, Direction.TowardEndA);

            Assert.That(checker.CanRouteTo(dest), Is.True,
                "Dead-end siding should still be routable from the lead");
        }

        // --- Test 2: Same-segment destination ---
        // Loco and destination on the same segment. Route should work and
        // distance should equal the position difference.
        [Test]
        public void RouteChecker_same_segment_destination_returns_position_difference()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("seg1", "n0", "n1", 500)
                .PlaceLoco("loco", "seg1", 100, Direction.TowardEndB)
                .Build();

            var checker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var target = new GraphPosition("seg1", 350, Direction.TowardEndA);

            var result = checker.GraphDistanceToLoco(target);
            Assert.That(result, Is.Not.Null);
            // Loco at 100, target at 350 => distance = 250
            Assert.That(result.Value.Distance, Is.EqualTo(250f).Within(0.01f),
                "Same-segment distance should be the absolute position difference");
        }

        // --- Test 3: Long chain of segments ---
        // Route through 4+ segments with multiple switches. Verify route is found
        // and distance accumulates correctly.
        [Test]
        public void RouteChecker_long_chain_through_multiple_switches()
        {
            // seg1(n0--sw1) -- seg2(sw1--sw2) -- seg3(sw2--sw3) -- seg4(sw3--n1)
            //                          \- seg5(sw2--n2)       \- seg6(sw3--n3)
            var scenario = new ScenarioBuilder()
                .AddSegment("seg1", "n0", "sw1", 100)
                .AddSegment("seg2", "sw1", "sw2", 100)
                .AddSegment("seg3", "sw2", "sw3", 100)
                .AddSegment("seg4", "sw3", "n1", 100)
                .AddSegment("seg5", "sw2", "n2", 100)
                .AddSegment("seg6", "sw3", "n3", 100)
                .AddSwitch("sw1", "seg1", "seg2", "seg2")
                .AddSwitch("sw2", "seg2", "seg3", "seg5")
                .AddSwitch("sw3", "seg3", "seg4", "seg6")
                .PlaceLoco("loco", "seg1", 50, Direction.TowardEndB)
                .Build();

            var checker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var dest = new GraphPosition("seg4", 50, Direction.TowardEndA);

            var result = checker.GraphDistanceToLoco(dest);
            Assert.That(result, Is.Not.Null, "Route through 4 segments should be found");
            // seg1: 50 to EndB (50m), seg2: full (100m), seg3: full (100m), seg4: 50m from EndA
            // Total: 50 + 100 + 100 + 50 = 300
            Assert.That(result.Value.Distance, Is.EqualTo(300f).Within(0.01f),
                "Distance should accumulate through the chain");
        }

        // --- Test 4: GraphDistanceToLoco where target is on loco's segment ---
        [Test]
        public void GraphDistanceToLoco_target_on_same_segment_as_loco()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("seg1", "n0", "n1", 1000)
                .PlaceLoco("loco", "seg1", 200, Direction.TowardEndB)
                .Build();

            var checker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var target = new GraphPosition("seg1", 800, Direction.TowardEndA);

            var result = checker.GraphDistanceToLoco(target);
            Assert.That(result, Is.Not.Null, "Target on same segment should always be reachable");
            Assert.That(result.Value.Distance, Is.EqualTo(600f).Within(0.01f),
                "Distance should be |800 - 200| = 600");
            Assert.That(result.Value.ReversalCount, Is.EqualTo(0),
                "Same segment route should have 0 reversals");
        }

        // --- Test 5: Approach with same-segment tail and destination ---
        // Should not crash and should handle the single-segment route correctly.
        [Test]
        public void ApproachAnalyzer_tail_and_dest_on_same_segment()
        {
            // Single segment scenario — tail and dest are both on "seg1"
            var scenario = new ScenarioBuilder()
                .AddSegment("seg1", "n0", "n1", 500)
                .PlaceLoco("loco", "seg1", 300, Direction.TowardEndB)
                .Build();

            var routeChecker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var analyzer = new ApproachAnalyzer(routeChecker, scenario.GraphAdapter,
                NullPlanningLogger.Instance);

            // Tail outward at 100 facing EndA, inward at 110 facing EndB
            // Destination at 50 (toward EndA from the tail)
            var tailOutward = new GraphPosition("seg1", 100, Direction.TowardEndA);
            var tailInward = new GraphPosition("seg1", 110, Direction.TowardEndB);
            var dest = new GraphPosition("seg1", 50, Direction.TowardEndA);

            // Should not throw. The route is on a single segment (routeSegmentIds count < 2),
            // so it defaults to tailFacesDest=true, reversals=0 => tailLeads=true.
            Assert.DoesNotThrow(() =>
            {
                analyzer.CheckApproachDirection(tailOutward, tailInward, dest);
            }, "Same-segment tail and dest should not crash");
        }

        // --- Test 6: Multiple blocked segments ---
        // Block 2 segments on different paths. Even though there are two routes,
        // blocking both should make the destination unreachable.
        [Test]
        public void RouteChecker_multiple_blocked_segments_all_paths_fail()
        {
            // lead(n0--sw1) -- pathA(sw1--n1)
            //                \- pathB(sw1--n2)
            // dest is reachable via either pathA or pathB, but both are blocked.
            // Actually, for BFS to find alternative paths, we need a topology
            // where two separate routes converge on the destination.
            //
            // lead(n0--sw1) -- mid1(sw1--sw2) -- dest(sw2--n3)
            //                \- mid2(sw1--sw2)
            //   (both mid1 and mid2 connect sw1 to sw2)
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 200)
                .AddSegment("mid1", "sw1", "sw2", 200)
                .AddSegment("mid2", "sw1", "sw2", 200)
                .AddSegment("dest", "sw2", "n3", 200)
                .AddSwitch("sw1", "lead", "mid1", "mid2")
                .AddSwitch("sw2", "dest", "mid1", "mid2")
                .PlaceLoco("loco", "lead", 100, Direction.TowardEndB)
                .Build();

            // Block both mid paths
            scenario.GraphAdapter.BlockSegment("mid1");
            scenario.GraphAdapter.BlockSegment("mid2");

            var checker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var target = new GraphPosition("dest", 100, Direction.TowardEndA);

            Assert.That(checker.CanRouteTo(target), Is.False,
                "Both paths blocked should make destination unreachable");
        }

        // --- Test 7: Block then unblock via ignoredCarIds ---
        // Block a segment, verify blocked, then verify CanRouteTo with ignoredCarIds unblocks it.
        [Test]
        public void RouteChecker_block_then_unblock_via_ignored_cars()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding", "sw1", "deadEnd", 200)
                .AddSwitch("sw1", "lead", "siding", "siding")
                .PlaceLoco("loco", "lead", 250, Direction.TowardEndB)
                .Build();

            scenario.GraphAdapter.BlockSegment("siding");

            var checker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var dest = new GraphPosition("siding", 100, Direction.TowardEndA);

            // Verify blocked without ignored cars
            Assert.That(checker.CanRouteTo(dest), Is.False,
                "Route should be blocked when segment is blocked");

            // Verify unblocked when the segment's ID is in ignoredCarIds
            // (MockGraphAdapter treats the segment ID as the "car ID" for ignore purposes)
            Assert.That(checker.CanRouteTo(dest, 0, new List<string> { "siding" }), Is.True,
                "Route should be unblocked when segment ID is in ignoredCarIds");
        }

        // --- Test 8: GraphDistanceToLoco picks shorter route ---
        // Loco is near a junction with two routes to the target; verify it returns
        // the shorter one.
        [Test]
        public void GraphDistanceToLoco_picks_shorter_of_two_routes()
        {
            // Loco on lead. Two paths to dest: short (100m) and long (400m).
            //
            //   lead(n0--sw1) -- short(sw1--sw2) -- dest(sw2--n3)
            //                 \- long(sw1--sw2)
            //   short is 100m, long is 400m, so BFS should find short path
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 200)
                .AddSegment("short", "sw1", "sw2", 100)
                .AddSegment("long", "sw1", "sw2", 400)
                .AddSegment("dest", "sw2", "n3", 200)
                .AddSwitch("sw1", "lead", "short", "long")
                .AddSwitch("sw2", "dest", "short", "long")
                .PlaceLoco("loco", "lead", 150, Direction.TowardEndB)
                .Build();

            var checker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var target = new GraphPosition("dest", 50, Direction.TowardEndA);

            var result = checker.GraphDistanceToLoco(target);
            Assert.That(result, Is.Not.Null, "Route should be found");
            // Shorter path: lead 150→EndB (50m) + short (100m) + dest EndA→50 (50m) = 200m
            // Longer path: lead 150→EndB (50m) + long (400m) + dest EndA→50 (50m) = 500m
            Assert.That(result.Value.Distance, Is.EqualTo(200f).Within(0.01f),
                "Should return the shorter route distance (200m via short path)");
        }

        // --- Test 9: Cross-segment tail (outward and inward on different segments) ---
        // Tests the cross-segment branch of RouteGoesOutward in ApproachAnalyzer.
        [Test]
        public void ApproachAnalyzer_cross_segment_tail_outward_and_inward()
        {
            // tailSeg(n0--jct) -- locoSeg(jct--sw1) -- siding(sw1--n2)
            // The tail outward is on tailSeg, the inward is on locoSeg.
            // The tail faces away from the junction (toward n0), while
            // the destination is past sw1 on siding.
            var scenario = new ScenarioBuilder()
                .AddSegment("tailSeg", "n0", "jct", 200)
                .AddSegment("locoSeg", "jct", "sw1", 200)
                .AddSegment("siding", "sw1", "n2", 200)
                .AddSwitch("sw1", "locoSeg", "siding", "siding")
                .PlaceLoco("loco", "locoSeg", 100, Direction.TowardEndB)
                .Build();

            var routeChecker = new RouteChecker(scenario.TrainService, scenario.GraphAdapter,
                NullPlanningLogger.Instance);
            var analyzer = new ApproachAnalyzer(routeChecker, scenario.GraphAdapter,
                NullPlanningLogger.Instance);

            // Tail outward on tailSeg at 180 facing EndA (away from jct — toward n0)
            // Tail inward on locoSeg at 20 facing EndB (toward sw1)
            // These are on different segments, connected through "jct"
            var tailOutward = new GraphPosition("tailSeg", 180, Direction.TowardEndA);
            var tailInward = new GraphPosition("locoSeg", 20, Direction.TowardEndB);
            var dest = new GraphPosition("siding", 100, Direction.TowardEndA);

            // The route from tailSeg to siding: tailSeg→locoSeg→siding
            // The connection node between tailSeg and locoSeg is "jct"
            // jct is at EndB of tailSeg, so connectIsEndA=false for tailSeg
            // The route exits tailSeg at EndB (toward jct), routeExitsEndA=false
            // RouteGoesOutward = (routeExitsEndA != connectIsEndA) = (false != false) = false
            // So tailFacesDest = false
            // 0 reversals => tailLeads = false XOR false = false
            var result = analyzer.CheckApproachDirection(tailOutward, tailInward, dest);
            Assert.That(result, Is.False,
                "Cross-segment tail facing away from dest should not lead");
        }

        // --- Test 10: FeasibilityChecker — correct approach but blocked route ---
        // The approach direction check passes, but the route to the destination
        // is blocked by a car. Should return false.
        [Test]
        public void FeasibilityChecker_correct_approach_but_blocked_returns_false()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding", "sw1", "deadEnd", 200)
                .AddSwitch("sw1", "lead", "siding", "siding")
                .PlaceLoco("loco", "lead", 300, Direction.TowardEndB)
                .Build();

            // Block the siding segment
            scenario.GraphAdapter.BlockSegment("siding");

            var checker = new FeasibilityChecker(scenario.TrainService, scenario.GraphAdapter);

            // Tail outward faces EndB (toward sw1/dest) — correct approach
            var tailOutward = new GraphPosition("lead", 410, Direction.TowardEndB);
            var tailInward = new GraphPosition("lead", 400, Direction.TowardEndA);
            var dest = new GraphPosition("siding", 100, Direction.TowardEndA);

            // Approach direction is correct (tail faces dest with 0 reversals),
            // but the siding is blocked => CanDeliver should fail on the route check
            Assert.That(checker.CanDeliver(tailOutward, tailInward, dest), Is.False,
                "Correct approach but blocked route should return false");
        }
    }
}
