using System.Collections.Generic;
using System.Linq;
using Autopilot.Planning;
using NUnit.Framework;

namespace Autopilot.Tests
{
    [TestFixture]
    public class LoopFinderTests
    {
        // --- Simple passing siding ---
        //
        //              SwitchA                     SwitchB
        //  stem_a ------[A]------- main -----------[B]------ stem_b
        //                 \_____ siding ___________/
        //
        // SwitchA: enter=stem_a, exitNormal=main, exitReverse=siding
        // SwitchB: enter=stem_b, exitNormal=main, exitReverse=siding
        //
        // Loco starts on stem_a, somewhere in the middle.
        // Expected: loop detected between A and B, train fits on siding (or main).

        private MockGraphAdapter BuildPassingSiding(
            float mainLength = 200f, float sidingLength = 180f,
            float stemALength = 100f, float stemBLength = 100f,
            float fouling = 10f)
        {
            return new MockGraphAdapter()
                .AddSegment("stem_a", "n_left", "A", stemALength)
                .AddSegment("main",   "A",      "B", mainLength)
                .AddSegment("siding", "A",      "B", sidingLength)
                .AddSegment("stem_b", "B", "n_right", stemBLength)
                .AddSwitch("A", "stem_a", "main", "siding", fouling)
                .AddSwitch("B", "stem_b", "main", "siding", fouling);
        }

        [Test]
        public void PassingSiding_DetectsLoop()
        {
            var graph = BuildPassingSiding();
            var finder = new LoopFinder(graph);

            // Loco on stem_a, 50m from End.A (n_left), 50m from End.B (SwitchA)
            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNotNull(loop, "Should detect a loop");
            Assert.That(loop.Branches.Count, Is.EqualTo(2));
            // One switch should be A, the other B
            var switchIds = new HashSet<string> { loop.SwitchAId, loop.SwitchBId };
            Assert.That(switchIds, Contains.Item("A"));
            Assert.That(switchIds, Contains.Item("B"));
        }

        [Test]
        public void PassingSiding_TrainFitsOnSiding()
        {
            var graph = BuildPassingSiding(mainLength: 200f, sidingLength: 180f, fouling: 10f);
            var finder = new LoopFinder(graph);

            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 100f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNotNull(loop);
            // Usable length of each branch: length - fouling*2
            // Main: 200 - 10 - 10 = 180. Siding: 180 - 10 - 10 = 160. Both fit 100m train.
            Assert.That(loop.FittingBranch.UsableLength, Is.GreaterThanOrEqualTo(100f));
        }

        [Test]
        public void PassingSiding_TrainDoesNotFit()
        {
            // Very short siding, long train
            var graph = BuildPassingSiding(mainLength: 50f, sidingLength: 40f, fouling: 10f);
            var finder = new LoopFinder(graph);

            // Main usable: 50 - 10 - 10 = 30. Siding usable: 40 - 10 - 10 = 20.
            // Train = 100m, doesn't fit either branch.
            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 100f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNull(loop, "Train should not fit on any branch");
        }

        [Test]
        public void PassingSiding_EntrySwitchDetermination()
        {
            var graph = BuildPassingSiding();
            var finder = new LoopFinder(graph);

            // Only switchA is reachable (loco comes from stem_a side)
            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) =>
                {
                    // Can reach switch A (on stem_a or main/siding near A)
                    // but not switch B
                    if (segId == "stem_a" || segId == "main" || segId == "siding")
                    {
                        // endA=true means switch is at End.A of the segment
                        var nodeAtEnd = graph.GetNodeAtEnd(segId, endA);
                        return nodeAtEnd == "A";
                    }
                    return false;
                });

            Assert.IsNotNull(loop);
            Assert.AreEqual("A", loop.EntrySwitchId, "Entry should be switch A (reachable)");
            Assert.AreEqual("B", loop.ExitSwitchId, "Far should be switch B");
        }

        // --- Dead-end siding (no loop) ---
        //
        //  buffer ---- seg1 ----[A]---- seg2 ---- n_end
        //
        // Only one switch, no second path. Should return null.

        [Test]
        public void DeadEnd_NoLoopFound()
        {
            var graph = new MockGraphAdapter()
                .AddSegment("seg1", "buffer", "A", 100f)
                .AddSegment("seg2", "A", "n_end", 100f)
                .AddSegment("seg3", "A", "n_spur", 50f)
                .AddSwitch("A", "seg1", "seg2", "seg3", 10f);

            var finder = new LoopFinder(graph);

            var loop = finder.FindNearestFittingLoop(
                "seg1", distToA: 50f, distToB: 50f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNull(loop, "Dead-end siding should not detect a loop");
        }

        // --- Multiple loops, closest selected ---
        //
        //  stem --[A]-- main1 --[B]-- main2 --[C]-- main3 --[D]-- stem2
        //           \_ siding1 _/       \____ siding2 ____/
        //
        // Loop A-B (close) and loop C-D (far). Should select A-B.

        [Test]
        public void MultipleLoops_ClosestSelected()
        {
            var graph = new MockGraphAdapter()
                .AddSegment("stem",     "n_left", "A", 50f)
                .AddSegment("main1",    "A", "B", 100f)
                .AddSegment("siding1",  "A", "B", 90f)
                .AddSegment("main2",    "B", "C", 200f)
                .AddSegment("main3",    "C", "D", 100f)
                .AddSegment("siding2",  "C", "D", 90f)
                .AddSegment("stem2",    "D", "n_right", 50f)
                .AddSwitch("A", "stem",  "main1", "siding1", 10f)
                .AddSwitch("B", "main2", "main1", "siding1", 10f)
                .AddSwitch("C", "main2", "main3", "siding2", 10f)
                .AddSwitch("D", "stem2", "main3", "siding2", 10f);

            var finder = new LoopFinder(graph);

            // Loco on stem, near switch A
            var loop = finder.FindNearestFittingLoop(
                "stem", distToA: 40f, distToB: 10f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNotNull(loop);
            // Should be the A-B loop (closer)
            var switchIds = new HashSet<string> { loop.SwitchAId, loop.SwitchBId };
            Assert.That(switchIds, Contains.Item("A"), "Should use the closer loop");
            Assert.That(switchIds, Contains.Item("B"), "Should use the closer loop");
        }

        // --- Train on spur, loop on mainline ---
        //
        // This is the topology from the user's bug report.
        // The spur branches off the mainline at a junction switch.
        // The passing siding is on the mainline, NOT directly on the spur path.
        //
        //           SwitchL         SwitchR
        //  left ------[L]--- main ---[R]------ right
        //              |\__ siding __/
        //              |
        //              | (junction exit from L)
        //           SwitchJ
        //   mainL ----[J]
        //              \__ spur __ spur_end
        //
        // Loco is on the spur. Walk reaches SwitchJ, then must explore
        // BOTH directions from J to find the loop L-R.

        [Test]
        public void SpurToMainline_FindsNearbyLoop()
        {
            // Topology:
            //   n_left --leftJ-- [J] --juncToL-- [L] --main-- [R] --right-- n_right
            //                     |               |             |
            //                    spur             siding--------/
            //                     |
            //                  spur_end
            //
            // J is a junction switch. Its enter=leftJ (mainline left),
            // exitNormal=juncToL (mainline toward L), exitReverse=spur.
            // L's enter=juncToL (from J side). R's enter=right.
            var graph = new MockGraphAdapter()
                .AddSegment("leftJ",   "n_left", "J", 100f)
                .AddSegment("juncToL", "J", "L", 30f)
                .AddSegment("main",    "L", "R", 150f)
                .AddSegment("siding",  "L", "R", 140f)
                .AddSegment("right",   "R", "n_right", 100f)
                .AddSegment("spur",    "J", "spur_end", 60f)
                // J: enter=leftJ (mainline), exitNormal=juncToL (toward loop), exitReverse=spur
                .AddSwitch("J", "leftJ", "juncToL", "spur", 10f)
                // L: enter=juncToL (from J), exitNormal=main, exitReverse=siding
                .AddSwitch("L", "juncToL", "main", "siding", 10f)
                // R: enter=right, exitNormal=main, exitReverse=siding
                .AddSwitch("R", "right", "main", "siding", 10f);

            var finder = new LoopFinder(graph);

            // Loco on spur, 30m from J
            var loop = finder.FindNearestFittingLoop(
                "spur", distToA: 30f, distToB: 30f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNotNull(loop, "Should find the L-R loop via junction J");
            var switchIds = new HashSet<string> { loop.SwitchAId, loop.SwitchBId };
            Assert.That(switchIds, Contains.Item("L"));
            Assert.That(switchIds, Contains.Item("R"));
        }

        // --- Visited switches are skipped ---

        [Test]
        public void VisitedSwitches_SkipsAlreadyTried()
        {
            var graph = BuildPassingSiding();
            var finder = new LoopFinder(graph);

            var visited = new HashSet<string> { "A", "B" };

            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 50f, visitedSwitches: visited,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNull(loop, "Both loop switches are visited, should find nothing");
        }

        // --- Wye topology ---
        //
        // Three switches forming a triangle. Each pair connected by one leg.
        //
        //        stem_a
        //          |
        //         [A]
        //        /   \
        //    legAB    legAC
        //      /       \
        //    [B]-------[C]
        //     |  legBC  |
        //  stem_b    stem_c

        [Test]
        public void Wye_DetectsLoop()
        {
            // For a wye, each switch's enter leg must be a triangle leg so the
            // algorithm can traverse through. B's enter=legAB means arriving from
            // A goes enter→exit, enabling traversal to C via legBC.
            var graph = new MockGraphAdapter()
                .AddSegment("stem_a", "n_a", "A", 80f)
                .AddSegment("legAB",  "A", "B", 60f)
                .AddSegment("legAC",  "A", "C", 60f)
                .AddSegment("legBC",  "B", "C", 60f)
                .AddSegment("stem_b", "B", "n_b", 80f)
                .AddSegment("stem_c", "C", "n_c", 80f)
                // A: enter=stem_a, exits to B and C
                .AddSwitch("A", "stem_a", "legAB", "legAC", 10f)
                // B: enter=legAB (from A), exitNormal=legBC (to C), exitReverse=stem_b
                .AddSwitch("B", "legAB", "legBC", "stem_b", 10f)
                // C: enter=legAC (from A), exitNormal=legBC (to B), exitReverse=stem_c
                .AddSwitch("C", "legAC", "legBC", "stem_c", 10f);

            var finder = new LoopFinder(graph);

            // Loco on stem_a
            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 40f, distToB: 40f,
                trainLength: 30f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNotNull(loop, "Should detect a loop in the wye");
        }

        // --- Complex junction (intermediate switch on branch) ---
        //
        //         SwitchA                  SwitchB
        //  stem --[A]------ main ----------[B]-- stem2
        //          \__ sid1 __[M]__ sid2 __/
        //                      |
        //                   spur (dead-end branch off siding)
        //
        // M is an intermediate switch on the siding branch.
        // The loop A-B should still be detected; M is recorded as intermediate.

        [Test]
        public void IntermediateSwitch_LoopStillDetected()
        {
            // M is an intermediate switch ON the siding. Its enter=sid1 (through-route
            // from A), exitNormal=sid2 (through-route to B), exitReverse=spur (branch).
            // This lets the algorithm traverse the siding: A → sid1 → M(enter→exitN) → sid2 → B
            var graph = new MockGraphAdapter()
                .AddSegment("stem",  "n_left", "A", 100f)
                .AddSegment("main",  "A", "B", 200f)
                .AddSegment("sid1",  "A", "M", 80f)
                .AddSegment("sid2",  "M", "B", 80f)
                .AddSegment("spur",  "M", "n_spur", 50f)
                .AddSegment("stem2", "B", "n_right", 100f)
                .AddSwitch("A", "stem", "main", "sid1", 10f)
                .AddSwitch("B", "stem2", "main", "sid2", 10f)
                // M: enter=sid1 (siding through-route), exitNormal=sid2 (siding through-route), exitReverse=spur
                .AddSwitch("M", "sid1", "sid2", "spur", 5f);

            var finder = new LoopFinder(graph);

            var loop = finder.FindNearestFittingLoop(
                "stem", distToA: 50f, distToB: 50f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNotNull(loop, "Should detect loop A-B despite intermediate switch M");
            var switchIds = new HashSet<string> { loop.SwitchAId, loop.SwitchBId };
            Assert.That(switchIds, Contains.Item("A"));
            Assert.That(switchIds, Contains.Item("B"));

            // The siding branch should have M as an intermediate switch
            var sidingBranch = loop.Branches.FirstOrDefault(b =>
                b.SegmentIds.Contains("sid1") || b.SegmentIds.Contains("sid2"));
            Assert.IsNotNull(sidingBranch, "Should have a siding branch");
            Assert.That(sidingBranch.IntermediateSwitchIds, Contains.Item("M"),
                "M should be an intermediate switch on the siding branch");
        }

        // --- Fouling distance calculation ---

        [Test]
        public void FoulingCalculation_ExitLegUsesFullFouling()
        {
            // Both branches connect via exit legs → full fouling at both ends
            var graph = BuildPassingSiding(mainLength: 100f, sidingLength: 100f, fouling: 15f);
            var finder = new LoopFinder(graph);

            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 10f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNotNull(loop);
            // Both branches connect to A and B via exit legs, so fouling = 15 at each end
            Assert.AreEqual(15f, loop.FittingBranch.FoulingAtStart, 0.01f);
            Assert.AreEqual(15f, loop.FittingBranch.FoulingAtEnd, 0.01f);
            // Usable = 100 - 15 - 15 = 70
            Assert.AreEqual(70f, loop.FittingBranch.UsableLength, 0.01f);
        }

        [Test]
        public void FoulingCalculation_EnterLegUsesMinimalFouling()
        {
            // If a branch connects to a switch's enter (stem) leg, fouling is 2m.
            // Set up: SwitchA enter=main (branch connects via enter leg),
            // so fouling at A for the main branch is 2m, not 15m.
            var graph = new MockGraphAdapter()
                .AddSegment("left",    "n_left", "A", 100f)
                .AddSegment("main",    "A", "B", 200f)
                .AddSegment("siding",  "A", "B", 180f)
                .AddSegment("right",   "B", "n_right", 100f)
                // A: enter=main (so the main branch connects via the enter leg of A)
                .AddSwitch("A", "main", "left", "siding", 15f)
                // B: enter=right
                .AddSwitch("B", "right", "main", "siding", 15f);

            var finder = new LoopFinder(graph);

            var loop = finder.FindNearestFittingLoop(
                "left", distToA: 50f, distToB: 50f,
                trainLength: 10f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNotNull(loop);

            // Find the main branch (contains segment "main")
            var mainBranch = loop.Branches.FirstOrDefault(b => b.SegmentIds.Contains("main"));
            Assert.IsNotNull(mainBranch);
            // main connects to A via enter → fouling at start = 2m (minimal)
            // main connects to B via exit → fouling at end = 15m
            Assert.AreEqual(2f, mainBranch.FoulingAtStart, 0.01f,
                "Enter leg should use 2m minimal fouling");
            Assert.AreEqual(15f, mainBranch.FoulingAtEnd, 0.01f,
                "Exit leg should use full fouling");
        }

        // --- Train overlaps entry switch ---
        //
        // Branch is 100m, fouling 10m at each end.
        // Old fit check: usable = 100 - 10 - 10 = 80m. Train 85m → REJECTED.
        // New fit check: fittingLength = 100 - 10 (far only) = 90m. Train 85m → ACCEPTED.
        // The train's tail extends past the entry switch onto the stem. This is
        // fine because the loco detaches at the far end and goes around via the
        // other branch, never needing to pass back through the entry switch.

        [Test]
        public void TrainOverlapsEntrySwitch_StillFits()
        {
            var graph = BuildPassingSiding(mainLength: 200f, sidingLength: 100f, fouling: 10f);
            var finder = new LoopFinder(graph);

            // Train = 85m. Siding usable (old) = 100 - 10 - 10 = 80 → too short.
            // But fitting length (new) = 100 - 10 (far fouling only) = 90 → fits!
            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 85f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNotNull(loop, "Train should fit — tail overlaps entry switch, which is OK");
        }

        [Test]
        public void TrainTooLongEvenWithOverlap_Rejected()
        {
            var graph = BuildPassingSiding(mainLength: 200f, sidingLength: 100f, fouling: 10f);
            var finder = new LoopFinder(graph);

            // Train = 95m. Fitting length = 100 - 10 = 90 → still too short.
            // Even allowing entry overlap, the train doesn't fit.
            // But main branch: fitting length = 200 - 10 = 190 → fits.
            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 95f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNotNull(loop, "Should fit on the main branch");
            // Should NOT use the 100m siding (even with overlap, 95 > 90)
            Assert.That(loop.FittingBranch.Length, Is.GreaterThanOrEqualTo(200f),
                "Should use the longer main branch, not the short siding");
        }

        // --- Arrival via loop branch (not stem) ---
        //
        // The BFS might reach a loop switch via one of its BRANCH legs rather
        // than the stem. The old code only checked the two non-arrival legs,
        // missing the loop. The fix checks ALL pairs of legs.
        //
        //   n_far ---- far_track ---- [A] ---- main ---- [B] ---- stem_b
        //                              \_____ siding _____/
        //
        // Loco arrives at A via "main" (exitNormal). Old code explored
        // enter(far_track) + exitR(siding) — no common switch. Fixed code
        // checks exitN(main) + exitR(siding) and finds B as common.

        [Test]
        public void ArrivalViaLoopBranch_StillDetectsLoop()
        {
            var graph = new MockGraphAdapter()
                .AddSegment("far_track", "n_far", "A", 300f)
                .AddSegment("main",      "A", "B", 150f)
                .AddSegment("siding",    "A", "B", 140f)
                .AddSegment("stem_b",    "B", "n_right", 100f)
                .AddSwitch("A", "far_track", "main", "siding", 10f)
                .AddSwitch("B", "stem_b", "main", "siding", 10f);

            var finder = new LoopFinder(graph);

            // Loco on far_track, approaching A from the "wrong" side.
            // BFS arrives at A via far_track (enter), but then reaches B
            // via main. When it later tries TryFindLoop at A, arrival might
            // be from a branch leg if BFS reached A from that direction.
            // Simulate: loco on main between A and B
            var loop = finder.FindNearestFittingLoop(
                "main", distToA: 10f, distToB: 140f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNotNull(loop, "Should detect loop even when arriving via a branch leg");
            var switchIds = new HashSet<string> { loop.SwitchAId, loop.SwitchBId };
            Assert.That(switchIds, Contains.Item("A"));
            Assert.That(switchIds, Contains.Item("B"));
        }

        // --- Waypoint distance formula tests ---
        //
        // waypointDist = min(trainLength + foulingAtEntry, branchLength - foulingAtFar)
        //
        // Three scenarios from the design:
        // 1. Train on mainline, enters via branch switch → needs full trainLength + fouling
        // 2. Train on siding midway, goes left toward left switch → capped by branch length
        // 3. Train on siding midway, goes right toward right switch → capped by branch length

        /// <summary>
        /// Compute waypoint distance from a switch, same formula as FeasibilityChecker.
        /// </summary>
        private static float CalcWaypointDist(float trainLength, float foulingNear,
            float branchLength, float foulingFar)
        {
            float maxDist = branchLength - foulingFar;
            return System.Math.Min(trainLength + foulingNear, maxDist);
        }

        // --- Waypoint distance formula tests ---
        //
        // waypointDist = min(trainLength + foulingNear, branchLength - foulingFar)
        // Tried from BOTH switches; pick the one the loco can route to.

        [Test]
        public void WaypointDist_Scenario1_EnterViaBranchSwitch()
        {
            // Train (70m) on mainline, enters via left branch switch.
            // Waypoint from left switch: 70 + 10 = 80. Max = 300 - 10 = 290. → 80m.
            // Tail clears left switch: 80 - 70 = 10 >= 10 (fouling). ✓
            float dist = CalcWaypointDist(trainLength: 70f, foulingNear: 10f,
                branchLength: 300f, foulingFar: 10f);
            Assert.AreEqual(80f, dist, 0.01f);
            Assert.That(dist - 70f, Is.GreaterThanOrEqualTo(10f),
                "Tail must clear entry switch fouling");
        }

        [Test]
        public void WaypointDist_Scenario1_LongTrain_CappedByBranch()
        {
            // Train (250m), branch only 200m. Capped at branchLength - foulingFar.
            float dist = CalcWaypointDist(trainLength: 250f, foulingNear: 10f,
                branchLength: 200f, foulingFar: 10f);
            Assert.AreEqual(190f, dist, 0.01f,
                "Capped at branchLength - foulingFar");
        }

        [Test]
        public void WaypointDist_Scenario2_SidingDirect_EnoughRoom()
        {
            // Train (40m) on siding midway. Distance from siding to nearest
            // branch switch is plenty. Waypoint from that switch: 40 + 10 = 50.
            float dist = CalcWaypointDist(trainLength: 40f, foulingNear: 10f,
                branchLength: 300f, foulingFar: 10f);
            Assert.AreEqual(50f, dist, 0.01f);
        }

        [Test]
        public void WaypointDist_Scenario2_SidingReverse_NotEnoughRoom()
        {
            // Train (40m) on siding, but siding is close to switch A (only 30m away).
            // Not enough room from A (30 < 40 + 10 = 50).
            // Must use switch B instead. From B: 40 + 10 = 50. Max = 300 - 10 = 290.
            // Both candidates are computed; the one the loco can route to is used.
            float distFromA = CalcWaypointDist(trainLength: 40f, foulingNear: 10f,
                branchLength: 300f, foulingFar: 10f);
            float distFromB = CalcWaypointDist(trainLength: 40f, foulingNear: 10f,
                branchLength: 300f, foulingFar: 10f);
            // Both give 50m — the code tries both and picks the routable one.
            Assert.AreEqual(50f, distFromA, 0.01f);
            Assert.AreEqual(50f, distFromB, 0.01f);
        }

        [Test]
        public void WaypointDist_TailClearsNearSwitch()
        {
            float trainLength = 70f;
            float foulingNear = 15f;
            float dist = CalcWaypointDist(trainLength, foulingNear,
                branchLength: 300f, foulingFar: 10f);

            float tailFromNear = dist - trainLength;
            Assert.That(tailFromNear, Is.GreaterThanOrEqualTo(foulingNear),
                "Tail must clear the near switch fouling");
        }

        [Test]
        public void WaypointDist_LocoDoesNotOvershootFarSwitch()
        {
            float foulingFar = 15f;
            float branchLength = 200f;
            float dist = CalcWaypointDist(trainLength: 300f, foulingNear: 10f,
                branchLength: branchLength, foulingFar: foulingFar);

            Assert.That(dist, Is.LessThanOrEqualTo(branchLength - foulingFar),
                "Loco must not overshoot past the far switch fouling");
        }

        [Test]
        public void WaypointDist_BothSwitchesTried_SymmetricBranch()
        {
            // Symmetric branch: same fouling at both ends.
            // Both candidates give the same distance.
            float distFromA = CalcWaypointDist(trainLength: 50f, foulingNear: 10f,
                branchLength: 200f, foulingFar: 10f);
            float distFromB = CalcWaypointDist(trainLength: 50f, foulingNear: 10f,
                branchLength: 200f, foulingFar: 10f);
            Assert.AreEqual(distFromA, distFromB, 0.01f,
                "Symmetric branch should give same waypoint from either end");
        }

        [Test]
        public void WaypointDist_AsymmetricFouling()
        {
            // Switch A: fouling 5m. Switch B: fouling 20m.
            // From A: min(50 + 5, 200 - 20) = min(55, 180) = 55
            // From B: min(50 + 20, 200 - 5) = min(70, 195) = 70
            // Different distances — code picks the routable one.
            float distFromA = CalcWaypointDist(trainLength: 50f, foulingNear: 5f,
                branchLength: 200f, foulingFar: 20f);
            float distFromB = CalcWaypointDist(trainLength: 50f, foulingNear: 20f,
                branchLength: 200f, foulingFar: 5f);
            Assert.AreEqual(55f, distFromA, 0.01f);
            Assert.AreEqual(70f, distFromB, 0.01f);
        }

        // --- Mainline branch preference ---

        [Test]
        public void PreferMainlineBranch_ExitNormalPreferred()
        {
            // Loop with two branches: main (exitNormal) and siding (exitReverse).
            // Both fit the train. The mainline branch should be selected as
            // FittingBranch (tried first because it uses exitNormal).
            var graph = new MockGraphAdapter()
                .AddSegment("stem_a", "n_left", "A", 100f)
                .AddSegment("main",   "A", "B", 200f)
                .AddSegment("siding", "A", "B", 180f)
                .AddSegment("stem_b", "B", "n_right", 100f)
                .AddSwitch("A", "stem_a", "main", "siding", 10f)
                .AddSwitch("B", "stem_b", "main", "siding", 10f);

            var finder = new LoopFinder(graph);

            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNotNull(loop);
            // FittingBranch should be the mainline (uses exitNormal = "main")
            Assert.That(loop.FittingBranch.SegmentIds, Contains.Item("main"),
                "Should prefer mainline branch (exitNormal) over siding (exitReverse)");
        }

        [Test]
        public void PreferMainlineBranch_SidingUsedWhenMainlineDoesNotFit()
        {
            // Main branch is too short (30m usable). Siding is longer (170m usable).
            // Train (50m) doesn't fit on main but fits on siding.
            var graph = new MockGraphAdapter()
                .AddSegment("stem_a", "n_left", "A", 100f)
                .AddSegment("main",   "A", "B", 50f)   // 50 - 10 - 10 = 30 usable
                .AddSegment("siding", "A", "B", 190f)   // 190 - 10 - 10 = 170 usable
                .AddSegment("stem_b", "B", "n_right", 100f)
                .AddSwitch("A", "stem_a", "main", "siding", 10f)
                .AddSwitch("B", "stem_b", "main", "siding", 10f);

            var finder = new LoopFinder(graph);

            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNotNull(loop);
            // Main doesn't fit (30 < 50), so siding should be used
            Assert.That(loop.FittingBranch.SegmentIds, Contains.Item("siding"),
                "Should use siding when mainline doesn't fit");
        }

        // --- Gap-based waypoint placement ---

        [Test]
        public void FindBestGap_NoIntermediateSwitches()
        {
            // Simple branch: 300m, fouling 10 at each end. No intermediate switches.
            // One big gap: 10 to 290 = 280m.
            // Train 50m → waypoint at 10 + 50 = 60.
            var result = Autopilot.Services.TrackWalker.FindBestGapPosition(
                branchLength: 300f, foulingAtStart: 10f, foulingAtEnd: 10f,
                intermediateSwitchPositions: null, trainLength: 50f);

            Assert.IsNotNull(result);
            Assert.AreEqual(60f, result.Value, 0.01f);
        }

        [Test]
        public void FindBestGap_OneIntermediateSwitch_TrainFitsInFirstGap()
        {
            // Branch 300m, fouling 10 at ends.
            // Intermediate switch at 100m, fouling 10.
            // Gap 1: 10 to 90 = 80m ← first fitting gap, train (50m) fits
            // Gap 2: 110 to 290 = 180m
            // Picks gap 1 (closest to entry = shortest move).
            // Waypoint in gap 1: 10 + 50 = 60
            var intermediates = new List<(float, float)> { (100f, 10f) };
            var result = Autopilot.Services.TrackWalker.FindBestGapPosition(
                branchLength: 300f, foulingAtStart: 10f, foulingAtEnd: 10f,
                intermediateSwitchPositions: intermediates, trainLength: 50f);

            Assert.IsNotNull(result);
            Assert.AreEqual(60f, result.Value, 0.01f);
        }

        [Test]
        public void FindBestGap_FirstGapTooSmall_UsesSecond()
        {
            // Branch 300m, fouling 10 at ends.
            // Intermediate switch at 40m, fouling 10.
            // Gap 1: 10 to 30 = 20m ← too small for 50m train
            // Gap 2: 50 to 290 = 240m ← first fitting gap
            // Waypoint in gap 2: 50 + 50 = 100
            var intermediates = new List<(float, float)> { (40f, 10f) };
            var result = Autopilot.Services.TrackWalker.FindBestGapPosition(
                branchLength: 300f, foulingAtStart: 10f, foulingAtEnd: 10f,
                intermediateSwitchPositions: intermediates, trainLength: 50f);

            Assert.IsNotNull(result);
            Assert.AreEqual(100f, result.Value, 0.01f);
        }

        [Test]
        public void FindBestGap_TwoIntermediateSwitches_PicksFirstFitting()
        {
            // Branch 400m, fouling 10 at ends.
            // Switches at 100m and 250m, fouling 10 each.
            // Gap 1: 10 to 90 = 80m ← fits 50m train, closest to entry
            // Gap 2: 110 to 240 = 130m
            // Gap 3: 260 to 390 = 130m
            // Picks gap 1 (shortest move). Waypoint: 10 + 50 = 60
            var intermediates = new List<(float, float)> { (100f, 10f), (250f, 10f) };
            var result = Autopilot.Services.TrackWalker.FindBestGapPosition(
                branchLength: 400f, foulingAtStart: 10f, foulingAtEnd: 10f,
                intermediateSwitchPositions: intermediates, trainLength: 50f);

            Assert.IsNotNull(result);
            Assert.AreEqual(60f, result.Value, 0.01f);
        }

        [Test]
        public void FindBestGap_TrainDoesNotFitInAnyGap()
        {
            // Branch 100m, fouling 10 at ends.
            // Intermediate switch at 50m, fouling 10.
            // Gap 1: 10 to 40 = 30m
            // Gap 2: 60 to 90 = 30m
            // Train 50m doesn't fit either gap → null (fall back to simple formula)
            var intermediates = new List<(float, float)> { (50f, 10f) };
            var result = Autopilot.Services.TrackWalker.FindBestGapPosition(
                branchLength: 100f, foulingAtStart: 10f, foulingAtEnd: 10f,
                intermediateSwitchPositions: intermediates, trainLength: 50f);

            Assert.IsNull(result, "No gap fits the train — should return null");
        }

        [Test]
        public void FindBestGap_TrainExactlyFits()
        {
            // Gap is exactly trainLength. Should fit.
            // Branch 100m, fouling 10 at ends. No intermediates.
            // Gap: 10 to 90 = 80m. Train 80m.
            var result = Autopilot.Services.TrackWalker.FindBestGapPosition(
                branchLength: 100f, foulingAtStart: 10f, foulingAtEnd: 10f,
                intermediateSwitchPositions: null, trainLength: 80f);

            Assert.IsNotNull(result);
            Assert.AreEqual(90f, result.Value, 0.01f);
        }

        // --- Visited loop keys ---

        [Test]
        public void VisitedLoopKey_SkipsMatchingLoop()
        {
            var graph = BuildPassingSiding();
            var finder = new LoopFinder(graph);

            var visited = new HashSet<string> { "A|B" };

            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true,
                visitedLoopKeys: visited);

            Assert.IsNull(loop, "Loop A|B is visited — should skip it");
        }

        [Test]
        public void VisitedLoopKey_FindsDifferentLoop()
        {
            // Two loops: A-B (close) and C-D (far). Mark A-B as visited.
            // Should find C-D.
            var graph = new MockGraphAdapter()
                .AddSegment("stem",     "n_left", "A", 50f)
                .AddSegment("main1",    "A", "B", 100f)
                .AddSegment("siding1",  "A", "B", 90f)
                .AddSegment("main2",    "B", "C", 200f)
                .AddSegment("main3",    "C", "D", 100f)
                .AddSegment("siding2",  "C", "D", 90f)
                .AddSegment("stem2",    "D", "n_right", 50f)
                .AddSwitch("A", "stem",  "main1", "siding1", 10f)
                .AddSwitch("B", "main2", "main1", "siding1", 10f)
                .AddSwitch("C", "main2", "main3", "siding2", 10f)
                .AddSwitch("D", "stem2", "main3", "siding2", 10f);

            var finder = new LoopFinder(graph);

            var visited = new HashSet<string> { "A|B" };

            var loop = finder.FindNearestFittingLoop(
                "stem", distToA: 40f, distToB: 10f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true,
                visitedLoopKeys: visited);

            Assert.IsNotNull(loop, "Should find C-D loop");
            var switchIds = new HashSet<string> { loop.SwitchAId, loop.SwitchBId };
            Assert.That(switchIds, Contains.Item("C"));
            Assert.That(switchIds, Contains.Item("D"));
        }

        [Test]
        public void LoopKey_SortedAlphabetically()
        {
            var graph = BuildPassingSiding();
            var finder = new LoopFinder(graph);

            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNotNull(loop);
            Assert.AreEqual("A|B", loop.LoopKey, "Loop key should be sorted alphabetically");
        }

        // --- Prefer all branches fit ---

        [Test]
        public void PreferAllBranchesFit()
        {
            // Loop1 (A-B): main=100, siding=40. Train=50.
            //   main fits (100-10=90>=50), siding doesn't (40-10=30<50). 1 of 2.
            // Loop2 (C-D): main=100, siding=100. Train=50.
            //   Both fit. 2 of 2.
            // Loop2 is further but all branches fit → preferred.
            var graph = new MockGraphAdapter()
                .AddSegment("stem",     "n_left", "A", 50f)
                .AddSegment("main1",    "A", "B", 100f)
                .AddSegment("siding1",  "A", "B", 40f)
                .AddSegment("link",     "B", "C", 50f)
                .AddSegment("main2",    "C", "D", 100f)
                .AddSegment("siding2",  "C", "D", 100f)
                .AddSegment("stem2",    "D", "n_right", 50f)
                .AddSwitch("A", "stem",  "main1", "siding1", 10f)
                .AddSwitch("B", "link",  "main1", "siding1", 10f)
                .AddSwitch("C", "link",  "main2", "siding2", 10f)
                .AddSwitch("D", "stem2", "main2", "siding2", 10f);

            var finder = new LoopFinder(graph);

            var loop = finder.FindNearestFittingLoop(
                "stem", distToA: 40f, distToB: 10f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true);

            Assert.IsNotNull(loop);
            var switchIds = new HashSet<string> { loop.SwitchAId, loop.SwitchBId };
            Assert.That(switchIds, Contains.Item("C"), "Should prefer C-D (all branches fit)");
            Assert.That(switchIds, Contains.Item("D"));
        }

        // --- Blocked branch segments ---

        [Test]
        public void BlockedBranch_SkippedForClearBranch()
        {
            // Siding is blocked by a parked car. Main is clear. Should use main.
            var graph = BuildPassingSiding();
            var finder = new LoopFinder(graph);

            var blocked = new HashSet<string> { "siding" };

            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true,
                isSegmentBlocked: segId => blocked.Contains(segId));

            Assert.IsNotNull(loop, "Should still find loop using the clear branch");
            Assert.That(loop.FittingBranch.SegmentIds, Contains.Item("main"),
                "Should use main (clear), not siding (blocked)");
        }

        [Test]
        public void AllBranchesBlocked_NoLoop()
        {
            // Both branches blocked. No usable loop.
            var graph = BuildPassingSiding();
            var finder = new LoopFinder(graph);

            var blocked = new HashSet<string> { "main", "siding" };

            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true,
                isSegmentBlocked: segId => blocked.Contains(segId));

            Assert.IsNull(loop, "Both branches blocked — no usable loop");
        }

        [Test]
        public void BlockedBranch_SkipsToNextLoop()
        {
            // Two loops: A-B (close, both branches blocked) and C-D (far, clear).
            // Should skip A-B and find C-D.
            var graph = new MockGraphAdapter()
                .AddSegment("stem",     "n_left", "A", 50f)
                .AddSegment("main1",    "A", "B", 100f)
                .AddSegment("siding1",  "A", "B", 90f)
                .AddSegment("main2",    "B", "C", 200f)
                .AddSegment("main3",    "C", "D", 100f)
                .AddSegment("siding2",  "C", "D", 90f)
                .AddSegment("stem2",    "D", "n_right", 50f)
                .AddSwitch("A", "stem",  "main1", "siding1", 10f)
                .AddSwitch("B", "main2", "main1", "siding1", 10f)
                .AddSwitch("C", "main2", "main3", "siding2", 10f)
                .AddSwitch("D", "stem2", "main3", "siding2", 10f);

            var finder = new LoopFinder(graph);

            // Block both branches of loop A-B
            var blocked = new HashSet<string> { "main1", "siding1" };

            var loop = finder.FindNearestFittingLoop(
                "stem", distToA: 40f, distToB: 10f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true,
                isSegmentBlocked: segId => blocked.Contains(segId));

            Assert.IsNotNull(loop, "Should find C-D loop (A-B is blocked)");
            var switchIds = new HashSet<string> { loop.SwitchAId, loop.SwitchBId };
            Assert.That(switchIds, Contains.Item("C"));
            Assert.That(switchIds, Contains.Item("D"));
        }

        [Test]
        public void BlockedBranch_MultiSegmentBranch_AnySegmentBlocks()
        {
            // Branch with intermediate switch: sid1 -- [M] -- sid2.
            // Blocking only sid2 should block the entire branch.
            var graph = new MockGraphAdapter()
                .AddSegment("stem",  "n_left", "A", 100f)
                .AddSegment("main",  "A", "B", 200f)
                .AddSegment("sid1",  "A", "M", 80f)
                .AddSegment("sid2",  "M", "B", 80f)
                .AddSegment("spur",  "M", "n_spur", 50f)
                .AddSegment("stem2", "B", "n_right", 100f)
                .AddSwitch("A", "stem", "main", "sid1", 10f)
                .AddSwitch("B", "stem2", "main", "sid2", 10f)
                .AddSwitch("M", "sid1", "sid2", "spur", 5f);

            var finder = new LoopFinder(graph);

            // Block only sid2 — the second segment of the siding branch
            var blocked = new HashSet<string> { "sid2" };

            var loop = finder.FindNearestFittingLoop(
                "stem", distToA: 50f, distToB: 50f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true,
                isSegmentBlocked: segId => blocked.Contains(segId));

            Assert.IsNotNull(loop, "Should still find the loop (main branch is clear)");
            Assert.That(loop.FittingBranch.SegmentIds, Contains.Item("main"),
                "Should use main branch — siding is blocked at sid2");
        }

        // --- Reversal clearance at switches ---

        [Test]
        public void ReversalClear_LoopFound()
        {
            // Standard passing siding. Stems are 100m, fouling 10m, loco 20m.
            // Required clearance = 10 + 20 = 30m. Stems are 100m. Plenty of room.
            var graph = BuildPassingSiding(stemALength: 100f, stemBLength: 100f, fouling: 10f);
            var finder = new LoopFinder(graph);

            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true,
                hasReversalClearance: (switchId, stemSegId, required) =>
                    graph.GetLength(stemSegId) >= required,
                locoLength: 20f);

            Assert.IsNotNull(loop, "Stems have plenty of room for reversal");
        }

        [Test]
        public void ReversalClearance_UsesStemFouling_NotExitFouling()
        {
            // Exit fouling=40m, stem fouling=2m (constant), trainLength=50m.
            // Required on stem = 2 + 50 = 52m, NOT 40 + 50 = 90m.
            // Stem is 55m. If exit fouling were used: 55 < 90 → rejected (wrong).
            // With stem fouling: 55 >= 52 → accepted (correct).
            var graph = BuildPassingSiding(stemALength: 55f, stemBLength: 55f, fouling: 40f);
            var finder = new LoopFinder(graph);

            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 27f, distToB: 28f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true,
                hasReversalClearance: (switchId, stemSegId, required) =>
                    graph.GetLength(stemSegId) >= required,
                locoLength: 20f);

            Assert.IsNotNull(loop,
                "Stem (55m) should have enough room: required = 2m stem fouling + 50m train = 52m");
        }

        [Test]
        public void ReversalBlocked_StemTooShort_LoopSkipped()
        {
            // Very short stems (15m). Stem fouling=2, loco=20. Required=22. 15 < 22 → blocked.
            var graph = BuildPassingSiding(stemALength: 15f, stemBLength: 15f, fouling: 10f);
            var finder = new LoopFinder(graph);

            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 7f, distToB: 8f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true,
                hasReversalClearance: (switchId, stemSegId, required) =>
                    graph.GetLength(stemSegId) >= required,
                locoLength: 20f);

            Assert.IsNull(loop, "Stems too short for loco to reverse — loop unusable");
        }

        [Test]
        public void ReversalBlocked_OneStemOnly_EntireLoopSkipped()
        {
            // Stem A is 100m (OK). Stem B is 15m (too short).
            // Both switches need reversal clearance. One failure → skip loop.
            var graph = BuildPassingSiding(stemALength: 100f, stemBLength: 15f, fouling: 10f);
            var finder = new LoopFinder(graph);

            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true,
                hasReversalClearance: (switchId, stemSegId, required) =>
                    graph.GetLength(stemSegId) >= required,
                locoLength: 20f);

            Assert.IsNull(loop, "One stem too short — entire loop is unusable");
        }

        [Test]
        public void ReversalBlocked_CarOnStem_LoopSkipped()
        {
            // Stems are 100m (geometrically fine), but a car blocks stem_b.
            var graph = BuildPassingSiding(stemALength: 100f, stemBLength: 100f, fouling: 10f);
            var finder = new LoopFinder(graph);

            var blockedStems = new HashSet<string> { "stem_b" };

            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true,
                hasReversalClearance: (switchId, stemSegId, required) =>
                {
                    if (blockedStems.Contains(stemSegId)) return false;
                    return graph.GetLength(stemSegId) >= required;
                },
                locoLength: 20f);

            Assert.IsNull(loop, "Car blocking stem_b — reversal impossible");
        }

        [Test]
        public void ReversalBlocked_SkipsToNextLoop()
        {
            // Two loops: A-B (close, stem too short) and C-D (far, stems OK).
            var graph = new MockGraphAdapter()
                .AddSegment("stem",     "n_left", "A", 15f)  // too short for reversal
                .AddSegment("main1",    "A", "B", 100f)
                .AddSegment("siding1",  "A", "B", 90f)
                .AddSegment("link",     "B", "C", 15f)       // also too short (stem of B)
                .AddSegment("main2",    "C", "D", 100f)
                .AddSegment("siding2",  "C", "D", 90f)
                .AddSegment("stem2",    "D", "n_right", 100f)
                .AddSwitch("A", "stem",  "main1", "siding1", 10f)
                .AddSwitch("B", "link",  "main1", "siding1", 10f)
                .AddSwitch("C", "link",  "main2", "siding2", 10f)
                .AddSwitch("D", "stem2", "main2", "siding2", 10f);

            var finder = new LoopFinder(graph);

            var loop = finder.FindNearestFittingLoop(
                "stem", distToA: 7f, distToB: 8f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true,
                hasReversalClearance: (switchId, stemSegId, required) =>
                    graph.GetLength(stemSegId) >= required,
                locoLength: 20f);

            // A-B: stem_a=15, link(stemB)=15. Both < 30 required → skip.
            // C-D: link(stemC)=15 < 30 → skip too.
            // Neither loop works.
            Assert.IsNull(loop, "Both loops have short stems — no usable loop");
        }

        // --- Combined: blocked branches + reversal clearance ---

        [Test]
        public void BlockedBranch_WithClearReversal_UsesOtherBranch()
        {
            // Siding blocked, reversal OK. Should use main branch.
            var graph = BuildPassingSiding(stemALength: 100f, stemBLength: 100f, fouling: 10f);
            var finder = new LoopFinder(graph);

            var blocked = new HashSet<string> { "siding" };

            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 50f, distToB: 50f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true,
                isSegmentBlocked: segId => blocked.Contains(segId),
                hasReversalClearance: (switchId, stemSegId, required) =>
                    graph.GetLength(stemSegId) >= required,
                locoLength: 20f);

            Assert.IsNotNull(loop, "Main branch is clear and reversal space is OK");
            Assert.That(loop.FittingBranch.SegmentIds, Contains.Item("main"));
        }

        [Test]
        public void ClearBranches_WithBlockedReversal_NoLoop()
        {
            // Branches are clear, but reversal space is blocked → loop is unusable.
            var graph = BuildPassingSiding(stemALength: 15f, stemBLength: 15f, fouling: 10f);
            var finder = new LoopFinder(graph);

            var loop = finder.FindNearestFittingLoop(
                "stem_a", distToA: 7f, distToB: 8f,
                trainLength: 50f, visitedSwitches: null,
                canReachSwitch: (segId, endA) => true,
                isSegmentBlocked: segId => false, // nothing blocked
                hasReversalClearance: (switchId, stemSegId, required) =>
                    graph.GetLength(stemSegId) >= required,
                locoLength: 20f);

            Assert.IsNull(loop, "Branches clear but stems too short — no usable loop");
        }
    }
}
