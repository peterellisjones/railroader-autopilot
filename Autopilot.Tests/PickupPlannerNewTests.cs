using System.Collections.Generic;
using System.Linq;
using NUnit.Framework;
using Model;
using Model.Ops;
using Track;
using Autopilot.Model;
using Autopilot.Planning;

namespace Autopilot.Tests
{
    [TestFixture]
    public class PickupPlannerNewTests
    {
        // ── Helpers ──────────────────────────────────────────────────────────────

        private static OpsCarPosition TestOpsCarPosition(string displayName, string identifier)
        {
            return new OpsCarPosition(displayName, identifier, System.Array.Empty<TrackSpan>());
        }

        private static MockCar WaybilledCar(string id, string destName, string destId)
        {
            return new MockCar
            {
                id = id,
                DisplayName = id,
                Waybill = new Waybill(default, null, TestOpsCarPosition(destName, destId), 0, false, null, 0)
            };
        }

        // =====================================================================
        // FilterAccessibleTargets — additional edge cases
        // =====================================================================

        // 3.1: Single accessible car
        [Test]
        public void FilterAccessible_single_target()
        {
            var car = new MockCar { id = "c1", DisplayName = "Boxcar" };
            var chain = new List<ICar> { car };
            var targets = new HashSet<string> { "c1" };
            var result = PickupPlanner.FilterAccessibleTargets(chain, targets);
            Assert.That(result.Count, Is.EqualTo(1));
            Assert.That(result[0].id, Is.EqualTo("c1"));
        }

        // 3.2: Car behind non-target is blocked
        [Test]
        public void FilterAccessible_blocked_by_nontarget()
        {
            var c1 = new MockCar { id = "c1", DisplayName = "Blocker" };
            var c2 = new MockCar { id = "c2", DisplayName = "Target" };
            MockCar.Couple(c1, c2);
            var chain = new List<ICar> { c1, c2 };
            var targets = new HashSet<string> { "c2" };
            var result = PickupPlanner.FilterAccessibleTargets(chain, targets);
            Assert.That(result.Count, Is.EqualTo(0));
        }

        // 3.3: Route check via ScenarioBuilder — unreachable segment
        [Test]
        public void Unreachable_segment_no_route()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "n1", 500)
                .AddSegment("island", "n2", "n3", 200)
                .PlaceLoco("loco", "lead", 250)
                .Build();

            var from = new GraphPosition("lead", 250, Direction.TowardEndB);
            var to = new GraphPosition("island", 100, Direction.TowardEndA);
            Assert.That(scenario.GraphAdapter.FindRoute(from, to), Is.Null);
        }

        // 3.4: MatchesFilter — no waybill returns false
        [Test]
        public void MatchesFilter_no_waybill_returns_false()
        {
            var car = new MockCar { id = "c1", DisplayName = "Empty" };
            var filter = new PickupFilter(
                new FilterAxis(FilterMode.Destination, new HashSet<string> { "Somewhere" }),
                FilterAxis.Any, float.MaxValue, false);
            Assert.That(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()), Is.False);
        }

        // 3.5: Multiple consecutive targets all accessible
        [Test]
        public void FilterAccessible_consecutive_targets()
        {
            var c1 = new MockCar { id = "c1", DisplayName = "T1" };
            var c2 = new MockCar { id = "c2", DisplayName = "T2" };
            var c3 = new MockCar { id = "c3", DisplayName = "NonTarget" };
            MockCar.Couple(c1, c2);
            MockCar.Couple(c2, c3);
            var chain = new List<ICar> { c1, c2, c3 };
            var targets = new HashSet<string> { "c1", "c2" };
            var result = PickupPlanner.FilterAccessibleTargets(chain, targets);
            Assert.That(result.Count, Is.EqualTo(2));
            Assert.That(result[0].id, Is.EqualTo("c1"));
            Assert.That(result[1].id, Is.EqualTo("c2"));
        }

        // 3.6: Chain walking from end car
        [Test]
        public void GetCoupledChain_from_end()
        {
            var c1 = new MockCar { id = "c1", DisplayName = "C1" };
            var c2 = new MockCar { id = "c2", DisplayName = "C2" };
            var c3 = new MockCar { id = "c3", DisplayName = "C3" };
            MockCar.Couple(c1, c2);
            MockCar.Couple(c2, c3);
            var chain = PickupPlanner.GetCoupledChain(c3);
            Assert.That(chain.Count, Is.EqualTo(3));
            // Should walk to the A-free end first
            Assert.That(chain[0].id, Is.EqualTo("c1"));
            Assert.That(chain[2].id, Is.EqualTo("c3"));
        }

        // 3.7: Single car chain
        [Test]
        public void GetCoupledChain_single_car()
        {
            var car = new MockCar { id = "c1" };
            var chain = PickupPlanner.GetCoupledChain(car);
            Assert.That(chain.Count, Is.EqualTo(1));
            Assert.That(chain[0].id, Is.EqualTo("c1"));
        }

        // 3.8: Gap in targets stops at gap
        [Test]
        public void FilterAccessible_gap_stops_collection()
        {
            var c1 = new MockCar { id = "c1", DisplayName = "T1" };
            var c2 = new MockCar { id = "c2", DisplayName = "Blocker" };
            var c3 = new MockCar { id = "c3", DisplayName = "T2" };
            MockCar.Couple(c1, c2);
            MockCar.Couple(c2, c3);
            var chain = new List<ICar> { c1, c2, c3 };
            var targets = new HashSet<string> { "c1", "c3" };
            var result = PickupPlanner.FilterAccessibleTargets(chain, targets);
            Assert.That(result.Count, Is.EqualTo(1)); // only c1, c2 blocks c3
            Assert.That(result[0].id, Is.EqualTo("c1"));
        }

        // 3.9: All targets
        [Test]
        public void FilterAccessible_all_targets()
        {
            var c1 = new MockCar { id = "c1" };
            var c2 = new MockCar { id = "c2" };
            MockCar.Couple(c1, c2);
            var chain = new List<ICar> { c1, c2 };
            var targets = new HashSet<string> { "c1", "c2" };
            var result = PickupPlanner.FilterAccessibleTargets(chain, targets);
            Assert.That(result.Count, Is.EqualTo(2));
        }

        // =====================================================================
        // MatchesFilter — comprehensive mode combinations
        // =====================================================================

        // 3.10: Switchlist filter — car on switchlist
        [Test]
        public void MatchesFilter_switchlist_from_matches_when_on_list()
        {
            var car = WaybilledCar("c1", "Mill S1", "mill-s1");
            var filter = new PickupFilter(
                new FilterAxis(FilterMode.Switchlist, new HashSet<string>()),
                FilterAxis.Any, float.MaxValue, false);
            var switchlist = new HashSet<string> { "c1" };
            var result = PickupPlanner.MatchesFilter(car, filter, switchlist);
            Assert.That(result, Is.True);
        }

        // 3.11: Switchlist filter — car NOT on switchlist
        [Test]
        public void MatchesFilter_switchlist_from_no_match_when_not_on_list()
        {
            var car = WaybilledCar("c1", "Mill S1", "mill-s1");
            var filter = new PickupFilter(
                new FilterAxis(FilterMode.Switchlist, new HashSet<string>()),
                FilterAxis.Any, float.MaxValue, false);
            var switchlist = new HashSet<string>(); // c1 not on switchlist
            var result = PickupPlanner.MatchesFilter(car, filter, switchlist);
            Assert.That(result, Is.False);
        }

        // 3.12: To:Destination filter — matching destination
        [Test]
        public void MatchesFilter_to_destination_matches()
        {
            var car = WaybilledCar("c1", "Mill S1", "mill-s1");
            var filter = new PickupFilter(
                FilterAxis.Any,
                new FilterAxis(FilterMode.Destination, new HashSet<string> { "Mill S1" }),
                float.MaxValue, false);
            Assert.That(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()), Is.True);
        }

        // 3.13: To:Destination filter — non-matching destination
        [Test]
        public void MatchesFilter_to_destination_no_match()
        {
            var car = WaybilledCar("c1", "Mill S1", "mill-s1");
            var filter = new PickupFilter(
                FilterAxis.Any,
                new FilterAxis(FilterMode.Destination, new HashSet<string> { "Lumber S2" }),
                float.MaxValue, false);
            Assert.That(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()), Is.False);
        }

        // 3.14: Any/Any filter matches any waybilled car
        [Test]
        public void MatchesFilter_any_any_matches_waybilled()
        {
            var car = WaybilledCar("c1", "Siding X", "sx");
            var filter = PickupFilter.Default;
            Assert.That(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()), Is.True);
        }

        // 3.15: Completed waybill — car displaced from destination still matches
        [Test]
        public void MatchesFilter_completed_displaced_matches()
        {
            // Completed waybill but car EndA/EndB segments are null (no Segment set on MockCar),
            // so WaybillHelper.IsAtDestination returns false (displaced), meaning IsPendingDelivery = true.
            var car = new MockCar
            {
                id = "c1",
                DisplayName = "Displaced",
                Waybill = new Waybill(default, null, TestOpsCarPosition("Mill S1", "mill-s1"), 0, true, null, 0)
            };
            var filter = PickupFilter.Default;
            Assert.That(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()), Is.True);
        }

        // 3.16: To:Switchlist mode — car on switchlist
        [Test]
        public void MatchesFilter_to_switchlist_matches()
        {
            var car = WaybilledCar("c1", "Mill S1", "mill-s1");
            var filter = new PickupFilter(
                FilterAxis.Any,
                new FilterAxis(FilterMode.Switchlist, new HashSet<string>()),
                float.MaxValue, false);
            Assert.That(PickupPlanner.MatchesFilter(car, filter, new HashSet<string> { "c1" }), Is.True);
        }

        // 3.17: To:Switchlist mode — car NOT on switchlist
        [Test]
        public void MatchesFilter_to_switchlist_no_match()
        {
            var car = WaybilledCar("c1", "Mill S1", "mill-s1");
            var filter = new PickupFilter(
                FilterAxis.Any,
                new FilterAxis(FilterMode.Switchlist, new HashSet<string>()),
                float.MaxValue, false);
            Assert.That(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()), Is.False);
        }

        // 3.18: Both axes Switchlist — car on switchlist
        [Test]
        public void MatchesFilter_both_switchlist_on_list()
        {
            var car = WaybilledCar("c1", "Mill S1", "mill-s1");
            var filter = new PickupFilter(
                new FilterAxis(FilterMode.Switchlist, new HashSet<string>()),
                new FilterAxis(FilterMode.Switchlist, new HashSet<string>()),
                float.MaxValue, false);
            Assert.That(PickupPlanner.MatchesFilter(car, filter, new HashSet<string> { "c1" }), Is.True);
        }

        // 3.19: Both axes Switchlist — car NOT on switchlist
        [Test]
        public void MatchesFilter_both_switchlist_not_on_list()
        {
            var car = WaybilledCar("c1", "Mill S1", "mill-s1");
            var filter = new PickupFilter(
                new FilterAxis(FilterMode.Switchlist, new HashSet<string>()),
                new FilterAxis(FilterMode.Switchlist, new HashSet<string>()),
                float.MaxValue, false);
            Assert.That(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()), Is.False);
        }

        // 3.20: To:Destination with multiple allowed destinations — one matches
        [Test]
        public void MatchesFilter_to_destination_multiple_choices()
        {
            var car = WaybilledCar("c1", "Mill S1", "mill-s1");
            var filter = new PickupFilter(
                FilterAxis.Any,
                new FilterAxis(FilterMode.Destination, new HashSet<string> { "Lumber S2", "Mill S1", "Yard S3" }),
                float.MaxValue, false);
            Assert.That(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()), Is.True);
        }

        // 3.21: From:Switchlist + To:Destination — both must pass
        [Test]
        public void MatchesFilter_switchlist_from_and_destination_to()
        {
            var car = WaybilledCar("c1", "Mill S1", "mill-s1");
            var filter = new PickupFilter(
                new FilterAxis(FilterMode.Switchlist, new HashSet<string>()),
                new FilterAxis(FilterMode.Destination, new HashSet<string> { "Mill S1" }),
                float.MaxValue, false);
            // Car on switchlist AND destination matches
            Assert.That(PickupPlanner.MatchesFilter(car, filter, new HashSet<string> { "c1" }), Is.True);
        }

        // 3.22: From:Switchlist + To:Destination — switchlist fails
        [Test]
        public void MatchesFilter_switchlist_from_fails_destination_to_passes()
        {
            var car = WaybilledCar("c1", "Mill S1", "mill-s1");
            var filter = new PickupFilter(
                new FilterAxis(FilterMode.Switchlist, new HashSet<string>()),
                new FilterAxis(FilterMode.Destination, new HashSet<string> { "Mill S1" }),
                float.MaxValue, false);
            // NOT on switchlist
            Assert.That(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()), Is.False);
        }

        // =====================================================================
        // GetCoupledChain — additional edge cases
        // =====================================================================

        // 3.23: Long chain preserves order
        [Test]
        public void GetCoupledChain_five_cars_preserves_order()
        {
            var cars = Enumerable.Range(1, 5)
                .Select(i => new MockCar { id = $"c{i}", DisplayName = $"C{i}" })
                .ToArray();
            for (int i = 0; i < cars.Length - 1; i++)
                MockCar.Couple(cars[i], cars[i + 1]);

            // Start from middle car
            var chain = PickupPlanner.GetCoupledChain(cars[2]);
            Assert.That(chain.Count, Is.EqualTo(5));
            Assert.That(chain[0].id, Is.EqualTo("c1"));
            Assert.That(chain[4].id, Is.EqualTo("c5"));
        }

        // 3.24: Chain from first car
        [Test]
        public void GetCoupledChain_from_first_car()
        {
            var c1 = new MockCar { id = "c1" };
            var c2 = new MockCar { id = "c2" };
            var c3 = new MockCar { id = "c3" };
            MockCar.Couple(c1, c2);
            MockCar.Couple(c2, c3);
            var chain = PickupPlanner.GetCoupledChain(c1);
            Assert.That(chain.Count, Is.EqualTo(3));
            Assert.That(chain[0].id, Is.EqualTo("c1"));
            Assert.That(chain[1].id, Is.EqualTo("c2"));
            Assert.That(chain[2].id, Is.EqualTo("c3"));
        }

        // 3.25: Two disjoint chains don't cross-contaminate
        [Test]
        public void GetCoupledChain_disjoint_chains()
        {
            var c1 = new MockCar { id = "c1" };
            var c2 = new MockCar { id = "c2" };
            MockCar.Couple(c1, c2);

            var c3 = new MockCar { id = "c3" };
            var c4 = new MockCar { id = "c4" };
            MockCar.Couple(c3, c4);

            var chain1 = PickupPlanner.GetCoupledChain(c1);
            var chain2 = PickupPlanner.GetCoupledChain(c3);

            Assert.That(chain1.Count, Is.EqualTo(2));
            Assert.That(chain2.Count, Is.EqualTo(2));
            Assert.That(chain1.Select(c => c.id), Does.Not.Contain("c3"));
            Assert.That(chain2.Select(c => c.id), Does.Not.Contain("c1"));
        }

        // =====================================================================
        // FilterAccessibleTargets — more nuanced scenarios
        // =====================================================================

        // 3.26: Empty targets set returns empty even with full chain
        [Test]
        public void FilterAccessible_empty_targets_returns_empty()
        {
            var c1 = new MockCar { id = "c1" };
            var c2 = new MockCar { id = "c2" };
            MockCar.Couple(c1, c2);
            var chain = new List<ICar> { c1, c2 };
            var targets = new HashSet<string>();
            var result = PickupPlanner.FilterAccessibleTargets(chain, targets);
            Assert.That(result.Count, Is.EqualTo(0));
        }

        // 3.27: Target IDs not in chain returns empty
        [Test]
        public void FilterAccessible_targets_not_in_chain_returns_empty()
        {
            var c1 = new MockCar { id = "c1" };
            var chain = new List<ICar> { c1 };
            var targets = new HashSet<string> { "c99" };
            var result = PickupPlanner.FilterAccessibleTargets(chain, targets);
            Assert.That(result.Count, Is.EqualTo(0));
        }

        // 3.28: Five-car chain, targets on both ends, only approach end accessible
        [Test]
        public void FilterAccessible_five_cars_mixed()
        {
            var cars = Enumerable.Range(1, 5)
                .Select(i => new MockCar { id = $"c{i}", DisplayName = $"C{i}" })
                .ToArray();
            for (int i = 0; i < cars.Length - 1; i++)
                MockCar.Couple(cars[i], cars[i + 1]);

            // Targets at positions 0, 1, and 4 (but 2 and 3 block 4)
            var chain = new List<ICar>(cars);
            var targets = new HashSet<string> { "c1", "c2", "c5" };
            var result = PickupPlanner.FilterAccessibleTargets(chain, targets);
            Assert.That(result.Count, Is.EqualTo(2));
            Assert.That(result[0].id, Is.EqualTo("c1"));
            Assert.That(result[1].id, Is.EqualTo("c2"));
        }

        // =====================================================================
        // ScenarioBuilder-based tests — SplitFinder abstract overload
        // =====================================================================

        // 3.29: SplitFinder on two-group consist finds a split
        [Test]
        public void SplitFinder_two_groups_finds_split()
        {
            // Topology: lead(n0--sw1) -- siding1(sw1--n1)
            //                         \- siding2(sw1--n2)
            // Consist: loco -- carB(siding2) -- carA(siding1)
            // carA is tail, going to siding1. carB is behind, going to siding2.
            // SplitFinder should find the split between carA and carB.
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding1", "sw1", "n1", 200)
                .AddSegment("siding2", "sw1", "n2", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding2")
                .PlaceCar("carA", "lead", 400, Direction.TowardEndB, destination: "siding1")
                .PlaceCar("carB", "lead", 380, Direction.TowardEndB, destination: "siding2",
                    coupledTo: "carA")
                .PlaceLoco("loco", "lead", 300, Direction.TowardEndB, coupledTo: "carB")
                .DefineDestination("siding1", "Siding 1", 200f,
                    new GraphPosition("siding1", 0, Direction.TowardEndB))
                .DefineDestination("siding2", "Siding 2", 200f,
                    new GraphPosition("siding2", 0, Direction.TowardEndB))
                .Build();

            var checker = new FeasibilityChecker(scenario.TrainService, scenario.GraphAdapter);
            var finder = new SplitFinder(scenario.TrainService, scenario.GraphAdapter, checker);

            // Build a CarGroup from the coupled cars (tail to loco)
            // The coupled list from scenario is ordered A-side -> loco -> B-side
            // We need just the B-side (cars on loco's B side) ordered tail-to-loco
            var coupled = scenario.TrainService.GetCoupled();
            var locoIdx = coupled.ToList().FindIndex(c => c.id == "loco");
            var bSide = coupled.Skip(locoIdx + 1).ToList();
            // bSide is [carB, carA] — from loco outward
            // CarGroup needs tail-to-loco: [carA, carB]
            bSide.Reverse();

            var group = new CarGroup(bSide);
            var split = finder.FindBestSplit(group);

            // The split should be found since the two cars go to different destinations
            // and both destinations are reachable
            Assert.That(split, Is.Not.Null, "Should find a split for two-group consist");
            Assert.That(split!.DroppedCars.Count, Is.GreaterThan(0));
        }

        // 3.30: SplitFinder — single destination group returns null
        [Test]
        public void SplitFinder_single_dest_group_returns_null()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding1", "sw1", "n1", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding1")
                .PlaceCar("carA", "lead", 400, Direction.TowardEndB, destination: "siding1")
                .PlaceCar("carB", "lead", 380, Direction.TowardEndB, destination: "siding1",
                    coupledTo: "carA")
                .PlaceLoco("loco", "lead", 300, Direction.TowardEndB, coupledTo: "carB")
                .DefineDestination("siding1", "Siding 1", 200f,
                    new GraphPosition("siding1", 0, Direction.TowardEndB))
                .Build();

            var checker = new FeasibilityChecker(scenario.TrainService, scenario.GraphAdapter);
            var finder = new SplitFinder(scenario.TrainService, scenario.GraphAdapter, checker);

            var coupled = scenario.TrainService.GetCoupled();
            var locoIdx = coupled.ToList().FindIndex(c => c.id == "loco");
            var bSide = coupled.Skip(locoIdx + 1).ToList();
            bSide.Reverse();

            var group = new CarGroup(bSide);
            var split = finder.FindBestSplit(group);

            Assert.That(split, Is.Null, "Single destination group should not need a split");
        }

        // 3.31: SplitFinder — empty group returns null
        [Test]
        public void SplitFinder_empty_group_returns_null()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "n1", 500)
                .PlaceLoco("loco", "lead", 300)
                .Build();

            var checker = new FeasibilityChecker(scenario.TrainService, scenario.GraphAdapter);
            var finder = new SplitFinder(scenario.TrainService, scenario.GraphAdapter, checker);

            var split = finder.FindBestSplit(CarGroup.Empty);
            Assert.That(split, Is.Null);
        }

        // =====================================================================
        // RunaroundBuilder abstract overload
        // =====================================================================

        // 3.32: RunaroundBuilder on a simple consist
        [Test]
        public void RunaroundBuilder_builds_action_for_simple_consist()
        {
            // Topology: siding1(n1--sw1) -- lead(sw1--sw2) -- siding2(sw2--n2)
            //                                               \- siding3(sw2--n3)
            // If approach direction requires runaround, the builder should produce an action.
            var scenario = new ScenarioBuilder()
                .AddSegment("siding1", "n1", "sw1", 200)
                .AddSegment("lead", "sw1", "sw2", 500)
                .AddSegment("siding2", "sw2", "n2", 200)
                .AddSegment("siding3", "sw2", "n3", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding1")
                .AddSwitch("sw2", "lead", "siding2", "siding3")
                .PlaceCar("carA", "lead", 400, Direction.TowardEndB, destination: "siding2")
                .PlaceCar("carB", "lead", 380, Direction.TowardEndB, destination: "siding3",
                    coupledTo: "carA")
                .PlaceLoco("loco", "lead", 300, Direction.TowardEndB, coupledTo: "carB")
                .DefineDestination("siding2", "Siding 2", 200f,
                    new GraphPosition("siding2", 0, Direction.TowardEndB))
                .DefineDestination("siding3", "Siding 3", 200f,
                    new GraphPosition("siding3", 0, Direction.TowardEndB))
                .Build();

            var checker = new FeasibilityChecker(scenario.TrainService, scenario.GraphAdapter);
            var analyzer = new DeliverabilityAnalyzer(scenario.TrainService);
            var builder = new RunaroundBuilder(scenario.TrainService, scenario.GraphAdapter, analyzer);

            var coupled = scenario.TrainService.GetCoupled();
            var locoIdx = coupled.ToList().FindIndex(c => c.id == "loco");
            var bSide = coupled.Skip(locoIdx + 1).ToList();
            bSide.Reverse();

            var group = new CarGroup(bSide);
            var action = builder.BuildRunaroundAction(group, checker);

            // The builder should produce an action (even if the split point is trivial)
            Assert.That(action, Is.Not.Null, "Should produce a runaround action");
            Assert.That(action!.CoupleTarget, Is.Not.Null);
            Assert.That(action.DisconnectedCars.Count, Is.GreaterThan(0));
        }

        // 3.33: RunaroundBuilder — empty group returns null
        [Test]
        public void RunaroundBuilder_empty_group_returns_null()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "n1", 500)
                .PlaceLoco("loco", "lead", 300)
                .Build();

            var checker = new FeasibilityChecker(scenario.TrainService, scenario.GraphAdapter);
            var analyzer = new DeliverabilityAnalyzer(scenario.TrainService);
            var builder = new RunaroundBuilder(scenario.TrainService, scenario.GraphAdapter, analyzer);

            var action = builder.BuildRunaroundAction(CarGroup.Empty, checker);
            Assert.That(action, Is.Null);
        }

        // =====================================================================
        // Integration: connected segments have routes, disconnected don't
        // =====================================================================

        // 3.34: Connected segments — route exists
        [Test]
        public void Connected_segments_route_exists()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("seg1", "n0", "n1", 200)
                .AddSegment("seg2", "n1", "n2", 300)
                .PlaceLoco("loco", "seg1", 100)
                .Build();

            var from = new GraphPosition("seg1", 100, Direction.TowardEndB);
            var to = new GraphPosition("seg2", 150, Direction.TowardEndB);
            var route = scenario.GraphAdapter.FindRoute(from, to);

            Assert.That(route, Is.Not.Null);
            Assert.That(route!.Value.Distance, Is.GreaterThan(0));
        }

        // 3.35: Route through a switch
        [Test]
        public void Route_through_switch()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 300)
                .AddSegment("siding1", "sw1", "n1", 200)
                .AddSegment("siding2", "sw1", "n2", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding2")
                .PlaceLoco("loco", "lead", 150)
                .Build();

            var from = new GraphPosition("lead", 150, Direction.TowardEndB);
            var to = new GraphPosition("siding2", 100, Direction.TowardEndB);
            var route = scenario.GraphAdapter.FindRoute(from, to);

            Assert.That(route, Is.Not.Null);
            Assert.That(route!.Value.RouteSegmentIds!.Count, Is.GreaterThanOrEqualTo(2));
        }

        // =====================================================================
        // PickupPlanner interface-based constructor compiles and doesn't crash
        // =====================================================================

        // 3.36: PickupPlanner can be constructed with ITrainService
        [Test]
        public void PickupPlanner_interface_constructor()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "n1", 500)
                .PlaceLoco("loco", "lead", 250)
                .Build();

            // Should not throw
            var planner = new PickupPlanner(scenario.TrainService);
            Assert.That(planner, Is.Not.Null);
        }

        // =====================================================================
        // MatchesFilter edge cases with From:Any modes
        // =====================================================================

        // 3.37: From:Area mode — MatchesFilter only checks waybill + switchlist
        // (Area/Industry checks are in MatchesFromFilter which is game-coupled)
        [Test]
        public void MatchesFilter_from_area_still_passes_base_checks()
        {
            var car = WaybilledCar("c1", "Mill S1", "mill-s1");
            var filter = new PickupFilter(
                new FilterAxis(FilterMode.Area, new HashSet<string> { "SomeArea" }),
                FilterAxis.Any, float.MaxValue, false);
            // Base MatchesFilter only checks waybill + switchlist membership.
            // Area check is in MatchesFromFilter (private, game-coupled).
            Assert.That(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()), Is.True);
        }

        // 3.38: From:Industry mode — base filter passes for waybilled car
        [Test]
        public void MatchesFilter_from_industry_passes_base_checks()
        {
            var car = WaybilledCar("c1", "Mill S1", "mill-s1");
            var filter = new PickupFilter(
                new FilterAxis(FilterMode.Industry, new HashSet<string> { "SomeIndustry" }),
                FilterAxis.Any, float.MaxValue, false);
            Assert.That(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()), Is.True);
        }

        // 3.39: To:Area mode — base filter passes (Area check is in MatchesToFilter)
        [Test]
        public void MatchesFilter_to_area_passes_base_checks()
        {
            var car = WaybilledCar("c1", "Mill S1", "mill-s1");
            var filter = new PickupFilter(
                FilterAxis.Any,
                new FilterAxis(FilterMode.Area, new HashSet<string> { "SomeArea" }),
                float.MaxValue, false);
            Assert.That(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()), Is.True);
        }

        // 3.40: To:Industry mode — base filter passes
        [Test]
        public void MatchesFilter_to_industry_passes_base_checks()
        {
            var car = WaybilledCar("c1", "Mill S1", "mill-s1");
            var filter = new PickupFilter(
                FilterAxis.Any,
                new FilterAxis(FilterMode.Industry, new HashSet<string> { "SomeIndustry" }),
                float.MaxValue, false);
            Assert.That(PickupPlanner.MatchesFilter(car, filter, new HashSet<string>()), Is.True);
        }

        // 3.41: Car with no waybill fails even with permissive filter
        [Test]
        public void MatchesFilter_no_waybill_fails_even_any_any()
        {
            var car = new MockCar { id = "c1", DisplayName = "Empty Car" };
            Assert.That(PickupPlanner.MatchesFilter(car, PickupFilter.Default, new HashSet<string>()), Is.False);
        }
    }
}
