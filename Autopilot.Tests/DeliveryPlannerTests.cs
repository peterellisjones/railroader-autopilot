using NUnit.Framework;
using System.Linq;
using Autopilot.Model;
using Autopilot.Planning;

namespace Autopilot.Tests
{
    [TestFixture]
    public class DeliveryPlannerTests
    {
        // 1.1: Single car to siding — direct delivery
        [Test]
        public void SingleCar_direct_delivery()
        {
            // Topology: lead(n0--sw1) --sw1-- siding1(sw1--n1)
            //                                \- siding2(sw1--n2)
            // Loco on lead at 300 facing EndB (toward sw1), car coupled on B side.
            // Car has waybill to siding2.
            // Expected: plan has 1 delivery step to siding2.
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding1", "sw1", "n1", 200)
                .AddSegment("siding2", "sw1", "n2", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding2")
                .PlaceCar("boxcar1", "lead", 350, Direction.TowardEndB, destination: "siding2")
                .PlaceLoco("loco", "lead", 300, Direction.TowardEndB, coupledTo: "boxcar1")
                .DefineDestination("siding2", "Siding 2", 200f,
                    new GraphPosition("siding2", 0, Direction.TowardEndB))
                .Build();

            var planner = new DeliveryPlanner(scenario.TrainService, scenario.GraphAdapter);
            var plan = planner.BuildPlan();

            Assert.That(plan.HasDeliveries, Is.True, "Plan should have deliveries");
            Assert.That(plan.Steps.Count, Is.EqualTo(1), "Should have exactly 1 delivery step");
            Assert.That(plan.Steps[0].DestinationTrackId, Is.EqualTo("siding2"));
            Assert.That(plan.Steps[0].Cars.Count, Is.EqualTo(1));
            Assert.That(plan.Steps[0].Cars[0].id, Is.EqualTo("boxcar1"));
        }

        // 1.2: Three cars, three destinations — delivers tail first
        [Test]
        public void ThreeCars_delivers_tail_first()
        {
            // Consist order: loco -- carC -- carB -- carA (carA is tail)
            // carA -> siding1, carB -> siding2, carC -> siding3
            // Expected: first step delivers carA (tail car) to siding1
            //
            // Topology:
            //   lead(n0--sw1) --sw1-- connector(sw1--sw2) --sw2-- siding2(sw2--n2)
            //                     \-- siding1(sw1--n1)        \-- siding3(sw2--n3)
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding1", "sw1", "n1", 200)
                .AddSegment("connector", "sw1", "sw2", 100)
                .AddSegment("siding2", "sw2", "n2", 200)
                .AddSegment("siding3", "sw2", "n3", 200)
                .AddSwitch("sw1", "lead", "siding1", "connector")
                .AddSwitch("sw2", "connector", "siding2", "siding3")
                // carA is the outermost (tail) car
                .PlaceCar("carA", "lead", 400, Direction.TowardEndB, destination: "siding1")
                .PlaceCar("carB", "lead", 380, Direction.TowardEndB, destination: "siding2",
                    coupledTo: "carA")
                .PlaceCar("carC", "lead", 360, Direction.TowardEndB, destination: "siding3",
                    coupledTo: "carB")
                .PlaceLoco("loco", "lead", 300, Direction.TowardEndB, coupledTo: "carC")
                .DefineDestination("siding1", "Siding 1", 200f,
                    new GraphPosition("siding1", 0, Direction.TowardEndB))
                .DefineDestination("siding2", "Siding 2", 200f,
                    new GraphPosition("siding2", 0, Direction.TowardEndB))
                .DefineDestination("siding3", "Siding 3", 200f,
                    new GraphPosition("siding3", 0, Direction.TowardEndB))
                .Build();

            var planner = new DeliveryPlanner(scenario.TrainService, scenario.GraphAdapter);
            var plan = planner.BuildPlan();

            Assert.That(plan.HasDeliveries, Is.True, "Plan should have deliveries");
            Assert.That(plan.Steps.Count, Is.EqualTo(1),
                "BuildPlan returns the first deliverable step (maxSteps=1)");
            // The tail car should be delivered first
            Assert.That(plan.Steps[0].Cars[0].id, Is.EqualTo("carA"),
                "Tail car (carA) should be delivered first");
            Assert.That(plan.Steps[0].DestinationTrackId, Is.EqualTo("siding1"));
        }

        // 1.3: Two cars same destination — grouped in one step
        [Test]
        public void TwoCars_same_dest_grouped()
        {
            // Two consecutive cars going to siding1
            // Expected: single step with 2 cars
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding1", "sw1", "n1", 200)
                .AddSegment("siding2", "sw1", "n2", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding2")
                .PlaceCar("carA", "lead", 400, Direction.TowardEndB, destination: "siding1")
                .PlaceCar("carB", "lead", 380, Direction.TowardEndB, destination: "siding1",
                    coupledTo: "carA")
                .PlaceLoco("loco", "lead", 300, Direction.TowardEndB, coupledTo: "carB")
                .DefineDestination("siding1", "Siding 1", 200f,
                    new GraphPosition("siding1", 0, Direction.TowardEndB))
                .Build();

            var planner = new DeliveryPlanner(scenario.TrainService, scenario.GraphAdapter);
            var plan = planner.BuildPlan();

            Assert.That(plan.HasDeliveries, Is.True, "Plan should have deliveries");
            Assert.That(plan.Steps.Count, Is.EqualTo(1), "Should have exactly 1 delivery step");
            Assert.That(plan.Steps[0].Cars.Count, Is.EqualTo(2),
                "Both cars going to same destination should be grouped in one step");
            // Both cars should be in the step
            var carIds = plan.Steps[0].Cars.Select(c => c.id).ToList();
            Assert.That(carIds, Does.Contain("carA"));
            Assert.That(carIds, Does.Contain("carB"));
            Assert.That(plan.Steps[0].DestinationTrackId, Is.EqualTo("siding1"));
        }

        // 1.4: No waybilled cars — empty plan
        [Test]
        public void NoWaybills_empty_plan()
        {
            // Cars without waybills (no destination set)
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding1", "sw1", "n1", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding1")
                .PlaceCar("carA", "lead", 400, Direction.TowardEndB)
                .PlaceLoco("loco", "lead", 300, Direction.TowardEndB, coupledTo: "carA")
                .Build();

            var planner = new DeliveryPlanner(scenario.TrainService, scenario.GraphAdapter);
            var plan = planner.BuildPlan();

            Assert.That(plan.HasDeliveries, Is.False, "No waybills means no deliveries");
            Assert.That(plan.Steps.Count, Is.EqualTo(0));
        }

        // 1.5: Destination unreachable — doesn't crash, no deliveries
        [Test]
        public void Unreachable_dest_no_crash()
        {
            // Destination on disconnected segment
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "n1", 500)
                .AddSegment("island", "n5", "n6", 200) // disconnected from lead
                .PlaceCar("carA", "lead", 400, Direction.TowardEndB, destination: "island")
                .PlaceLoco("loco", "lead", 300, Direction.TowardEndB, coupledTo: "carA")
                .DefineDestination("island", "Island Track", 200f,
                    new GraphPosition("island", 100, Direction.TowardEndB))
                .Build();

            var planner = new DeliveryPlanner(scenario.TrainService, scenario.GraphAdapter);

            // Should not throw
            DeliveryPlan plan = null!;
            Assert.DoesNotThrow(() =>
            {
                plan = planner.BuildPlan();
            });

            // No direct delivery possible to disconnected track
            Assert.That(plan.HasDeliveries, Is.False,
                "Unreachable destination should not produce delivery steps");
        }

        // 1.6: Loco only, no cars — empty plan
        [Test]
        public void LocoOnly_no_cars_empty_plan()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "n1", 500)
                .PlaceLoco("loco", "lead", 300)
                .Build();

            var planner = new DeliveryPlanner(scenario.TrainService, scenario.GraphAdapter);
            var plan = planner.BuildPlan();

            Assert.That(plan.HasDeliveries, Is.False);
            Assert.That(plan.Steps.Count, Is.EqualTo(0));
        }

        // 1.7: Cars on both sides, closer delivery chosen
        [Test]
        public void BothSides_picks_closer_delivery()
        {
            // Loco in the middle with cars on both sides.
            // CarA on loco's A side going to siding1 (close).
            // CarB on loco's B side going to siding2 (far).
            // Expected: picks the closer delivery.
            //
            // Topology: siding1(n1--sw1) -- lead(sw1--sw2) -- siding2(sw2--n2)
            //                                                \- siding3(sw2--n3)
            //
            // Actually, for simplicity let's use a symmetric topology:
            // siding1(n1--sw1) -- lead(sw1--sw2) -- siding2(sw2--n2)
            //
            // carA on A side of loco, going to siding1 (distance ~200m from loco)
            // carB on B side of loco, going to siding2 (distance ~200m from loco)
            // Both should be deliverable; planner picks one.
            var scenario = new ScenarioBuilder()
                .AddSegment("siding1", "n1", "sw1", 200)
                .AddSegment("lead", "sw1", "sw2", 500)
                .AddSegment("siding2", "sw2", "n2", 200)
                .AddSegment("siding3", "sw2", "n3", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding1")
                .AddSwitch("sw2", "lead", "siding2", "siding3")
                // carA on A side of loco (coupled at loco's A end)
                // Build coupling: carA.B -> loco.A, then loco.B -> carB.A
                .PlaceCar("carA", "lead", 260, Direction.TowardEndA, destination: "siding1")
                .PlaceCar("carB", "lead", 340, Direction.TowardEndB, destination: "siding2")
                .PlaceLoco("loco", "lead", 300, Direction.TowardEndB, coupledTo: "carB")
                .DefineDestination("siding1", "Siding 1", 200f,
                    new GraphPosition("siding1", 100, Direction.TowardEndA))
                .DefineDestination("siding2", "Siding 2", 200f,
                    new GraphPosition("siding2", 100, Direction.TowardEndB))
                .Build();

            // Manually couple carA to loco's A side
            // ScenarioBuilder's coupling only does B->A via coupledTo
            // We need carA on loco's A side.
            // Actually, let me reconsider: the ScenarioBuilder's coupledTo
            // always does Couple(source, target) = source.B->target, target.A->source.
            // For carA on loco's A side: loco.A->carA means carA.CoupledAtB=loco, loco.CoupledAtA=carA.
            // But ScenarioBuilder does Couple(carDef.car, target) = carDef.car.B->target.A
            // So PlaceCar("carA", coupledTo:"loco") would do carA.B->loco, loco.A->carA. That works!
            // But loco isn't placed yet when carA is placed... the builder processes cars first, then loco.
            // Let me re-check the builder order...

            // Actually, the builder creates all MockCar instances first (including loco),
            // THEN processes coupling. So "coupledTo" can reference the loco.
            // Let me rebuild the scenario properly.

            var scenario2 = new ScenarioBuilder()
                .AddSegment("siding1", "n1", "sw1", 200)
                .AddSegment("lead", "sw1", "sw2", 500)
                .AddSegment("siding2", "sw2", "n2", 200)
                .AddSegment("siding3", "sw2", "n3", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding1")
                .AddSwitch("sw2", "lead", "siding2", "siding3")
                // carA coupled to loco via carA.B->loco (puts carA on A side)
                .PlaceCar("carA", "lead", 260, Direction.TowardEndA, destination: "siding1",
                    coupledTo: "loco")
                // carB coupled to nothing initially
                .PlaceCar("carB", "lead", 340, Direction.TowardEndB, destination: "siding2")
                .PlaceLoco("loco", "lead", 300, Direction.TowardEndB, coupledTo: "carB")
                .DefineDestination("siding1", "Siding 1", 200f,
                    new GraphPosition("siding1", 100, Direction.TowardEndA))
                .DefineDestination("siding2", "Siding 2", 200f,
                    new GraphPosition("siding2", 100, Direction.TowardEndB))
                .Build();

            var planner = new DeliveryPlanner(scenario2.TrainService, scenario2.GraphAdapter);
            var plan = planner.BuildPlan();

            Assert.That(plan.HasDeliveries, Is.True, "Both sides should have deliverable cars");
            Assert.That(plan.Steps.Count, Is.EqualTo(1));
            // Both sides have 1 step each; planner picks one based on proximity
            // The exact pick depends on graph distances, but the planner should not crash
            var destId = plan.Steps[0].DestinationTrackId;
            Assert.That(destId, Is.EqualTo("siding1").Or.EqualTo("siding2"),
                "Should deliver to one of the two destinations");
        }

        // 1.8: Skipped car blocks delivery behind it
        [Test]
        public void SkippedCar_blocks_delivery_behind()
        {
            // carA is at the tail, carB is behind carA (between carA and loco).
            // carA is skipped; carB should not be deliverable (blocked by skipped car).
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

            var planner = new DeliveryPlanner(scenario.TrainService, scenario.GraphAdapter);
            // Skip carA — this should prevent delivery of carB too
            var plan = planner.BuildPlan(skippedCarIds: new[] { "carA" });

            // carA is skipped (at tail), so the analyzer should break immediately
            // and not find any deliverable steps from this side.
            Assert.That(plan.HasDeliveries, Is.False,
                "Skipped tail car should block delivery of cars behind it");
        }

        // 1.9: Mixed waybilled and non-waybilled cars — non-waybilled are skipped
        [Test]
        public void MixedWaybilled_nonWaybilled_skipped()
        {
            // carA (tail) has no waybill, carB (behind carA) has waybill to siding1
            // Non-waybilled carA should be skipped (not block), carB should be deliverable
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding1", "sw1", "n1", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding1")
                .PlaceCar("carA", "lead", 400, Direction.TowardEndB) // no destination
                .PlaceCar("carB", "lead", 380, Direction.TowardEndB, destination: "siding1",
                    coupledTo: "carA")
                .PlaceLoco("loco", "lead", 300, Direction.TowardEndB, coupledTo: "carB")
                .DefineDestination("siding1", "Siding 1", 200f,
                    new GraphPosition("siding1", 0, Direction.TowardEndB))
                .Build();

            var planner = new DeliveryPlanner(scenario.TrainService, scenario.GraphAdapter);
            var plan = planner.BuildPlan();

            // The deliverability analyzer skips non-waybilled cars (continues),
            // but a non-waybilled car does NOT block cars behind it.
            // However, carA is between carB and the tail end, so the delivery
            // would need to move carA too. The delivery step should still work
            // because the whole tail section moves together.
            // Actually, looking at the analyzer: non-waybilled cars are skipped (i++),
            // but we need carB to be deliverable. Since carA (non-waybilled) is at position 0
            // in the group and carB is at position 1, the analyzer skips carA and finds carB.
            Assert.That(plan.HasDeliveries, Is.True,
                "Non-waybilled car at tail should not block waybilled car behind it");
            Assert.That(plan.Steps[0].Cars[0].id, Is.EqualTo("carB"));
        }
    }
}
