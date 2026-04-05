using NUnit.Framework;
using Autopilot.Model;

namespace Autopilot.Tests
{
    [TestFixture]
    public class ScenarioBuilderSmokeTest
    {
        [Test]
        public void Build_creates_scenario_with_loco_and_cars()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("lead", "n0", "sw1", 500)
                .AddSegment("siding1", "sw1", "n1", 200)
                .AddSegment("siding2", "sw1", "n2", 200)
                .AddSwitch("sw1", "lead", "siding1", "siding2")
                .PlaceCar("boxcar", "siding1", 50, destination: "siding2")
                .PlaceLoco("loco", "lead", 100, coupledTo: "boxcar")
                .Build();

            Assert.That(scenario.CoupledCars.Count, Is.EqualTo(2));
            Assert.That(scenario.LocoCar.id, Is.EqualTo("loco"));
            Assert.That(scenario.GetCar("boxcar").id, Is.EqualTo("boxcar"));
            Assert.That(scenario.TrainService.HasWaybill(scenario.GetCar("boxcar")));
            Assert.That(scenario.TrainService.GetDestinationTrackId(scenario.GetCar("boxcar")),
                Is.EqualTo("siding2"));
        }

        [Test]
        public void MockGraphAdapter_finds_route_between_connected_segments()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("seg1", "n0", "sw1", 100)
                .AddSegment("seg2", "sw1", "n1", 200)
                .AddSegment("seg3", "sw1", "n2", 150)
                .AddSwitch("sw1", "seg1", "seg2", "seg3")
                .PlaceLoco("loco", "seg1", 50)
                .Build();

            var from = new GraphPosition("seg1", 50, Direction.TowardEndB);
            var to = new GraphPosition("seg2", 100, Direction.TowardEndA);
            var result = scenario.GraphAdapter.FindRoute(from, to);

            Assert.That(result, Is.Not.Null);
            Assert.That(result.Value.RouteSegmentIds, Does.Contain("seg1"));
            Assert.That(result.Value.RouteSegmentIds, Does.Contain("seg2"));
        }

        [Test]
        public void Three_car_consist_ordered_correctly()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("track", "n0", "n1", 500)
                .PlaceCar("carA", "track", 50)
                .PlaceCar("carB", "track", 70, coupledTo: "carA")
                .PlaceLoco("loco", "track", 100, coupledTo: "carB")
                .Build();

            // Coupled list should be: carA, carB, loco
            // (carA and carB are on loco's B side since loco.B->carB.A, carB.B->carA.A)
            Assert.That(scenario.CoupledCars.Count, Is.EqualTo(3));
            Assert.That(scenario.CoupledCars[0].IsLocomotive || scenario.CoupledCars[2].IsLocomotive);
        }

        [Test]
        public void Train_length_computed_correctly()
        {
            var scenario = new ScenarioBuilder()
                .AddSegment("track", "n0", "n1", 500)
                .PlaceCar("car1", "track", 50, length: 10)
                .PlaceLoco("loco", "track", 100, coupledTo: "car1", length: 15)
                .Build();

            // 10 + 15 + 1 gap = 26
            Assert.That(scenario.TrainService.GetTrainLength(), Is.EqualTo(26f));
        }
    }
}
