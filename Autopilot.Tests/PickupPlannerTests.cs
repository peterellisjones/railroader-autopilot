using System.Collections.Generic;
using NUnit.Framework;
using Autopilot.Model;
using Autopilot.Planning;

namespace Autopilot.Tests
{
    [TestFixture]
    public class PickupPlannerTests
    {
        [Test]
        public void FilterAccessibleTargets_empty_chain_returns_empty()
        {
            var chain = new List<ICar>();
            var targetIds = new HashSet<string>();
            var result = PickupPlanner.FilterAccessibleTargets(chain, targetIds);
            Assert.AreEqual(0, result.Count);
        }

        [Test]
        public void FilterAccessibleTargets_single_target_car_returns_it()
        {
            var car = new MockCar { DisplayName = "C1", id = "c1" };
            var chain = new List<ICar> { car };
            var targetIds = new HashSet<string> { "c1" };
            var result = PickupPlanner.FilterAccessibleTargets(chain, targetIds);
            Assert.AreEqual(1, result.Count);
            Assert.AreEqual("C1", result[0].DisplayName);
        }

        [Test]
        public void FilterAccessibleTargets_single_non_target_returns_empty()
        {
            var car = new MockCar { DisplayName = "C1", id = "c1" };
            var chain = new List<ICar> { car };
            var targetIds = new HashSet<string>();
            var result = PickupPlanner.FilterAccessibleTargets(chain, targetIds);
            Assert.AreEqual(0, result.Count);
        }

        [Test]
        public void FilterAccessibleTargets_all_targets_returns_all()
        {
            var c1 = new MockCar { DisplayName = "C1", id = "c1" };
            var c2 = new MockCar { DisplayName = "C2", id = "c2" };
            MockCar.Couple(c1, c2);
            var chain = new List<ICar> { c1, c2 };
            var targetIds = new HashSet<string> { "c1", "c2" };
            var result = PickupPlanner.FilterAccessibleTargets(chain, targetIds);
            Assert.AreEqual(2, result.Count);
        }

        [Test]
        public void FilterAccessibleTargets_targets_then_nontargets_returns_targets_only()
        {
            var c1 = new MockCar { DisplayName = "C1", id = "c1" };
            var c2 = new MockCar { DisplayName = "C2", id = "c2" };
            var c3 = new MockCar { DisplayName = "C3", id = "c3" };
            MockCar.Couple(c1, c2);
            MockCar.Couple(c2, c3);
            var chain = new List<ICar> { c1, c2, c3 };
            var targetIds = new HashSet<string> { "c1", "c2" };
            var result = PickupPlanner.FilterAccessibleTargets(chain, targetIds);
            Assert.AreEqual(2, result.Count);
            Assert.AreEqual("C1", result[0].DisplayName);
            Assert.AreEqual("C2", result[1].DisplayName);
        }

        [Test]
        public void FilterAccessibleTargets_nontarget_on_approach_blocks_everything()
        {
            var c1 = new MockCar { DisplayName = "C1", id = "c1" };
            var c2 = new MockCar { DisplayName = "C2", id = "c2" };
            MockCar.Couple(c1, c2);
            var chain = new List<ICar> { c1, c2 };
            var targetIds = new HashSet<string> { "c2" };
            var result = PickupPlanner.FilterAccessibleTargets(chain, targetIds);
            Assert.AreEqual(0, result.Count);
        }

        [Test]
        public void FilterAccessibleTargets_gap_in_targets_stops_at_gap()
        {
            var c1 = new MockCar { DisplayName = "C1", id = "c1" };
            var c2 = new MockCar { DisplayName = "C2", id = "c2" };
            var c3 = new MockCar { DisplayName = "C3", id = "c3" };
            MockCar.Couple(c1, c2);
            MockCar.Couple(c2, c3);
            var chain = new List<ICar> { c1, c2, c3 };
            var targetIds = new HashSet<string> { "c1", "c3" };
            var result = PickupPlanner.FilterAccessibleTargets(chain, targetIds);
            Assert.AreEqual(1, result.Count);
            Assert.AreEqual("C1", result[0].DisplayName);
        }

        [Test]
        public void GetCoupledChain_single_car()
        {
            var car = new MockCar { DisplayName = "C1", id = "c1" };
            var chain = PickupPlanner.GetCoupledChain(car);
            Assert.AreEqual(1, chain.Count);
            Assert.AreEqual("C1", chain[0].DisplayName);
        }

        [Test]
        public void GetCoupledChain_three_coupled_cars_from_middle()
        {
            var c1 = new MockCar { DisplayName = "C1", id = "c1" };
            var c2 = new MockCar { DisplayName = "C2", id = "c2" };
            var c3 = new MockCar { DisplayName = "C3", id = "c3" };
            MockCar.Couple(c1, c2);
            MockCar.Couple(c2, c3);
            var chain = PickupPlanner.GetCoupledChain(c2);
            Assert.AreEqual(3, chain.Count);
            Assert.AreEqual("C1", chain[0].DisplayName);
            Assert.AreEqual("C2", chain[1].DisplayName);
            Assert.AreEqual("C3", chain[2].DisplayName);
        }

        [Test]
        public void GetCoupledChain_starting_from_end_car()
        {
            var c1 = new MockCar { DisplayName = "C1", id = "c1" };
            var c2 = new MockCar { DisplayName = "C2", id = "c2" };
            MockCar.Couple(c1, c2);
            var chain = PickupPlanner.GetCoupledChain(c1);
            Assert.AreEqual(2, chain.Count);
            Assert.AreEqual("C1", chain[0].DisplayName);
            Assert.AreEqual("C2", chain[1].DisplayName);
        }
    }
}
