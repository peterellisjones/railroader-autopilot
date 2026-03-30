using System.Collections.Generic;
using NUnit.Framework;
using Autopilot.Model;
using Autopilot.Planning;
using GameCar = Model.Car;

namespace Autopilot.Tests
{
    [TestFixture]
    public class CarGroupTests
    {
        [Test]
        public void Empty_group_has_no_cars()
        {
            var group = CarGroup.Empty;
            Assert.IsTrue(group.IsEmpty);
            Assert.AreEqual(0, group.Count);
            Assert.IsNull(group.TailCar);
        }

        [Test]
        public void FromSide_orders_tail_first_when_tail_at_start()
        {
            var c1 = new MockCar { DisplayName = "C1" };
            var c2 = new MockCar { DisplayName = "C2" };
            MockCar.Couple(c1, c2);

            var group = CarGroup.FromSide(new List<ICar> { c1, c2 }, tailAtStart: true);

            Assert.AreEqual(2, group.Count);
            Assert.AreEqual("C1", group.TailCar.DisplayName);
            Assert.AreEqual("C2", group.LocomotiveEndCar.DisplayName);
        }

        [Test]
        public void FromSide_reverses_when_tail_at_end()
        {
            var c1 = new MockCar { DisplayName = "C1" };
            var c2 = new MockCar { DisplayName = "C2" };
            MockCar.Couple(c1, c2);

            var group = CarGroup.FromSide(new List<ICar> { c1, c2 }, tailAtStart: false);

            Assert.AreEqual("C2", group.TailCar.DisplayName);
            Assert.AreEqual("C1", group.LocomotiveEndCar.DisplayName);
        }

        [Test]
        public void Reversed_reverses_order()
        {
            var c1 = new MockCar { DisplayName = "C1" };
            var c2 = new MockCar { DisplayName = "C2" };
            MockCar.Couple(c1, c2);

            var group = CarGroup.FromSide(new List<ICar> { c1, c2 }, tailAtStart: true);
            var reversed = group.Reversed();

            Assert.AreEqual("C2", reversed.TailCar.DisplayName);
            Assert.AreEqual("C1", reversed.LocomotiveEndCar.DisplayName);
        }

        [Test]
        public void Single_car_group()
        {
            var c1 = new MockCar { DisplayName = "C1" };
            var group = CarGroup.FromSide(new List<ICar> { c1 }, tailAtStart: true);

            Assert.AreEqual(1, group.Count);
            Assert.AreEqual("C1", group.TailCar.DisplayName);
            Assert.AreEqual("C1", group.LocomotiveEndCar.DisplayName);
        }

        [Test]
        public void FindCutPoint_returns_null_for_empty_group()
        {
            var group = CarGroup.Empty;
            var (car, end) = CutPointFinder.FindCutPoint(group, new List<GameCar>());
            Assert.IsNull(car);
        }
    }
}
