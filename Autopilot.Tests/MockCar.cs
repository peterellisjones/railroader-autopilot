using Model;
using Model.Definition;
using Model.Ops;
using Track;
using Autopilot.Model;

namespace Autopilot.Tests
{
    public class MockCar : ICar
    {
        public string DisplayName { get; set; } = "MockCar";
        public string id { get; set; } = "mock";
        public bool IsLocomotive { get; set; }
        public bool IsLocoOrTender { get; set; }
        public float CarLength { get; set; } = 10f;
        public CarArchetype Archetype { get; set; } = CarArchetype.Gondola;
        public Waybill? Waybill { get; set; }
        public DirectedPosition EndA { get; set; }
        public DirectedPosition EndB { get; set; }
        public DirectedPosition Front { get; set; }
        public DirectedPosition Rear { get; set; }

        public ICar? CoupledAtA { get; set; }
        public ICar? CoupledAtB { get; set; }
        public bool CoupledEndGearA { get; set; }
        public bool CoupledEndGearB { get; set; }

        public Car.LogicalEnd ClosestLogicalEndTo(DirectedPosition pos)
            => Car.LogicalEnd.A;

        public Car.End LogicalToEnd(Car.LogicalEnd logical)
            => logical == Car.LogicalEnd.A ? Car.End.F : Car.End.R;

        public ICar? CoupledTo(Car.LogicalEnd end)
            => end == Car.LogicalEnd.A ? CoupledAtA : CoupledAtB;

        public bool IsCoupled(Car.LogicalEnd end)
            => end == Car.LogicalEnd.A ? CoupledEndGearA : CoupledEndGearB;

        /// <summary>Helper: couple two MockCars together (a.B -> b.A)</summary>
        public static void Couple(MockCar a, MockCar b)
        {
            a.CoupledAtB = b;
            a.CoupledEndGearB = true;
            b.CoupledAtA = a;
            b.CoupledEndGearA = true;
        }
    }
}
