using Model;
using Model.Definition;
using Model.Ops;
using Track;

namespace Autopilot.Model
{
    public class CarAdapter : ICar
    {
        private readonly Car _car;

        public CarAdapter(Car car) => _car = car;

        public string DisplayName => _car.DisplayName;
        public string id => _car.id;
        public bool IsLocomotive => _car.IsLocomotive;
        public bool IsLocoOrTender
        {
            get
            {
                var arch = _car.Archetype;
                return arch == CarArchetype.LocomotiveDiesel
                    || arch == CarArchetype.LocomotiveSteam
                    || arch == CarArchetype.Tender;
            }
        }
        public Waybill? Waybill => _car.Waybill;
        public DirectedPosition EndA => DirectedPosition.FromLocation(_car.LocationA);
        public DirectedPosition EndB => DirectedPosition.FromLocation(_car.LocationB);
        public DirectedPosition Front => DirectedPosition.FromLocation(_car.LocationF);
        public DirectedPosition Rear => DirectedPosition.FromLocation(_car.LocationR);
        public Car Car => _car;
        public float CarLength => _car.carLength;
        public CarArchetype Archetype => _car.Archetype;

        public Car.LogicalEnd ClosestLogicalEndTo(DirectedPosition pos)
            => _car.ClosestLogicalEndTo(pos.ToLocation(), Graph.Shared);

        public Car.End LogicalToEnd(Car.LogicalEnd logical)
            => _car.LogicalToEnd(logical);

        public ICar? CoupledTo(Car.LogicalEnd end)
        {
            try
            {
                var coupled = _car.CoupledTo(end);
                return coupled != null ? new CarAdapter(coupled) : null;
            }
            catch { return null; }
        }

        public bool IsCoupled(Car.LogicalEnd end)
        {
            var endGear = end == Car.LogicalEnd.A ? _car.EndGearA : _car.EndGearB;
            return endGear.IsCoupled;
        }

        public override bool Equals(object? obj)
        {
            if (obj is CarAdapter other) return _car == other._car;
            return false;
        }

        public override int GetHashCode() => _car.GetHashCode();
    }
}
