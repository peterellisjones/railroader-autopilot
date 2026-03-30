using Model;
using Model.Definition;
using Model.Ops;
using Track;

namespace Autopilot.Model
{
    public interface ICar
    {
        string DisplayName { get; }
        string id { get; }
        bool IsLocomotive { get; }
        bool IsLocoOrTender { get; }
        float CarLength { get; }
        CarArchetype Archetype { get; }
        Waybill? Waybill { get; }
        DirectedPosition EndA { get; }
        DirectedPosition EndB { get; }
        DirectedPosition Front { get; }
        DirectedPosition Rear { get; }
        Car.LogicalEnd ClosestLogicalEndTo(DirectedPosition pos);
        Car.End LogicalToEnd(Car.LogicalEnd logical);
        ICar? CoupledTo(Car.LogicalEnd end);
        bool IsCoupled(Car.LogicalEnd end);
    }
}
