// Autopilot/Model/ConsistLayout.cs
using System.Collections.Generic;
using System.Linq;
using Model;
using Autopilot.Services;

namespace Autopilot.Model
{
    public class ConsistLayout
    {
        public BaseLocomotive? Loco { get; }
        public CarGroup SideA { get; }
        public CarGroup SideB { get; }
        public bool HasWaybilledCars { get; }

        private ConsistLayout(BaseLocomotive? loco, CarGroup sideA, CarGroup sideB, bool hasWaybilledCars)
        {
            Loco = loco;
            SideA = sideA;
            SideB = sideB;
            HasWaybilledCars = hasWaybilledCars;
        }

        public static ConsistLayout Create(BaseLocomotive loco, TrainService trainService)
        {
            var consist = trainService.GetCoupled(loco);
            int locoIndex = consist.FindIndex(c => c == (Car)loco);

            if (locoIndex < 0)
                return new ConsistLayout(loco, CarGroup.Empty, CarGroup.Empty, false);

            var rawSideA = consist.Take(locoIndex).Select(c => (ICar)new CarAdapter(c)).ToList();
            var rawSideB = consist.Skip(locoIndex + 1).Select(c => (ICar)new CarAdapter(c)).ToList();

            var sideA = CarGroup.FromSide(rawSideA, tailAtStart: true);
            var sideB = CarGroup.FromSide(rawSideB, tailAtStart: false);

            bool anyWaybilled = rawSideA.Concat(rawSideB).Any(c => c.Waybill != null);

            return new ConsistLayout(loco, sideA, sideB, anyWaybilled);
        }
    }
}
