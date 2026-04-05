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

        /// <summary>
        /// Create a ConsistLayout from an ITrainService (abstract/testable path).
        /// The coupled list from ITrainService is already ordered [sideA cars..., loco, sideB cars...].
        /// </summary>
        public static ConsistLayout Create(ITrainService trainService)
        {
            var coupled = trainService.GetCoupled();
            var locoCar = trainService.GetLocoCar();

            int locoIndex = -1;
            for (int i = 0; i < coupled.Count; i++)
            {
                if (coupled[i].id == locoCar.id)
                {
                    locoIndex = i;
                    break;
                }
            }

            if (locoIndex < 0)
                return new ConsistLayout(null, CarGroup.Empty, CarGroup.Empty, false);

            var rawSideA = new List<ICar>();
            for (int i = 0; i < locoIndex; i++)
                rawSideA.Add(coupled[i]);

            var rawSideB = new List<ICar>();
            for (int i = locoIndex + 1; i < coupled.Count; i++)
                rawSideB.Add(coupled[i]);

            var sideA = CarGroup.FromSide(rawSideA, tailAtStart: true);
            var sideB = CarGroup.FromSide(rawSideB, tailAtStart: false);

            bool anyWaybilled = false;
            foreach (var car in rawSideA)
            {
                if (trainService.HasWaybill(car)) { anyWaybilled = true; break; }
            }
            if (!anyWaybilled)
            {
                foreach (var car in rawSideB)
                {
                    if (trainService.HasWaybill(car)) { anyWaybilled = true; break; }
                }
            }

            return new ConsistLayout(null, sideA, sideB, anyWaybilled);
        }
    }
}
