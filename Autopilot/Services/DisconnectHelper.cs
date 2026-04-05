using System.Collections.Generic;
using Model;
using Autopilot.Model;

namespace Autopilot.Services
{
    public static class DisconnectHelper
    {
        /// <summary>
        /// Set handbrakes and bleed air on disconnected cars (game Car objects).
        /// Called after uncoupling to secure dropped cars.
        /// </summary>
        public static void DisconnectCars(IEnumerable<Car> cars, TrainService trainService)
        {
            foreach (var car in cars)
            {
                trainService.SetHandbrake(car, true);
                trainService.BleedAir(car);
            }
        }

        /// <summary>
        /// Set handbrakes and bleed air on disconnected cars (ICar objects).
        /// Unwraps CarAdapter to get game Car objects.
        /// </summary>
        public static void DisconnectCars(IEnumerable<ICar> cars, TrainService trainService)
        {
            foreach (var car in cars)
            {
                var gameCar = (car as CarAdapter)?.Car;
                if (gameCar != null)
                {
                    trainService.SetHandbrake(gameCar, true);
                    trainService.BleedAir(gameCar);
                }
            }
        }
    }
}
