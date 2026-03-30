using System.Collections.Generic;
using Model;

namespace Autopilot.Services
{
    public static class DisconnectHelper
    {
        /// <summary>
        /// Set handbrakes and bleed air on disconnected cars.
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
    }
}
