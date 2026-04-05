using System.Collections.Generic;
using Autopilot.Model;
using Autopilot.Services;
using Autopilot.TrackGraph;

namespace Autopilot.Tests
{
    public class Scenario
    {
        public MockTrainService TrainService { get; }
        public MockGraphAdapter GraphAdapter { get; }
        public IReadOnlyList<ICar> CoupledCars { get; }
        public ICar LocoCar { get; }
        private readonly Dictionary<string, ICar> _cars;

        public Scenario(MockTrainService trainService, MockGraphAdapter graphAdapter,
            IReadOnlyList<ICar> coupledCars, ICar locoCar, Dictionary<string, ICar> cars)
        {
            TrainService = trainService;
            GraphAdapter = graphAdapter;
            CoupledCars = coupledCars;
            LocoCar = locoCar;
            _cars = cars;
        }

        public ICar GetCar(string id) => _cars[id];
    }
}
