#include <traci_api/TraCIAPI.h>
#include <iostream>

int main(int argc, char* argv[]) {
  /* traci client */

  TraCIAPI traci;

  traci.connect("localhost", 1337);
  std::cout << "time in ms: " << traci.simulation.getCurrentTime() << "\n";

  std::vector<std::string> ids = traci.simulation.getLoadedIDList();
  for (int i = 0; i < ids.size(); i++) {
    std::cout << ids[i] << ", ";
  }
  std::cout << "Loaded vehicle COUNT = " << traci.simulation.getLoadedNumber()
            << std::endl;

  // main loop. do something every simulation step until no more vehicles are
  // loaded or running
  int step = 0;
  int release_time = 1000;
  while (traci.simulation.getMinExpectedNumber() > 0) {
    traci.simulationStep();
    step += 1;
    for (std::string vehicle : traci.vehicle.getIDList()) {
      if (vehicle == "veh0") {
        if (abs(traci.vehicle.getPosition(vehicle).y - (-15)) < 0.1 &&
            release_time-- > 0)
          traci.vehicle.setSpeed(vehicle, 0.0);
        else
          traci.vehicle.setSpeed(vehicle, 5.0);
        if (step % 10 == 0) {
          std::cout << traci.vehicle.getSpeed(vehicle) << ", "
                    << traci.vehicle.getPosition(vehicle).x << ", "
                    << traci.vehicle.getPosition(vehicle).y << ", "
                    << std::endl;
        }
      }
    }
  }

  traci.simulationStep(1 * 100);

  std::cout << "time in ms: " << traci.simulation.getCurrentTime() << "\n";
  traci.close();
}
