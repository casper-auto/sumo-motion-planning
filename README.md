# sumo-motion-planning

SUMO is a high level simulator for the traffic and autonomous vehicles.

## Dependencies
SpeedPlanner package needs to be compiled and installed before compiling sumo-motion-planning package.

## Install SUMO simulator on Linux / Ubuntu

If you run debian or ubuntu, SUMO is part of the regular distribution and can be installed like this:

```bash
sudo apt-get install sumo sumo-tools sumo-doc
```

If you need a more up-to-date ubuntu version, it may be found in a separate ppa, which is added like this:

```bash
sudo add-apt-repository ppa:sumo/stable
sudo apt-get update
```

and then again

```bash
sudo apt-get install sumo sumo-tools sumo-doc
```

## Using TraCI with python (under ./traci-python)

After starting the sumo simulator, simply run the python script:

```bash
python runner.py
```

## Using TraCI/C++TraCIAPI (under ./traci-cpp)

Under ./traci-cpp there is a concised Traci C++ API called "traci_api" for sumo, we put our developed commanding
programs in the ./tests folder.

### Example Useage

#### Under ./traci-cpp, compile

```bash
mkdir build
cmake ..
make -j 8
```

The executables will be generated in ./traci-cpp/bin.

#### Run executables

Step 1. Start sumo simulator in one terminal with the port number.

In scenarios folder, pick a scenario and parse the port number (here we hard-coded it as 1337),

```bash
sumo-gui -c scenario.sumocfg --remote-port 1337
```

Step 2. Open another terminal, and under ./traci-cpp/bin run.

```bash
./speed_planner_client --relative-path-to-sumocfg-file --map-foler-path
```

Example:

```bash
cd scenarios/enter_road
sumo-gui -c scenario.sumocfg --remote-port 1337
```

and

```bash
cd ./traci-cpp/bin
./speed_planner_client ../../scenarios/enter_road/scenario.sumocfg ../data
```
