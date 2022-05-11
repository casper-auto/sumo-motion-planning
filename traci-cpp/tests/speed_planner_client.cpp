#include <speed_planner/speed_planner.h>
#include <traci_api/TraCIAPI.h>
#include <iostream>
#include <string>
#include <random>
#include <unordered_set>
#include <unordered_map>

// using namespace std;
using namespace HRI_Planning;

/**
 * Example use:
 *   speed_planner_client scenario_file
 *
 */

bool FindVehicle(std::vector<std::string> vehicles, std::string veh) {
  for (std::string vehicle : vehicles) {
    if (vehicle == veh) {
      return true;
    }
  }
  return false;
}

int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cout << "Need more parameters." << std::endl;
    std::cout << "Example:" << std::endl;
    std::cout << "./speed_planner_client scenario.sumocfg map_folder_path" << std::endl;
    return 1;
  }

  std::string sumo_file = argv[1];
  std::string map_folder_path = argv[2];
  std::cout << sumo_file << std::endl;
  std::cout << map_folder_path << std::endl;

  std::cout << "----------------- MAIN ----------------------" << std::endl;
  /* speed planner */
  SpeedPlanner sp;
  std::cout << map_folder_path << std::endl;
  sp.SetMapFolder(map_folder_path);

  std::vector<std::string> scenarios = {"not_defined", "enter_road", "exit_road", "left_turn", "right_turn", "crosswalk"};
  ScenarioID ID;
  for(int i = 0; i < scenarios.size(); i++) {
    std::size_t found = sumo_file.find(scenarios[i]);
    if (found!=std::string::npos) {
      std::cout << "Find Scenario: " << scenarios[i] << '\n';
      ID = ScenarioID(i);
    }
  }

  Status ret = sp.InitScenario(ID);

  /* traci client */
  TraCIAPI traci;
  traci.connect("localhost", 1337);

  std::random_device rd;
  std::mt19937 mt(rd());

  // load local map info
  std::vector<Point2D> raw_ego_path = sp.GetEgoPath();
  std::vector<TrajectoryPoint> raw_ego_traj = sp.GetEgoTraj();
  std::vector<Point2D> target_path = sp.GetTargetPath();

  std::vector<Point2D> ego_path;
  std::vector<double> speed_limits;

  // process the ego path for denser points

  Frenet raw_ego_path_frenet(raw_ego_path);

  Point_Frenet end_sd;
  // int idx = closestPoint(raw_ego_path, ego_x, ego_y);
  raw_ego_path_frenet.ToFrenet(raw_ego_path.back(), raw_ego_path.size() - 1, end_sd);

  std::vector<double> s_vec;
  std::vector<double> v_vec;
  s_vec.push_back(0);
  v_vec.push_back(raw_ego_traj[0].v_limit);
  double s_acc = 0;
  for (int i = 1; i < raw_ego_path.size(); i++) {
    double dx = raw_ego_path[i].x - raw_ego_path[i-1].x;
    double dy = raw_ego_path[i].y - raw_ego_path[i-1].y;
    s_acc += std::sqrt(dx*dx + dy*dy);
    s_vec.push_back(s_acc);
    v_vec.push_back(raw_ego_traj[i].v_limit);
  }

  tk::spline s_and_v;
  s_and_v.set_points(s_vec, v_vec);

  // create new list
  double s_end = end_sd.s;
  ego_path.push_back(raw_ego_path[0]);
  for (double s = 0.5; s < s_end; s += 0.5) {
    double d = 0;
    Point_Frenet sd(s, d);
    Point2D p_xy;
    raw_ego_path_frenet.ToCartesian(sd, p_xy);
    ego_path.push_back(p_xy);
    speed_limits.push_back(s_and_v(s));
  }

  double lower_bound = 5.0;
  double upper_bound = 10.0;
  std::uniform_real_distribution<double> unif(lower_bound,upper_bound);
  std::default_random_engine re;

  // person list
  std::unordered_set<std::string> entered_peds;
  std::unordered_map<std::string, Point2D> ped_prev_positions;

  while (true) {
    traci.load({"-c", sumo_file, "--remote-port", "1337"});
    std::cout << "time in ms: " << traci.simulation.getCurrentTime() << "\n";
    double start_time = traci.simulation.getTime();

    // record the initial position of the ego car
    bool is_initialized = false;
    bool is_initial = false;
    double initial_x, initial_y, initial_speed, initial_angle;
    libsumo::TraCIColor initial_color;
    std::string ego = "veh0";

    // to obtain initial values
    traci.simulationStep();

    // main loop. do something every simulation step until no more vehicles are
    // loaded or running
    double planning_cycle = 0.1;  // assume a planning cycle is 0.1 seconds
    double previous_time = traci.simulation.getTime();
    double elapsed = 0.0;

    while (traci.simulation.getTime() - start_time < 1000.0) {
      if (!is_initialized && FindVehicle(traci.vehicle.getIDList(), ego)) {
        initial_x = traci.vehicle.getPosition(ego).x;
        initial_y = traci.vehicle.getPosition(ego).y;
        initial_speed = traci.vehicle.getSpeed(ego);
        initial_angle = traci.vehicle.getAngle(ego);
        initial_color = traci.vehicle.getColor(ego);
        traci.vehicle.setSpeedMode(ego, 0);
        double next_vel = unif(re);
        traci.vehicle.setSpeed(ego, next_vel); // let the ego car start from 1-5
        is_initialized = true;
        is_initial = true;
        std::cout << "SUMO: Command_v: " << next_vel << std::endl;
        traci.simulationStep();
      }

      if (is_initialized && !FindVehicle(traci.vehicle.getIDList(), ego)) {
        std::cout << "------- The world is so fresh! -------" << std::endl;
        sp.Reset();
        traci.vehicle.add(ego, "route01", "CarA", "-1");
        traci.vehicle.moveToXY(ego, std::string(""), 0, initial_x, initial_y, initial_angle, 2);
        traci.vehicle.setColor(ego, initial_color);
        traci.vehicle.setSpeedMode(ego, 0);
        double next_vel = unif(re);
        traci.vehicle.setSpeed(ego, next_vel); // let the ego car start from 1-5
        is_initial = true;
        std::cout << "SUMO: Command_v: " << next_vel << std::endl;
        traci.simulationStep();
      } else if (is_initial || elapsed > planning_cycle) {
        //std::cout << "Do Planning " << elapsed << std::endl;
        previous_time = traci.simulation.getTime();
        is_initial = false;

        // next traffic lights
        std::cout << "Next Traffic Light " << std::endl;
        std::vector<libsumo::TraCINextTLSData> next_tls = traci.vehicle.getNextTLS(ego);
        // for(int i = 0; i < next_tls.size(); i++) {
        //   std::cout << "tl: " << i << std::endl;
        //   std::cout << next_tls[i].id << ", " 
        //             << next_tls[i].tlIndex << ", "
        //             << next_tls[i].state << ", "
        //             << next_tls[i].dist << std::endl;
        // }
        if(!next_tls.empty()) {
          char tf_state = next_tls[0].state;
          switch(tf_state) {
            case 'r':
            case 'R':
              sp.SetTrafficLightStatus(TrafficLightStatus::RED);
              break;
            case 'y':
            case 'Y':
              sp.SetTrafficLightStatus(TrafficLightStatus::YELLOW);
              break;
            case 'g':
            case 'G':
              sp.SetTrafficLightStatus(TrafficLightStatus::GREEN);
              break;
            default:
              sp.SetTrafficLightStatus(TrafficLightStatus::NA);
              break;
          }
        }

        // assign speed value with time
        double ego_x = traci.vehicle.getPosition(ego).x;
        double ego_y = traci.vehicle.getPosition(ego).y;
        double ego_speed = traci.vehicle.getSpeed(ego);
        int closest_idx = closestPoint(ego_path, ego_x, ego_y);
        double ego_angle = atan2(ego_path[closest_idx+1].y - ego_path[closest_idx].y,
                                 ego_path[closest_idx+1].x - ego_path[closest_idx].x);
        while(ego_angle > 180) ego_angle -= 360;
        while(ego_angle < -180) ego_angle += 360;

        // convert front bumper point to center point
        ego_x = ego_x - 2.5 * cos(ego_angle);
        ego_y = ego_y - 2.5 * sin(ego_angle);

        std::vector<Obstacle> perception_obstacles;

        // push in perception obstacles
        for (std::string vehicle : traci.vehicle.getIDList()) {
          if (vehicle != ego) {
            // deceived an vehicle by perception module
            //std::cout << "-------" << vehicle << "-------" << std::endl;
            double veh_x = traci.vehicle.getPosition(vehicle).x;
            double veh_y = traci.vehicle.getPosition(vehicle).y;
            double veh_speed = traci.vehicle.getSpeed(vehicle);

            Point2D veh_p(veh_x, veh_y);
            std::vector<Point2D> veh_path = sp.GetRoadLineByPoint(veh_p);
            int veh_closest_idx = closestPoint(veh_path, veh_x, veh_y);
            double veh_angle = atan2(veh_path[veh_closest_idx+1].y - veh_path[veh_closest_idx].y,
                                     veh_path[veh_closest_idx+1].x - veh_path[veh_closest_idx].x);

            if(ID == ScenarioID::ENTER_ROAD || ID == ScenarioID::EXIT_ROAD) {
              traci.vehicle.setSpeedMode(vehicle, 0);
              traci.vehicle.setSpeed(vehicle, veh_speed);
            } else if (ID == ScenarioID::CROSS_WALK) {
              // release control to sumo
              traci.vehicle.setSpeedMode(vehicle, 0);
            } else if (ID == ScenarioID::LEFT_TURN) {
              // release control to sumo
            } else if (ID == ScenarioID::RIGHT_TURN) {
              std::cout << "ego lane: " << traci.vehicle.getLaneID(ego) << std::endl;
              std::cout << "veh lane: " << traci.vehicle.getLaneID(vehicle) << std::endl;
              if( traci.vehicle.getLaneID(vehicle) == "SC_1"
                || traci.vehicle.getLaneID(vehicle) == "CE_1") {
                // release control to sumo
                std::cout << "release control to sumo" << std::endl;
                traci.vehicle.setSpeedMode(vehicle, -1);
              } else {
                traci.vehicle.setSpeedMode(vehicle, 0);
                traci.vehicle.setSpeed(vehicle, std::max(veh_speed, 8.0));
              }
            }

            // construct perception obstacles
            // convert front bumper point to center point
            veh_x = veh_x - 2.5 * cos(veh_angle);
            veh_y = veh_y - 2.5 * sin(veh_angle);

            // dummy prediction
            Frenet veh_path_frenet(veh_path);
            Point_Frenet start_sd;
            veh_path_frenet.ToFrenet(veh_p, veh_closest_idx, start_sd);
            std::vector<TrajectoryPoint> pred_traj;
            TrajectoryPoint traj_pt(veh_x, veh_y, veh_speed, 0.0);
            pred_traj.push_back(traj_pt);
            for(int i = veh_closest_idx+1; i < veh_path.size(); i++) {
              Point_Frenet curr_sd;
              veh_path_frenet.ToFrenet(veh_p, i, curr_sd);              
              double relative_time = (curr_sd.s - start_sd.s)/veh_speed;
              TrajectoryPoint traj_pt(veh_path[i].x, veh_path[i].y, veh_speed, relative_time);
              pred_traj.push_back(traj_pt);
            }
            Obstacle perception_obstacle(ObstacleType::CAR, pred_traj);
            perception_obstacles.push_back(perception_obstacle);
          }
        }

        // process pedestrians
        for (std::string person : traci.person.getIDList()) {
          if (traci.person.getStage(person) != 2) continue;
          double ped_x = traci.person.getPosition(person).x;
          double ped_y = traci.person.getPosition(person).y;
          double ped_speed = traci.person.getSpeed(person);

          if(!entered_peds.count(person)) {
            std::uniform_real_distribution<double> dist(1.0, 5.0);
            ped_speed = dist(mt);
            traci.person.setSpeed(person, ped_speed);
            entered_peds.insert(person);
          }

           // construct perception obstacles
          Point2D ped_p(ped_x, ped_y);
          double ped_angle = 0;
          if(ped_prev_positions.count(person)) {
            Point2D prev_pos = ped_prev_positions[person];
            ped_angle = atan2(ped_y - prev_pos.y, ped_x - prev_pos.x) * 180 / M_PI;
          }
          while(ped_angle > 180) ped_angle -= 360;
          while(ped_angle < -180) ped_angle += 360;
          ped_prev_positions[person] = ped_p;

          std::vector<Point2D> ped_path = sp.GetWalkLineByPoint(ped_p);

          if(!ped_path.empty()) {
            int p_idx = closestPoint(ped_path, ped_x, ped_y);
            double path_angle = atan2(ped_path[p_idx+1].y - ped_path[p_idx].y, ped_path[p_idx+1].x - ped_path[p_idx].x) * 180 / M_PI;
            while(path_angle > 180) path_angle -= 360;
            while(path_angle < -180) path_angle += 360;
            if(abs( abs(path_angle - ped_angle) - 180) < 0.01 ) {
              std::cout << "ped " << person << " reversed!" << std::endl;
              std:reverse(ped_path.begin(), ped_path.end());
              p_idx = closestPoint(ped_path, ped_x, ped_y);
            }
            // dummy prediction
            Frenet ped_path_frenet(ped_path);
            Point_Frenet start_sd;
            ped_path_frenet.ToFrenet(ped_p, p_idx, start_sd);
            std::vector<TrajectoryPoint> pred_traj;
            TrajectoryPoint traj_pt(ped_x, ped_y, ped_speed, 0.0);
            pred_traj.push_back(traj_pt);
            for(int i = p_idx+1; i < ped_path.size(); i++) {
              Point_Frenet curr_sd;
              Point2D curr_xy;
              ped_path_frenet.ToFrenet(ped_path[i], i, curr_sd);
              ped_path_frenet.ToCartesian(Point_Frenet(curr_sd.s, start_sd.d), curr_xy);
              double relative_time = (curr_sd.s - start_sd.s)/ped_speed;
              TrajectoryPoint traj_pt(curr_xy.x, curr_xy.y, ped_speed, relative_time);
              pred_traj.push_back(traj_pt);
            }
            Obstacle perception_obstacle(ObstacleType::PEDESTRIAN, pred_traj);
            perception_obstacles.push_back(perception_obstacle);
          }
        }

        // populate ego trajectory which is assumed to be given by the path
        // planning module
        Frenet ego_path_frenet(ego_path);
        Point_Frenet start_sd;
        int idx = closestPoint(ego_path, ego_x, ego_y);
        ego_path_frenet.ToFrenet(Point2D(ego_x, ego_y), idx, start_sd);
        std::vector<TrajectoryPoint> ref_trajectory;
        for (double s = 0.0; s < 500; s += 0.5) {
          double d = 0;
          Point_Frenet sd(start_sd.s + s, d);
          Point2D xy;
          ego_path_frenet.ToCartesian(sd, xy);

          // construct each trajectory point
          TrajectoryPoint p;
          p.x = xy.x;
          p.y = xy.y;
          if (s < 0.5) p.v = ego_speed;
          p.relative_time = 0.0;
          p.v_limit = s_and_v(start_sd.s + s);
          ref_trajectory.push_back(p);
        }

        Status st = sp.Plan(perception_obstacles, ref_trajectory);
        double next_vel = sp.NextVelocityCommand();
        traci.vehicle.setSpeed(ego, next_vel);
        std::cout << "SUMO: Command_v: " << next_vel << std::endl;
      } else {
        double next_vel = sp.NextVelocityCommand();
        traci.vehicle.setSpeed(ego, next_vel);
        std::cout << "SUMO: Command_v: " << next_vel << std::endl;
      }

      traci.simulationStep();
      elapsed = traci.simulation.getTime() - previous_time;
    }
  }

  std::cout << "time in ms: " << traci.simulation.getCurrentTime() << "\n";
  traci.close();

  return 1;
}
