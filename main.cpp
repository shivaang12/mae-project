#include <SDL.h>
#include <vector>
#include <utility>
#include <cmath>
#include <iostream>
#include "sdl_wrapper.hpp"
#include "obstacle.hpp"
#include "draw.hpp"
#include "astar.hpp"
#include "robot.hpp"
#include "group.hpp"
#include "controller.hpp"

double calculate_norm(double first_var, double second_var) {
    return std::sqrt(std::pow(first_var, 2) + std::pow(second_var, 2));
}

std::pair<double, double> vector_field(
    const std::pair<float, float> &B_vector,
    const std::pair<int, int> &currentPos,
    const std::vector<std::pair<int, int> > &obstacle_info) {
    double obstacle_radius = 4;
    double re = 6;
    double r = obstacle_radius + re + 10;
    double BF = std::pow(obstacle_radius, 2) - std::pow(r, 2);
    double BZ = std::pow(obstacle_radius, 2) - std::pow(r - 10, 2);

    double Fx = 0;
    double Fy = 0;

    // Debug
    double EPSILON = 0.000001;

    double sigma_value = 1.0;
    for (auto &xyo : obstacle_info) {
        double dx = static_cast<double>(currentPos.first - xyo.first);  //> x axis distance between obstacle and robot
        double dy = static_cast<double>(currentPos.second - xyo.second);  //> y axis distance between obstacle and robot

        double B =
            std::pow(obstacle_radius, 2) - std::pow(dx, 2) - std::pow(dy, 2);

        double sigma = 0.0;

        if ((B <= BF) || (B > 0)) {
            sigma = 1;
        } else if ((B > BF && B < BZ)) {
            sigma = 1 - std::abs((B - BF) / (BF - BZ));
        } else {
            sigma = 0;
        }

        float a = B_vector.first * dx + B_vector.second * dy;

        if (a >= 0) {
            auto Fdenom = calculate_norm(
                      (B_vector.second * dx * dy) - (B_vector.first * dy * dy),
                      (B_vector.second * dx * dy) - (B_vector.first * dx * dx));

            if (std::abs(Fdenom) < EPSILON)
            {
                // Do nothing
            } else {
                Fx += ((1 - sigma) *
                   ((B_vector.second * dx * dy) - (B_vector.first * dy * dy))) / Fdenom;
            Fy += (1 - sigma) *
                  ((B_vector.second * dx * dy) - (B_vector.first * dx * dx)) / Fdenom;
            }
        }

        if (a < 0) {
            double Fdenom = calculate_norm(
                    (-B_vector.first * dx * dx) - (B_vector.first * dy * dy),
                    (-B_vector.second * dy * dy) - (B_vector.second * dx * dx));
            
            if (std::abs(Fdenom) < EPSILON)
            {
                // Do nothing
            } else {
                Fx +=
                ((1 - sigma) *
                 ((-B_vector.first * dx * dx) - (B_vector.first * dy * dy))) / Fdenom;
            Fy +=
                ((1 - sigma) *
                 ((-B_vector.second * dy * dy) - (B_vector.second * dx * dx))) / Fdenom;
            }
        }

        if (sigma < 1) {
            int bbb = 0;
        }

        sigma_value *= sigma;
    }

    // std::cout << sigma_value * B_vector.first << " " << Fx << " " << Fy << '\n';

    return std::make_pair(Fx - (sigma_value * B_vector.first),
                          Fy - (sigma_value * B_vector.second));
}

int main(int argc, char * args[]) {
    // Screen Dims Constants
    const int SCREEN_WIDTH = 640;
    const int SCREEN_HEIGHT = 480;

    bool isquit = false;

    SDLWrapper sdl_obj(SCREEN_WIDTH, SCREEN_HEIGHT);

    // Contructing Tunnel and Accessing Shared Pointer
    auto obstacle_layer_object = Obstacle(SCREEN_WIDTH, SCREEN_HEIGHT);
    obstacle_layer_object.constructTunnel();
    auto obstacle_layer_shr_ptr = obstacle_layer_object.getObstacleLayer();
    auto obstacle_points_list = obstacle_layer_object.getObstacleList();

    // Constructing Live Obstacle
    auto live_obstacle = LiveObstacle(6, 6);
    live_obstacle.configure(395, 240, 0.5, 0);
    auto live_obstacle_points = live_obstacle.getObtaclePoints();

    // Const Draw Object and Drawing the tunnel
    auto draw_obj = Draw(SCREEN_WIDTH, SCREEN_HEIGHT);
    draw_obj.DrawPointsUsingLayer(obstacle_layer_shr_ptr, sdl_obj);

    // Const Astar object and make a path
    auto astar_obj = Astar();
    astar_obj.initialize(SCREEN_WIDTH, SCREEN_HEIGHT, 22, 5);
    astar_obj.loadObstacleInfo(obstacle_layer_shr_ptr);

    astar_obj.setGoalPoint(370,240);
    astar_obj.setStartPoint(250,140);
    // astar_obj.setGoalPoint(80,80);
    // astar_obj.setStartPoint(40,40);
    std::cout << "Starting .." << '\n';
    std::cout << "Making plan .." << '\n';
    auto path = astar_obj.makePlanCoordinate();

    // Robot
    Group grp(250, 140, 4, 22);
    auto robots = grp.getRobotVector();

    // Controller
    Controller ctrl;
    ctrl.register_path(path);

    for(const auto &item : robots)
    {
        draw_obj.DrawPointRobot(item, sdl_obj);
    }

    
    draw_obj.DrawPointsFromCoords(live_obstacle_points, sdl_obj);
    sdl_obj.show();

    int index = 1;



    //Wait two seconds
    while (!isquit) {
        isquit = sdl_obj.check_poll_event();

        live_obstacle.next();
        live_obstacle_points  = live_obstacle.getObtaclePoints();

        // ******OBSTACLE********
        std::vector<std::pair<int, int>> obstacle_list;
        obstacle_list.reserve(obstacle_points_list->size() + live_obstacle_points.size());
        obstacle_list.insert(obstacle_list.end(), obstacle_points_list->begin(), obstacle_points_list->end());
        obstacle_list.insert(obstacle_list.end(), live_obstacle_points.begin(), live_obstacle_points.end());

        //
        auto change_vectors = ctrl.getRobotPose(grp);
        std::vector<std::pair<double, double>> obstacle_change_vector;
        auto robots = grp.getRobotVector();

        for(int i=0; i<grp.getNumberOfRobots(); i++)
        {
            auto change_vec_neg = change_vectors[i];
            if ((change_vec_neg.first < 0.0001) && (change_vec_neg.second < 0.0001))
            {
                obstacle_change_vector.emplace_back(change_vec_neg.first, change_vec_neg.second);
            }
            else
            {
                change_vec_neg.first = - change_vec_neg.first;
                change_vec_neg.second = - change_vec_neg.second;
                auto obst_change_vec = vector_field(change_vec_neg, robots[i], obstacle_list);
                std::cout << obst_change_vec.first << " " << obst_change_vec.second << '\n';
                std::pair<double, double> total_vec = {0.0, 0.0};
                total_vec.first = (obst_change_vec.first/calculate_norm(obst_change_vec.first, obst_change_vec.second));
                total_vec.second = (obst_change_vec.second/calculate_norm(obst_change_vec.first, obst_change_vec.second));
                obstacle_change_vector.emplace_back(total_vec);
            }
        }

        grp.updateRobotPosition(obstacle_change_vector);
        // grp.updateRobotPosition(change_vectors);
        robots = grp.getRobotVector();

        sdl_obj.clear();
        draw_obj.DrawPointsUsingLayer(obstacle_layer_shr_ptr, sdl_obj);

        for(const auto &item:robots)
        {
            // std::cout << item.first << " " << item.second << '\t';
            draw_obj.DrawPointRobot(item, sdl_obj);
        }

        // std::cout << '\n';

        auto circle_info = grp.getCircleInfo();
        auto new_circle_info = std::make_tuple(static_cast<int>(std::get<0>(circle_info)),
                                            static_cast<int>(std::get<1>(circle_info)),
                                            static_cast<int>(std::get<2>(circle_info)));
        draw_obj.DrawCircle(new_circle_info, sdl_obj);
        draw_obj.DrawPointsFromCoords(live_obstacle_points, sdl_obj);

        sdl_obj.show();
        sdl_obj.delay(200);

        // if (index < (path.size() - 1))
        // {
        //     auto item_previous = path[index-1];
        //     std::pair<int, int> current_pose = {std::get<0>(item_previous), std::get<1>(item_previous)};
        //     auto item_current = path[index];
        //     auto circle_obst_list = obstacle_layer_object.getCirclePoints(std::get<0>(item_previous),
        //         std::get<1>(item_previous), std::get<2>(item_previous));

        //     // Vector Field Variables
        //     live_obstacle.next();
        //     live_obstacle_points  = live_obstacle.getObtaclePoints();

        //     std::vector<std::pair<int, int>> obstacle_list;
        //     obstacle_list.reserve(circle_obst_list.size() + live_obstacle_points.size());
        //     obstacle_list.insert(obstacle_list.end(), circle_obst_list.begin(), circle_obst_list.end());
        //     obstacle_list.insert(obstacle_list.end(), live_obstacle_points.begin(), live_obstacle_points.end());

        //     std::pair<double, double> direction_vector = {std::get<0>(item_current) - std::get<0>(item_previous), 
        //                                                   std::get<1>(item_current) - std::get<1>(item_previous)};
        //     direction_vector = {-0.5, 0};
        //     current_pose = robot_1.getPoseByPair();
        //     // direction_vector.first = (direction_vector.first/calculate_norm(direction_vector.first, direction_vector.second));
        //     // direction_vector.second = (direction_vector.second/calculate_norm(direction_vector.first, direction_vector.second));
        //     // std::cout << direction_vector.first << " " << direction_vector.second << '\n';
        //     // std::cout << direction_vector.first << " " << direction_vector.second << '\n';
        //     auto change_ = vector_field(direction_vector, current_pose, obstacle_list);
        //     // std::cout << change_.first << " " << change_.second << '\n';

        //     double total_vec_x, total_vec_y;
        //     total_vec_x = (change_.first/calculate_norm(change_.first, change_.second));
        //     total_vec_y = (change_.second/calculate_norm(change_.first, change_.second));

        //     robot_1.change_pose_by(total_vec_x, total_vec_y);
        //     // robot_1.print_robot_path();

        //     std::cout << total_vec_x << " " << total_vec_y << '\n';
            

        //     // Drawing ...
        //     sdl_obj.clear();
        //     draw_obj.DrawPointsUsingLayer(obstacle_layer_shr_ptr, sdl_obj);
        //     // draw_obj.DrawCircle(item_previous, sdl_obj);
        //     draw_obj.DrawPointsFromCoords(live_obstacle_points, sdl_obj);
        //     draw_obj.DrawPointRobot(robot_1.getPoseByPair(), sdl_obj);

        //     sdl_obj.show();
        //     sdl_obj.delay(50);
        //     ++index;
        // }

    }

    return 0;
}
