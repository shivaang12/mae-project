#ifndef ASTAR_INCLUDE_ASTAR_ASTAR_CPP
#define ASTAR_INCLUDE_ASTAR_ASTAR_CPP
#include <vector>
#include <memory>
#include <iostream>
#include <cmath>
#include <utility>
#include <unordered_map>
#include <limits>
#include <algorithm>
#include <queue>


struct comparator {
  /**
   * @brief Operator() overload to enable sorting for priority queue
   *
   * @param[in] lhs Refers to left most param
   * @param[in] rhs Refers to right most param
   */
  bool operator()(const std::pair<int, double>& lhs, const std::pair<int, double>& rhs) {
    return lhs.second > rhs.second;
  }
};

class Astar
{
public:
    /**
     * @brief A constructor for the class planner::Astar
     */
    Astar() : width_(0),
              height_(0),
              start_(0),
              goal_(0),
              obstacle_flag(0),
              obstacle_info_ptr_(NULL) {}

    /**
     * @brief Function to initialize member variables
     *
     * @param[in] width Width of the area
     * @param[in] height height of the area
     *
     * @return Void
     */
    void initialize(const int width,
                    const int height)
    {
        width_ = width;
        height_ = height;
    }

    /**
     * @brief Setter function which sets start position for planner
     *
     * @param[in] start_x Start position's x coordinate
     * @param[in] start_y Start position's y coordinate
     *
     * @return Void
     */
    void setStartPoint(const int start_x,
                       const int start_y)
    {
        convertCoordinateToCell(start_x, start_y, start_);
    }

    /**
     * @brief Setter function which sets goal position for planner
     *
     * @param[in] goal_x Goal position's x coordinate
     * @param[in] goal_y Goal position's y coordinate
     *
     * @return Void
     */
    void setGoalPoint(const int goal_x,
                      const int goal_y)
    {
        convertCoordinateToCell(goal_x, goal_y, goal_);
        std::cout << "[ASTAR] GOAL is " << goal_ << '\n';
    }

    /**
     * @brief Generates plan from given start and goal points
     */
    std::vector<int> makePlan()
    {
        std::vector<int> path;

        // Checking if goal and start is valid
        if (obstacle_info_ptr_)
        {
            if (!isValid(start_) || !isValid(goal_))
            {
                return path;
            }
        }

        // Open List
        std::priority_queue<
            std::pair<int, double>,
            std::vector<std::pair<int, double>>,
            comparator>
            open_list;
        open_list.emplace(start_, 0);

        // Dictionary
        std::unordered_map<int, int> parent_dict;
        parent_dict[start_] = start_;

        // cost to come (g cost)
        std::unordered_map<int, double> gCost;
        gCost[start_] = 0.0;

        // exhaustive loop
        while (!open_list.empty())
        {
            auto current_cell = open_list.top();
            open_list.pop();

            if (current_cell.first == goal_)
            {
                break;
            }

            for (auto &neighbor : getNeighbors(current_cell.first))
            {
                double new_g_cost = gCost[current_cell.first] +
                                    calcGCost(neighbor, current_cell.first) + getHCost(neighbor);

                if ((gCost.find(neighbor) == gCost.end()) || (new_g_cost < gCost[neighbor]))
                {
                    gCost[neighbor] = new_g_cost;
                    double f_cost = new_g_cost + getHCost(neighbor);
                    open_list.emplace(neighbor, f_cost);
                    parent_dict[neighbor] = current_cell.first;
                }
            }
        }

        // Building path
        int current_cell = goal_;
        path.emplace_back(goal_);

        while (current_cell != start_)
        {
            current_cell = parent_dict[current_cell];
            path.emplace_back(current_cell);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    /**
     * @brief This method loads obstacle information
     *
     * @param[in] obstacle_info Shared pointer to vector of bool which contains obstacle infomation
     */
    void loadObstacleInfo(
        const std::shared_ptr<std::vector<int>> obstacle_info)
    {
        obstacle_info_ptr_ = obstacle_info;
    }

    /**
     * @brief This method converts cells into x-y coordinates and return vector of pair
     *
     * @return Vector of pairs containing x-y coordinates of the plan
     */
    std::vector<std::pair<int, int>> makePlanCoordinate() {
        std::vector<std::pair<int, int> > return_vector;

        for (auto& cell : makePlan()) {
            int x, y;
            convertCellToCoordinate(cell, x, y);
            return_vector.emplace_back(x, y);
        }
        return return_vector;
    }

    /**
     * @brief Method calculate cost-to-come (G) for Astar
     *
     * @param[in] cell1 Cell
     * @param[in] cell2 Cell
     *
     * @return cost-to-come in double
     */
    double calcGCost(const int &cell1,
                     const int &cell2) const
    {
        int cell1_x, cell1_y;
        int cell2_x, cell2_y;

        convertCellToCoordinate(cell1, cell1_x, cell1_y);
        convertCellToCoordinate(cell2, cell2_x, cell2_y);

        return std::hypot(cell1_x - cell2_x, cell1_y - cell2_y);
    }

    /**
     * @brief Method calculates Heuristic Cost (H) for Astar
     *
     * @param[in] cell Cell
     *
     * @return Heuristic cost in double
     */
    double getHCost(const int &cell) const
    {
        int cell_x, cell_y;
        int goal_x, goal_y;

        convertCellToCoordinate(cell, cell_x, cell_y);
        convertCellToCoordinate(goal_, goal_x, goal_y);

        return std::hypot(cell_x - goal_x, cell_y - goal_y);
    }

    /**
     * @brief Method checks if the cell is not in obstacle
     *
     * @param[in] cell Cell
     *
     * @return True if cell is in obstacle else False
     */
    bool isValid(const int &cell) const
    {
        if (obstacle_info_ptr_)
        {
            if ((*obstacle_info_ptr_)[cell] == 1)
            {
                return false;
            } else {
                return true;
            }
        }
        return true;
    }

    /**
     * @brief Method retrives the neighbor cells for the given cells
     *
     * @param[in] current_cell Cell
     *
     * @return Vector of cell which are neighbor to current_cell
     */
    std::vector<int> getNeighbors(const int &current_cell) const
    {
        int current_x, current_y;

        convertCellToCoordinate(current_cell, current_x, current_y);
        std::vector<int> return_vector;
        return_vector.reserve(8);

        for (int i = current_cell - width_; i <= current_cell + width_; i = i + width_)
        {
            for (int j = -1; j <= 1; j++)
            {
                // Handle Corner cases
                // Right corner
                if (current_cell % width_ == width_ - 1)
                {
                    if (j > 0)
                    {
                        continue;
                    }
                }

                // Left corner
                if (current_cell % width_ == 0)
                {
                    if (j < 0)
                    {
                        continue;
                    }
                }

                if ((i + j >= 0) && (i + j < width_ * height_))
                {
                    if (isValid(i + j) && (current_cell != (i + j)))
                    {
                        return_vector.emplace_back(i + j);
                    }
                }
            }
        }

        return return_vector;
    }

    /**
     * @brief Converts x-y coordinate to cell
     *
     * @param[in] x x-coordinate
     * @param[in] y y-coordinate
     *
     * @param[out] cell Cell representing x-y coordinate
     */
    void convertCoordinateToCell(const int &x,
                                 const int &y,
                                 int &cell) const
    {
        cell = (y * width_) + x;
    }

    /**
     * @brief Coverts cell to x-y coordinate
     *
     * @param[in] cell Cell
     *
     * @param[out] x x-coordinate
     * @param[out] y y-coordinate
     */
    void convertCellToCoordinate(const int &cell,
                                 int &x,
                                 int &y) const
    {
        x = cell % width_;
        y = cell / width_;
    }
    /**
     * @brief Getter method gets start point
     *
     * @param[out] start_x X coordinate for start point
     * @param[out] start_y Y coordinate for start point
     */
    void getStartPoint(int &start_x, int &start_y) const
    {
        convertCellToCoordinate(start_, start_x, start_y);
    }
    /**
     * @brief Getter method gets goal point
     *
     * @param[out] goal_x X coordinate for goal point
     * @param[out] goal_y Y coordinate for goal point
     */
    void getGoalPoint(int &goal_x, int &goal_y) const
    {
        convertCellToCoordinate(goal_, goal_x, goal_y);
    }

private:
    int width_;
    int height_;

    int start_;
    int goal_;
    bool obstacle_flag;
    std::shared_ptr<std::vector<int>> obstacle_info_ptr_;
};

#endif // ASTAR_INCLUDE_ASTAR_ASTAR_CPP