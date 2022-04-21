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


// 3-Dimentional hypot
double hypot3(double x, double y, double z)
{
    return std::sqrt(x*x + y*y + z*z);
} 

class Astar
{
public:
    /**
     * @brief A constructor for the class planner::Astar
     */
    Astar() : width_(0),
              height_(0),
              max_radius_(0),
              radius_length_(0),
              min_radius_(0),
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
                    const int height,
                    const int max_radius,
                    const int radius_length)
    {
        width_ = width;
        height_ = height;
        max_radius_ = max_radius;
        radius_length_ = radius_length;
        min_radius_ = max_radius - radius_length;
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
        // Temporary: Goal Z will always be max_radius
        int start_z = this->max_radius_;

        convertCoordinateToCell(start_x, start_y, start_z, start_);
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
        // Temporary: Goal Z will always be max_radius
        int goal_z = this->max_radius_;

        convertCoordinateToCell(goal_x, goal_y, goal_z, goal_);
        std::cout << "[ASTAR] GOAL is " << goal_ << '\n';
    }

    /**
     * @brief Generates plan from given start and goal points
     */
    // TODO: Change
    std::vector<int> makePlan()
    {
        std::vector<int> path;
        std::cout << "[DEBUG] Check 0" << '\n';

        // Checking if goal and start is valid
        if (obstacle_info_ptr_)
        {
            if (!isValid(start_) || !isValid(goal_))
            {
                return path;
            }
        }

        std::cout << "[DEBUG] Check 1" << '\n';

        // Open List
        std::priority_queue<
            std::pair<int, double>,
            std::vector<std::pair<int, double>>,
            comparator>
            open_list;
        open_list.emplace(start_, 0);

        std::cout << "[DEBUG] Check 2" << '\n';

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
    std::vector<std::tuple<int, int, int> > makePlanCoordinate() {
        std::vector<std::tuple<int, int, int> > return_vector;
        for (auto& cell : makePlan()) {
            int x, y, z;
            convertCellToCoordinate(cell, x, y, z);
            std::cout << "[DEBUG] " << x << " " << y << " " << z << '\n';
            std::cout << "[] " << cell << '\n';
            return_vector.emplace_back(x, y, z);
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
        int cell1_x, cell1_y, cell1_z;
        int cell2_x, cell2_y, cell2_z;

        convertCellToCoordinate(cell1, cell1_x, cell1_y, cell1_z);
        convertCellToCoordinate(cell2, cell2_x, cell2_y, cell2_z);

        return hypot3(static_cast<double>(cell1_x - cell2_x),
                      static_cast<double>(cell1_y - cell2_y),
                      static_cast<double>(cell1_z - cell2_z));
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
        int cell_x, cell_y, cell_z;
        int goal_x, goal_y, goal_z;

        convertCellToCoordinate(cell, cell_x, cell_y, cell_z);
        convertCellToCoordinate(goal_, goal_x, goal_y, goal_z);

        return hypot3(static_cast<double>(cell_x - goal_x),
                      static_cast<double>(cell_y - goal_y),
                      static_cast<double>(cell_z - goal_z));
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
        // std::cout << "[DEBUG] isValid 1" << '\n';
        // Since the cell is 3D-hash version we will convert it to 2D-hash
        int x, y, z, cell2d;
        this->convertCellToCoordinate(cell, x, y, z);
        // std::cout << "[DEBUG] Cell " << x << " " << y << " " << z << '\n';
        cell2d = (y * width_) + x;

        // Now checking for obstacles
        // We will check in the square of size radius
        if (obstacle_info_ptr_)
        {
            for(int i=x-z; i<x+z; i++)
            {
                for(int j=y-z; j<y+z; j++)
                {
                    if(std::hypot(i-x, j-y) <= z)
                    {
                        // std::cout << " INSIDE " << '\n';
                        if ((i < 0) || (i >= width_) || (j < 0) || (j >= height_))
                        {
                            return false;
                        }
                        if ((*obstacle_info_ptr_)[(j * width_) + i] == 1)
                        {
                            return false;
                        }
                    }
                }
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
        int current_x, current_y, current_z;

        const std::array<int, 3> state_change = {1, 0, -1};

        convertCellToCoordinate(current_cell, current_x, current_y, current_z);
        std::vector<int> return_vector;
        return_vector.reserve(26);

        for (int i : state_change)
        {
            // Corner case for width (x)
            if((current_x + i < 0) || (current_x + i >= width_))
            {
                continue;
            }

            for (int j : state_change)
            {
                // Corner case for height (y)
                if((current_y + j < 0) || (current_y + j >= height_))
                {
                    continue;
                }
                
                for (int k : state_change)
                {
                    // Corner case for circle (z)
                    if((current_z + k < min_radius_) || (current_z + k > max_radius_))
                    {
                        continue;
                    }

                    // Make sure this cell is not as current_cell
                    int current_test_cell = -1;
                    this->convertCoordinateToCell(current_x+i, current_y+j, current_z+k, current_test_cell);

                    if (current_test_cell == current_cell)
                    {
                        continue;
                    }

                    if ((current_test_cell >= 0) && isValid(current_test_cell))
                    {
                        return_vector.emplace_back(current_test_cell);
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
                                 const int &z,
                                 int &cell) const
    {
        cell = (z * height_ * width_) + (y * width_) + x;
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
                                 int &y,
                                 int &z) const
    {
        z = cell / (height_ * width_);
        int new_cell = cell - (z * height_ * width_);
        y = new_cell / width_;
        x = new_cell % width_;
    }
    /**
     * @brief Getter method gets start point
     *
     * @param[out] start_x X coordinate for start point
     * @param[out] start_y Y coordinate for start point
     */
    void getStartPoint(int &start_x, int &start_y, int start_z) const
    {
        convertCellToCoordinate(start_, start_x, start_y, start_z);
    }
    /**
     * @brief Getter method gets goal point
     *
     * @param[out] goal_x X coordinate for goal point
     * @param[out] goal_y Y coordinate for goal point
     */
    void getGoalPoint(int &goal_x, int &goal_y, int &goal_z) const
    {
        convertCellToCoordinate(goal_, goal_x, goal_y, goal_z);
    }

private:
    int width_;
    int height_;

    int max_radius_;
    int radius_length_;
    int min_radius_;

    int start_;
    int goal_;
    bool obstacle_flag;
    std::shared_ptr<std::vector<int>> obstacle_info_ptr_;
};

#endif // ASTAR_INCLUDE_ASTAR_ASTAR_CPP