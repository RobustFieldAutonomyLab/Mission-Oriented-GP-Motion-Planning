#include "GeneticAlgorithm.h"

int main() {
    constexpr int n = 11;
    std::vector<std::vector<int>> grid(n, std::vector<int>(n, 0));
    MakeGrid(grid);

    std::random_device rd;   // obtain a random number from hardware
    std::mt19937 eng(rd());  // seed the generator
    std::uniform_int_distribution<int> distr(0, n - 1);  // define the range

    Node start(distr(eng), distr(eng), 0, 0, 0, 0);
    Node goal(distr(eng), distr(eng), 0, 0, 0, 0);

    start.id_ = start.x_ * n + start.y_;
    start.pid_ = start.x_ * n + start.y_;
    goal.id_ = goal.x_ * n + goal.y_;
    start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);

    // Make sure start and goal are not obstacles and their ids are correctly
    // assigned.
    grid[start.x_][start.y_] = 0;
    grid[goal.x_][goal.y_] = 0;

    start.PrintStatus();
    goal.PrintStatus();

    PrintGrid(grid);

    constexpr int generations = 10000;
    constexpr int popsize = 30;
    constexpr double c = 1.05;
    constexpr bool shorten_chromosome = true;
    constexpr int path_length_x_factor = 4;

    GeneticAlgorithm new_genetic_algorithm(grid);
    new_genetic_algorithm.SetParams(
            generations, popsize, c, shorten_chromosome,
            static_cast<int>(path_length_x_factor * start.h_cost_));
    const auto [path_found, path_vector] =
            new_genetic_algorithm.Plan(start, goal);
    PrintPathInOrder(path_vector, start, goal, grid);
}
