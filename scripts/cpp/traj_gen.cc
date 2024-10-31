#include <vector>
#include <array>
#include <utility>
#include <iostream>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/poly.hh>

#include <vamp/robots/panda.hh>


using Robot = vamp::robots::Panda;
static constexpr std::size_t dimension = Robot::dimension;
using Configuration = Robot::Configuration;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

// Start and goal configurations
static constexpr std::array<float, dimension> start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
static constexpr std::array<float, dimension> goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};

// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {};

// Radius for obstacle spheres
static constexpr float radius = 0.2;

// Maximum planning time
static constexpr float planning_time = 1.0;

// Maximum simplification time
static constexpr float simplification_time = 1.0;

auto main(int argc, char **) -> int
{
    // // Build sphere cage environment
    // EnvironmentInput environment;
    // for (const auto &sphere : problem)
    // {
    //     environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    // }

    // environment.sort();
    // auto env_v = EnvironmentVector(environment);

    
    // // Get bounds from VAMP Robot information, scale 0/1 config to min/max
    // static constexpr std::array<float, dimension> zeros = {0., 0., 0., 0., 0., 0., 0.};
    // static constexpr std::array<float, dimension> ones = {1., 1., 1., 1., 1., 1., 1.};

    // auto zero_v = Configuration(zeros);
    // auto one_v = Configuration(ones);

    // Robot::scale_configuration(zero_v);
    // Robot::scale_configuration(one_v);
    std::cout << "Testttttttttttttt" << std::endl;
    std::vector<vamp::FloatVector<dimension>> coeffs;
    static constexpr std::array<float, dimension> coeff0 = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
    static constexpr std::array<float, dimension> coeff1 = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};
    static constexpr std::array<float, dimension> coeff2 = {1., 1., 1., 1., 1., 1., 1.};
    coeffs.emplace_back(vamp::FloatVector<dimension>(coeff0));
    coeffs.emplace_back(vamp::FloatVector<dimension>(coeff1));
    coeffs.emplace_back(vamp::FloatVector<dimension>(coeff2));
    vamp::planning::Polynomial<dimension> traj(coeffs, 2);
    std::cout << "traj(1.0) = " << traj.eval(1.0) << std::endl;
    auto traj_dev = traj.derivatives();
    std::cout << "traj_dev(1.0) = " << traj_dev.eval(1.0) << std::endl;
    auto traj_int = traj.integral();
    std::cout << "traj_int(1.0) = " << traj_int.eval(1.0) << std::endl;
    auto traj_squared = traj*traj;
    std::cout << "traj_squared(1.0) = " << traj_squared.eval(1.0) << std::endl;
    std::cout << "Testttttttttttttt Doneeeeeeeee" << std::endl;
    return 0;
}
