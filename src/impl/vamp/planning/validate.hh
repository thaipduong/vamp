#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/poly.hh>

namespace vamp::planning
{
    template <std::size_t n, std::size_t... I>
    inline constexpr auto generate_percents(std::index_sequence<I...>) -> std::array<float, n>
    {
        return {(static_cast<void>(I), static_cast<float>(I + 1) / static_cast<float>(n))...};
    }

    template <std::size_t n>
    struct Percents
    {
        inline static constexpr auto percents = generate_percents<n>(std::make_index_sequence<n>());
    };

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_vector(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &vector,
        float distance,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
        const auto percents = FloatVector<rake>(Percents<rake>::percents);

        // Vector: num of rows = DOF, num of scalar per row = rake 
        typename Robot::template ConfigurationBlock<rake> block;

        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = start.broadcast(i) + (vector.broadcast(i) * percents);
        }

        // n is the number of points per rake
        //  resolution is the number of point per a unit of distance
        const std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);

        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                 Robot::template fkcc<rake>(environment, block);
        if (not valid or n == 1)
        {
            return valid;
        }

        const auto backstep = vector / (rake * n); // = vector / (distance * resolution) = (vector/distance)/resolution, vector/distance is a unit vector
        for (auto i = 1U; i < n; ++i)
        {
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                block[j] = block[j] - backstep.broadcast(j);
            }

            if (not Robot::template fkcc<rake>(environment, block))
            {
                return false;
            }
        }

        return true;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_poly(
        Polynomial<Robot::flat_dimension> &traj,
        float T,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
        const auto sampling_times = T*FloatVector<rake>(Percents<rake>::percents);

        // Vector: num of rows = DOF, num of scalar per row = rake 
        typename Robot::template ConfigurationBlock<rake> block;

        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        // What does this broadcast function do?
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = traj.eval_rake(i, sampling_times);
        }

        // n is the number of points per rake
        //  resolution is the number of point per a unit of distance
        const std::size_t n = std::max(std::ceil(T / static_cast<float>(rake) * resolution), 1.F);

        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                 Robot::template fkcc<rake>(environment, block);
        if (not valid or n == 1)
        {
            return valid;
        }

        // This backstep is in time now.
        const auto backstep = T / static_cast<float>(rake * n); // = vector / (distance * resolution) = (vector/distance)/resolution, vector/distance is a unit vector
        for (auto i = 1U; i < n; ++i)
        {
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                block[j] = traj.eval_rake(j, sampling_times -  i*backstep);
            }

            if (not Robot::template fkcc<rake>(environment, block))
            {
                return false;
            }
        }

        return true;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        auto vector = goal - start;
        auto ret = validate_vector<Robot, rake, resolution>(start, vector, vector.l2_norm(), environment);
        return  ret;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_poly_motion(
        const typename Robot::ConfigurationFlatStateVecArray &start,
        const typename Robot::ConfigurationFlatStateVecArray &goal,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        const float T = 1.5;
        auto traj = opt_traj<Robot::flat_dimension>(start[0], start[1], goal[0], goal[1], T);
        return validate_poly<Robot, rake, Robot::resolution>(traj, T, environment);
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_poly_motion(
        const typename Robot::ConfigurationFlatState &start,
        const typename Robot::ConfigurationFlatState &goal,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        auto flat_start = Robot::flatstate_to_vecarray(start);
        auto flat_goal = Robot::flatstate_to_vecarray(goal);
        return validate_poly_motion(flat_start, flat_goal, environment);
    }



}  // namespace vamp::planning
