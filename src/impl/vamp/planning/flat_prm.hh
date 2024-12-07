#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <limits>
#include <memory>

#include <utility>
#include <vamp/collision/environment.hh>
#include <vamp/planning/nn.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/utils.hh>
#include <vamp/planning/roadmap.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/halton.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vector>

namespace vamp::planning
{

    template <
        typename Robot,
        typename RNG,
        std::size_t rake,
        std::size_t resolution,
        typename NeighborParamsT = PRMStarNeighborParams,
        std::size_t batch = 128>
    struct FlatPRM
    {
        static constexpr auto flat_dimension = Robot::flat_dimension;
        static constexpr auto flat_order = Robot::flat_order;
        static constexpr auto flatstate_dimension = Robot::flatstate_dimension;
        using ConfigurationFlat = typename Robot::ConfigurationFlat;
        using FlatState = typename Robot::ConfigurationFlatState;
        //using FlatStateFloatArray = typename Robot::ConfigurationFlatStateArray;
        //using FlatStateVecArray = typename Robot::ConfigurationFlatStateVecArray;


        inline static auto solve(
            const FlatState &start,
            const FlatState &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const RoadmapSettings<NeighborParamsT> &settings) noexcept -> PlanningResult<flatstate_dimension>
        {
            return solve(start, std::vector<FlatState>{goal}, environment, settings);
        }

        inline static auto solve(
            const FlatState &start,
            const std::vector<FlatState> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const RoadmapSettings<NeighborParamsT> &settings) noexcept -> PlanningResult<flatstate_dimension>
        {
            PlanningResult<flatstate_dimension> result;

            FlatNN<flat_dimension> roadmap;

            auto start_time = std::chrono::steady_clock::now();

            // We only need to double the state space from q to (q, q_dot) for: KD-Tree, Path, Trajectory Generation (Steering Fcn).
            // Maybe it is better to do that here in the planner instead of changing the robot class?
            // Check if the straight-line solution is valid
            auto flat_start = Robot::flatstate_to_vecarray(start);
            for (const auto &goal : goals)
            {   
                auto flat_goal = Robot::flatstate_to_vecarray(goal);
                // if (validate_motion<Robot, rake, resolution>(start, goal, environment))
                if (validate_poly_motion<Robot, rake, resolution>(flat_start, flat_goal, environment))
                {
                    result.path.emplace_back(start);
                    result.path.emplace_back(goal);
                    result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
                    result.iterations = 0;
                    result.size.emplace_back(1);
                    result.size.emplace_back(1);

                    return result;
                }
            }

            RNG rng;
            std::size_t iter = 0;
            std::vector<std::pair<NNFlatNode<flat_dimension, flat_order>, float>> neighbors;
            typename Robot::template ConfigurationBlock<rake> temp_block;
            auto states = std::unique_ptr<float>(
                vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                    settings.max_samples * FlatState::num_scalars_rounded));
            // TODO: Is it better to just use arrays for these since we're reserving full capacity
            // anyway? Test it!
            std::vector<RoadmapNode> nodes;
            nodes.reserve(settings.max_samples);
            std::vector<utils::ConnectedComponent> components;
            components.reserve(settings.max_samples);

            const auto state_index = [&states](unsigned int index) -> float *
            { return states.get() + index * FlatState::num_scalars_rounded; };

            // Add start and goal to structures
            constexpr const unsigned int start_index = 0;

            auto *start_state = state_index(start_index);
            start.to_array(start_state);
            nodes.emplace_back(start_index, start_index, 0.0);
            roadmap.insert(NNFlatNode<flat_dimension, flat_order>{start_index, {start_state}});
            components.emplace_back(utils::ConnectedComponent{start_index, 1});

            for (const auto &goal : goals)
            {
                std::size_t index = nodes.size();
                auto *goal_state = state_index(index);
                goal.to_array(goal_state);
                nodes.emplace_back(index, index);
                roadmap.insert(NNFlatNode<flat_dimension, flat_order>{index, {goal_state}});
                components.emplace_back(utils::ConnectedComponent{index, 1});
            }

            const std::size_t goal_max_index = nodes.size();

            while (iter++ < settings.max_iterations and nodes.size() < settings.max_samples)
            {
                auto temp = rng.next();
                
                Robot::scale_flatstate(temp);
                //auto sample = temp.reshape(Robot::flat_order, Robot::flat_dimension);
                // TODO: This is a gross hack to get around the instruction cache issue...I realized
                // that edge sampling, while valid, wastes too much effort with our current
                // validation API

                // Check sample validity, the first flat_dimension elements are the configuration q.
                // TODO: In general, we need to implement a function to convert the flat state to 
                // the configuration here. For manipulator, they happen to be the same.
                for (auto i = 0U; i < flat_dimension; ++i)
                {
                    temp_block[i] = temp.broadcast(i);
                }

                if (not Robot::template fkcc<rake>(environment, temp_block))
                {
                    continue;
                }

                // Insert valid state into graph structures
                auto *state = state_index(nodes.size());
                temp.to_array(state);
                auto &node = nodes.emplace_back(nodes.size(), std::numeric_limits<unsigned int>::max());

                // Add valid edges
                // TODO: Start a struct "FlatNeighbor" that converts two SimD vectors to one flat vector for the kd tree,
                // and converts a node in the kd tree back to 2 SimD vectors, one for config, one for vel.
                const auto k = settings.neighbor_params.max_neighbors(roadmap.size());
                const auto r = settings.neighbor_params.neighbor_radius(roadmap.size());
                auto state_node = NNFlatNode<flat_dimension, flat_order>{node.index, {state}};
                roadmap.nearest(neighbors, state_node.array, k, r);
                for (const auto &[neighbor, distance] : neighbors)
                {   
                    if (validate_poly_motion<Robot, rake, resolution>(neighbor.as_vector(), state_node.as_vector(),
                                                                      environment))
                    {
                        node.neighbors.emplace_back(typename RoadmapNode::Neighbor{
                            static_cast<unsigned int>(neighbor.index), distance});
                        nodes[neighbor.index].neighbors.emplace_back(
                            typename RoadmapNode::Neighbor{node.index, distance});
                    }
                }

                // Insert valid state into roadmap - after query to prevent returning self as
                // neighbor
                roadmap.insert(state_node);

                // Unify connected components
                if (node.neighbors.empty())
                {
                    node.component = components.size();
                    components.emplace_back(
                        utils::ConnectedComponent{static_cast<unsigned int>(components.size()), 1});
                }
                else
                {
                    node.component = nodes[node.neighbors.front().index].component;
                    for (const auto &neighbor : node.neighbors)
                    {
                        utils::merge_components(components, node.component, nodes[neighbor.index].component);
                    }
                }

                for (auto i = 1U; i < goal_max_index; ++i)
                {
                    // If the start and goal are in the same connected component, run A* to find the
                    // solution
                    if (utils::find_root(components, start_index) != utils::find_root(components, i))
                    {
                        continue;
                    }

                    const auto &goal = goals[i - 1];
                    auto parents = utils::astar(nodes, start, goal, state_index);
                    // NOTE: If the connected component check is correct, we can assume that a solution
                    // was found by A* when we've reached this point
                    utils::recover_path<FlatState>(std::move(parents), state_index, result.path);
                    result.cost = nodes[i].g;
                    result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
                    result.iterations = iter;
                    result.size.emplace_back(roadmap.size());
                    result.size.emplace_back(0);
                    return result;
                }
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;
            result.size.emplace_back(roadmap.size());
            result.size.emplace_back(0);
            return result;
        }

        inline static auto build_roadmap(
            const FlatState &start,
            const FlatState &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const RoadmapSettings<NeighborParamsT> &settings) noexcept -> Roadmap<flatstate_dimension>
        {
            FlatNN<flatstate_dimension> roadmap;

            constexpr const unsigned int start_index = 0;
            constexpr const unsigned int goal_index = 1;

            auto start_time = std::chrono::steady_clock::now();

            RNG rng;
            std::size_t iter = 0;
            std::vector<std::pair<NNFlatNode<flatstate_dimension>, float>> neighbors;
            typename Robot::template ConfigurationBlock<rake> temp_block;
            auto states = std::unique_ptr<float, decltype(&free)>(
                vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                    settings.max_samples * FlatState::num_scalars_rounded),
                &free);
            // TODO: Is it better to just use arrays for these since we're reserving full capacity
            // anyway? Test it!
            std::vector<RoadmapNode> nodes;
            nodes.reserve(settings.max_samples);

            const auto state_index = [&states](unsigned int index) -> float *
            { return states.get() + index * FlatState::num_scalars_rounded; };

            // Add start and goal to structures
            start.to_array(state_index(start_index));
            auto *goal_state = state_index(goal_index);
            goal.to_array(goal_state);
            nodes.emplace_back(start_index, start_index, 0.0);
            nodes.emplace_back(goal_index, goal_index);
            roadmap.insert(NNFlatNode<flatstate_dimension>{start_index, {state_index(start_index)}});
            roadmap.insert(NNFlatNode<flatstate_dimension>{goal_index, {goal_state}});

            while (iter++ < settings.max_iterations and nodes.size() < settings.max_samples)
            {
                auto temp = rng.next();
                Robot::scale_flatstate(temp);

                // TODO: This is a gross hack to get around the instruction cache issue...I realized
                // that edge sampling, while valid, wastes too much effort with our current
                // validation API

                // Check sample validity
                for (auto i = 0U; i < flat_dimension; ++i)
                {
                    temp_block[i] = temp.broadcast(i);
                }

                if (not Robot::template fkcc<rake>(environment, temp_block))
                {
                    continue;
                }

                // Insert valid state into graph structures
                auto *state = state_index(nodes.size());
                temp.to_array(state);
                auto &node = nodes.emplace_back(nodes.size(), std::numeric_limits<unsigned int>::max());

                // Add valid edges
                const auto k = settings.neighbor_params.max_neighbors(roadmap.size());
                const auto r = settings.neighbor_params.neighbor_radius(roadmap.size());
                auto state_node = NNFlatNode<flat_dimension, flat_order>{node.index, {state}};
                roadmap.nearest(neighbors, state_node.array, k, r);
                for (const auto &[neighbor, distance] : neighbors)
                {
                    if (validate_motion<Robot, rake, resolution>(neighbor.as_vector(), state_node.as_vector(), environment))
                    {
                        node.neighbors.emplace_back(typename RoadmapNode::Neighbor{
                            static_cast<unsigned int>(neighbor.index), distance});
                        nodes[neighbor.index].neighbors.emplace_back(
                            typename RoadmapNode::Neighbor{node.index, distance});
                    }
                }

                // Insert valid state into roadmap - after query to prevent returning self as
                // neighbor
                roadmap.insert(state_node);
            }

            Roadmap<flatstate_dimension> result;
            result.vertices.reserve(nodes.size());

            for (const auto &node : nodes)
            {
                result.vertices.emplace_back(state_index(node.index));
                result.edges.emplace_back(std::vector<std::size_t>());

                for (const auto &nbr : node.neighbors)
                {
                    result.edges.back().emplace_back(nbr.index);
                }
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;

            return result;
        }
    };
}  // namespace vamp::planning
