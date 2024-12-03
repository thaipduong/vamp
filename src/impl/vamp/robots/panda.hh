#pragma once

#include <vamp/robots/panda/fk.hh>
#include <vamp/vector.hh>

namespace vamp::robots
{
    struct Panda
    {
        static constexpr auto name = "panda";
        static constexpr auto dimension = 7;
        static constexpr auto flat_dimension = 7;
        static constexpr auto flat_order = 2;
        static constexpr auto flatstate_dimension = flat_dimension*flat_order;
        static constexpr auto resolution = 32;
        static constexpr auto n_spheres = panda::n_spheres;
        static constexpr auto space_measure = panda::space_measure;

        using Configuration = FloatVector<dimension>;
        using ConfigurationArray = std::array<FloatT, dimension>;
        using ConfigurationFlat = FloatVector<flat_dimension>; // Flat output
        using ConfigurationFlatState = FloatVector<flatstate_dimension>; // Flat state (flat output + its derivatives)
        using ConfigurationFlatStateVecArray = std::array<FloatVector<flat_dimension>, flat_order>; // For trajectory generation, should be merged with FlatState later
        using ConfigurationFlatStateArray = std::array<std::array<FloatT, flat_dimension>, flat_order>; // This is for pybinding

        struct alignas(FloatVectorAlignment) ConfigurationBuffer
          : std::array<float, Configuration::num_scalars_rounded>
        {
        };

        template <std::size_t rake>
        using ConfigurationBlock = panda::ConfigurationBlock<rake>;

        template <std::size_t rake>
        using Spheres = panda::Spheres<rake>;

        static constexpr auto scale_configuration = panda::scale_configuration;
        static constexpr auto descale_configuration = panda::descale_configuration;
        static constexpr auto scale_flatstate = panda::scale_flatstate;

        template <std::size_t rake>
        static constexpr auto scale_configuration_block = panda::scale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto descale_configuration_block = panda::descale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc = panda::interleaved_sphere_fk<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc_attach = panda::interleaved_sphere_fk_attachment<rake>;

        template <std::size_t rake>
        static constexpr auto sphere_fk = panda::sphere_fk<rake>;

        static constexpr auto eefk = panda::eefk;
    };
}  // namespace vamp::robots
