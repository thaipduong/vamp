#pragma once

#include <vamp/robots/sphere/fk.hh>
#include <vamp/vector.hh>

namespace vamp::robots
{
    struct Sphere
    {
        static constexpr auto name = "sphere";
        static constexpr auto dimension = 3;
        static constexpr auto flat_dimension = 3;
        static constexpr auto flat_order = 2;
        static constexpr auto flatstate_dimension = flat_dimension*flat_order;
        static constexpr auto resolution = 32;
        static constexpr auto n_spheres = sphere::n_spheres;
        static constexpr auto space_measure = sphere::space_measure;

        using Configuration = FloatVector<dimension>;
        using ConfigurationArray = std::array<FloatT, dimension>;
        using ConfigurationFlat = FloatVector<flat_dimension>;
        using ConfigurationFlatState = FloatVector<flatstate_dimension>; // Flat state (flat output + its derivatives)
        using ConfigurationFlatStateVecArray = std::array<FloatVector<flat_dimension>, flat_order>; // For trajectory generation, should be merged with FlatState later
        using ConfigurationFlatStateArray = std::array<FloatT, flatstate_dimension>; // This is for pybinding

        struct alignas(FloatVectorAlignment) ConfigurationBuffer
          : std::array<float, Configuration::num_scalars_rounded>
        {
        };

        template <std::size_t rake>
        using ConfigurationBlock = sphere::ConfigurationBlock<rake>;

        template <std::size_t rake>
        using Spheres = sphere::Spheres<rake>;

        static constexpr auto scale_configuration = sphere::scale_configuration;
        static constexpr auto descale_configuration = sphere::descale_configuration;
        static constexpr auto scale_flatstate = sphere::scale_flatstate;



        static auto flatstate_to_vecarray(const ConfigurationFlatState &flatstate) -> ConfigurationFlatStateVecArray
        {
          
          auto flat_array = flatstate.to_array();
          std::array<FloatT, flat_dimension> flat_output{0};
          std::array<FloatT, flat_dimension> flat_dev{0};
          std::copy(flat_array.begin(), flat_array.begin() + flat_dimension, flat_output.begin());
          std::copy(flat_array.begin() + flat_dimension, flat_array.begin() + flatstate_dimension, flat_dev.begin());
          ConfigurationFlatStateVecArray ret{flat_output,flat_dev};
          // for (int i = 0; i < flat_order; i++)
          // { 
          //   ret[i].pack(flat_array.data() + i*flat_dimension);
          // }
          return ret;
        }

        static auto flatarray_to_vecarray(const ConfigurationFlatStateArray &flat_array) -> ConfigurationFlatStateVecArray
        {
          std::array<FloatT, flat_dimension> flat_output{0};
          std::array<FloatT, flat_dimension> flat_dev{0};
          std::copy(flat_array.begin(), flat_array.begin() + flat_dimension, flat_output.begin());
          std::copy(flat_array.begin() + flat_dimension, flat_array.begin() + flatstate_dimension, flat_dev.begin());
          ConfigurationFlatStateVecArray ret{flat_output,flat_dev};
          // for (int i = 0; i < flat_order; i++)
          // { 
          //   ret[i].pack(flat_array.data() + i*flat_dimension);
          // }
          return ret;
        }

        template <std::size_t rake>
        static constexpr auto scale_configuration_block = sphere::scale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto descale_configuration_block = sphere::descale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc = sphere::interleaved_sphere_fk<rake>;

        // Currently not implemented
        template <std::size_t rake>
        static constexpr auto fkcc_attach = sphere::interleaved_sphere_fk<rake>;

        template <std::size_t rake>
        static constexpr auto sphere_fk = sphere::sphere_fk<rake>;

        // Currently not implemented
        static constexpr auto eefk = sphere::eefk;
    };
}  // namespace vamp::robots
