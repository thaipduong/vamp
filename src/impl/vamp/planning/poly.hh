#pragma once

#include <limits>
#include <vamp/vector.hh>
#include <vamp/planning/plan.hh>
#include <vector>

namespace vamp::planning
{   
    template <std::size_t dim>
    using Coeffs = std::vector<FloatVector<dim>>;

    template <std::size_t dim>
    struct Polynomial
    {
        std::size_t order;
        Coeffs<dim> coeffs;
        explicit Polynomial(Coeffs<dim> coeffs_, std::size_t order_) noexcept : order(order_), coeffs(std::move(coeffs_))
        {
            assert(coeffs_.size() == order_ + 1);
        }
        
        inline auto print() noexcept
        {
            for (auto coeff: coeffs)
            {
                std::cout << coeff << std::endl;
            }
        }

        inline auto eval(float t) noexcept
        {
            FloatVector<dim> val = coeffs.at(0);

            for(auto i = 1U; i < coeffs.size(); i++)
            {
                val = val + coeffs.at(i)*std::pow(t, i);
            }
            return val;
        }

        inline auto derivative() noexcept
        {
            Coeffs<dim> ret_coeffs;
            for(auto i = 0U; i < coeffs.size(); i++)
            {
                ret_coeffs.emplace_back(i*this->coeffs.at(i));
            }
            return Polynomial<dim>(ret_coeffs, this->order - 1);
        }

        inline auto integral() noexcept
        {
            Coeffs<dim> ret_coeffs;
            ret_coeffs.emplace_back(FloatVector<dim>::zero_vector());
            for(auto i = 0U; i < coeffs.size(); i++)
            {
                ret_coeffs.emplace_back(this->coeffs.at(i)/(i+1));
            }
            return Polynomial<dim>(ret_coeffs, this->order + 1);
        }

        inline constexpr auto operator*(Polynomial<dim> o) const noexcept -> Polynomial<dim> 
        {
            Coeffs<dim> ret_coeffs;
            for(auto i = 0U; i < coeffs.size(); i++)
            {
                for(auto j = 0U; j < coeffs.size(); j++)
                {
                    if (ret_coeffs.size() < i + j + 1)
                        ret_coeffs.emplace_back(FloatVector<dim>::zero_vector());
                    ret_coeffs.at(i+j) = ret_coeffs.at(i+j) + this->coeffs.at(i)*o.coeffs.at(j);
                }
            }
            return Polynomial<dim>(ret_coeffs, this->order + o.order);
        }

        inline auto to_path(float T,
                                std::size_t resolution /*number of samples per second*/
                                ) noexcept -> Path<dim>
        {
            Path<dim> new_path;
            auto n_states = static_cast<std::size_t>(T * static_cast<float>(resolution));
            new_path.reserve(n_states);

            for (auto i = 0U; i < n_states; ++i)
            {
                new_path.emplace_back(this->eval(static_cast<float>(i)/static_cast<float>(resolution)));
            }
            return new_path;
        }
    };

    template <std::size_t dim>
    inline auto opt_traj(FloatVector<dim> y0,
                         FloatVector<dim> y0_dot,
                         FloatVector<dim> yf,
                         FloatVector<dim> yf_dot,
                         float T
                         ) noexcept -> Polynomial<dim>
    {
        assert(T>0);
        Coeffs<dim> ret_coeffs;
        ret_coeffs.emplace_back(y0);
        ret_coeffs.emplace_back(y0_dot);
        auto dT1 = yf - y0 - y0_dot*T;
        auto dT2 = yf_dot - y0_dot;
        auto coeff2 = dT1*3/(T*T) - dT2/T;
        auto coeff3 = dT1*(-2)/(T*T*T) + dT2/(T*T);
        ret_coeffs.emplace_back(coeff2);
        ret_coeffs.emplace_back(coeff3);
        return Polynomial<dim>(ret_coeffs, 3);
    }

}