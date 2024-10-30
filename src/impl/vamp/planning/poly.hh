#pragma once

#include <limits>
#include <vamp/vector.hh>
#include <vector>

namespace vamp::planning
{   
    template <std::size_t dim>
    using Coeffs = std::vector<FloatVector<dim>>;

    template <std::size_t dim, std::size_t order>
    struct Polynomial
    {
        std::size_t order;
        Coeffs<dim> coeffs;
        explicit Polynomial(Coeffs<dim> coeffs_, std::size_t order_) noexcept : order(order_), coeffs(std::move(coeffs_))
        {
            assert(coeffs_.size() == order_ + 1);
        }
        
        inline auto eval(float t) noexcept
        {
            FloatVector<dim> val = coeffs.at(0);

            for(auto i = 1U; i < coeffs.size(); i++)
            {
                val += coeffs.at(i)*t;
            }
            return val;
        }

        inline auto derivatives() noexcept
        {
            Coeffs<dim> ret_coeffs;
            for(auto i = 0U; i < coeffs.size(); i++)
            {
                ret_coeffs.emplace_back(i*this->coeffs.at(i));
            }
            return Polynomial(ret_coeffs, this->order - 1);
        }

        inline auto integral() noexcept
        {
            Coeffs<dim> ret_coeffs;
            ret_coeffs.emplace_back(FloatVector<dim>::zero_vector());
            for(auto i = 0U; i < coeffs.size(); i++)
            {
                ret_coeffs.emplace_back(this->coeffs.at(i)/(i+1));
            }
            return Polynomial(ret_coeffs, this->order + 1);
        }

        inline constexpr auto operator*(Polynomial o) const noexcept -> D
        {
            Coeffs<dim> ret_coeffs;
            for(auto i = 0U; i < coeffs.size(); i++)
            {
                for(auto j = 0U; j < coeffs.size(); i++)
                {
                    if (ret_coeffs.size() < i + j + 1)
                        ret_coeffs.emplace_back(FloatVector<dim>::zero_vector());
                    ret_coeffs.at(i+j+1) += this->coeffs.at(i)*o.coeffs.at(j);
                }
            }
            return Polynomial(ret_coeffs, this->order + o.order);
        }
    };
}