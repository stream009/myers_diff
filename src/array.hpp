#ifndef UTILITY_ARRAY_HPP
#define UTILITY_ARRAY_HPP

#include <cassert>
#include <vector>

#include <boost/numeric/conversion/cast.hpp>

namespace utility {

/*
 * min, max = 2, 2
 * arr[2] = 2 - 2 ... v[0]
 *
 * min, max = 2, 5
 * arr[2] = 2 - 2 ... v[0]
 * arr[3] = 3 - 2 ... v[1]
 * arr[4] = 4 - 2 ... v[2]
 * arr[5] = 5 - 2 ... v[3]
 *
 * min, max = -2, 2
 * arr[-2] = -2 - (-2) ... v[0]
 * arr[-1] = -1 - (-2) ... v[1]
 * arr[ 0] =  0 - (-2) ... v[2]
 * arr[ 1] =  1 - (-2) ... v[3]
 * arr[ 2] =  2 - (-2) ... v[4]
 *
 */
template<typename T>
class array_t
{
public:
    using container_type = std::vector<T>;
    using size_type = typename container_type::size_type;
    using index_type = std::make_signed_t<size_type>;
    using value_type = typename container_type::value_type;

public:
    array_t(index_type const min, index_type const max)
        : m_min { min }
        , m_max { max }
    {
        assert(min <= max);

        auto const size =
            boost::numeric_cast<size_type>(std::abs(min) + std::abs(max) + 1);

        m_values.resize(size);
    }

    index_type min() const { return m_min; }
    index_type max() const { return m_max; }

    value_type& operator[](index_type const index)
    {
        assert(m_min <= index);
        assert(index <= m_max);

        return m_values[static_cast<size_t>(index - m_min)];
    }

    value_type const& operator[](index_type const index) const
    {
        assert(m_min <= index);
        assert(index <= m_max);

        return m_values[static_cast<size_t>(index - m_min)];
    }

    size_type size() const { return m_values.size(); }

    auto begin() const { return m_values.begin(); }
    auto end() const { return m_values.end(); }

private:
    container_type m_values;
    index_type m_min;
    index_type m_max;
};

} // namespace utility

#endif // UTILITY_ARRAY_HPP
