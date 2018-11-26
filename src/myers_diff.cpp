#include <myers_diff.hpp>

#include "array.hpp"

#include <cassert>
#include <iostream>
#include <optional>
#include <utility>
#include <vector>

#include <boost/format.hpp>
#include <boost/numeric/conversion/cast.hpp>

#include <range/v3/iterator_range.hpp>
#include <range/v3/view/zip.hpp>

namespace myers_diff {

class point_t;
using index_t = std::intmax_t;

} // namespace myers_diff

namespace std {

    template<>
    struct tuple_size<myers_diff::point_t> : integral_constant<size_t, 2> {};

    template<size_t N>
    struct tuple_element<N, myers_diff::point_t>
    {
        using type = myers_diff::index_t;
    };

} // namespace std

namespace myers_diff {

template<typename T>
index_t
to_index_t(T const value)
{
    return boost::numeric_cast<index_t>(value);
}

size_t
to_size_t(index_t const value)
{
    return boost::numeric_cast<size_t>(value);
}

using array_type = utility::array_t<index_t>;

class point_t
{
public:
    point_t(index_t const x, index_t const y)
        : m_x { x }
        , m_y { y }
    {
        assert(m_x >= 0);
        assert(m_y >= 0);
    }

    index_t x() const { return m_x; }
    index_t y() const { return m_y; }

    friend std::ostream& operator<<(std::ostream& os, point_t const p)
    {
        os << "[" << p.m_x << ", " << p.m_y << "]";
        return os;
    }

private:
    index_t m_x;
    index_t m_y;
};

template<size_t N>
index_t get(point_t const& point)
{
    if      constexpr (N == 0) return point.x();
    else if constexpr (N == 1) return point.y();
}

class snake_t
{
public:
    snake_t(point_t const from, point_t const to)
        : m_from { from }
        , m_to { to }
    {
        assert(m_to.x() >= m_from.x());
        assert(m_to.y() >= m_from.y());
    }

    snake_t(index_t const from_x, index_t const from_y,
            index_t const to_x, index_t const to_y)
        : snake_t { { from_x, from_y }, { to_x, to_y } }
    {}

    point_t from() const { return m_from; }
    point_t to() const { return m_to; }

    friend std::ostream& operator<<(std::ostream& os, snake_t const snake)
    {
        os << snake.m_from << ", " << snake.m_to;
        return os;
    }

private:
    point_t m_from;
    point_t m_to;
};

class box_t
{
public:
    box_t(point_t const top_left, point_t const bottom_right)
        : m_top_left { top_left }
        , m_bottom_right { bottom_right }
    {
        assert(m_bottom_right.x() >= m_top_left.x());
        assert(m_bottom_right.y() >= m_top_left.y());
    }

    box_t(index_t const left, index_t const top,
          index_t const right, index_t const bottom)
        : box_t { { left, top }, { right, bottom } }
    {}

    index_t left() const { return m_top_left.x(); }
    index_t right() const { return m_bottom_right.x(); }
    index_t top() const { return m_top_left.y(); }
    index_t bottom() const { return m_bottom_right.y(); }

    point_t const& top_left() const { return m_top_left; }
    point_t const& bottom_right() const { return m_bottom_right; }

    index_t width() const { return right() - left(); }
    index_t height() const { return bottom() - top(); }
    index_t size() const { return width() + height(); }
    index_t delta() const { return width() - height(); }

    index_t k_to_c(index_t const k) const { return k - delta(); }
    index_t c_to_k(index_t const c) const { return c + delta(); }

    index_t to_x(index_t const y, index_t const k) const
    {
        return left() + y - top() + k;
    }

    index_t to_y(index_t const x, index_t const k) const
    {
        return top() + x - left() - k;
    }

    bool contains(index_t const x, index_t const y) const
    {
        return left() <= x && x <= right()
            && top() <= y && y <= bottom();
    }

    bool has_odd_delta() const { return delta() % 2; }
    bool has_even_delta() const { return !has_odd_delta(); }

    friend std::ostream& operator<<(std::ostream& os, box_t const box)
    {
        os << box.m_top_left << ", " << box.m_bottom_right;
        return os;
    }

private:
    point_t m_top_left;
    point_t m_bottom_right;
};

static std::optional<snake_t>
forwards(box_t const& box, equal_fn const& equal,
         array_type& vf, array_type const& vb, index_t const d)
{
    std::optional<snake_t> snake;
    auto snake_score = std::numeric_limits<index_t>::min();
    index_t px = 0, py = 0, x = 0, y = 0;

    for (auto k = -d; k <= d; k += 2) {
        if (k == -d || (k != d && vf[k - 1] < vf[k + 1])) {
            px = x = vf[k + 1];
        }
        else {
            px = vf[k - 1];
            x = px + 1;
        }

        y = box.to_y(x, k);
        py = (d == 0 || x != px) ? y : y - 1;

        while (box.contains(x+1, y+1) && equal(to_size_t(x), to_size_t(y))) {
            ++x, ++y;
        }

        vf[k] = x;

        auto const c = box.k_to_c(k);
#if 0
        std::cout << "forward : " << box
                  << ": (d, k, c) = (" << d << ", " << k << ", " << c << ")"
                  << ", (x, y) = (" << x << ", " << y << ")"
                  << ", (px, py) = (" << px << ", " << py << ")"
                  << "\n";
#endif
        if (box.has_odd_delta() &&
                (-(d - 1) <= c && c <= (d - 1)) && y >= vb[c])
        {
            assert(box.contains(x, y));
            assert(box.contains(px, py));

            auto const score =
                    (x - box.left()) + (y - box.top()) - std::abs(k);
#if 0
            std::cout << "mid snake candidate: "
                      << point_t { px, py } << " -> " << point_t { x, y }
                      << ", score = " << score << "\n";
#endif
            if (!snake || snake_score < score) {
                snake.emplace(px, py, x, y);
                snake_score = score;
            }
        }
    }

    return snake;
}

static std::optional<snake_t>
backward(box_t const& box, equal_fn const& equal,
         array_type const& vf, array_type& vb, index_t const d)
{
    std::optional<snake_t> snake;
    auto snake_score = std::numeric_limits<index_t>::min();
    index_t px = 0, py = 0, x = 0, y = 0;

    for (auto c = -d; c <= d; c += 2) {
        if (c == -d || (c != d && vb[c - 1] > vb[c + 1])) {
            py = y = vb[c + 1];
        }
        else {
            py = vb[c - 1];
            y = py - 1;
        }

        auto const k = box.c_to_k(c);

        x = box.to_x(y, k);
        px = (d == 0 || y != py) ? x : x + 1;

        while (box.contains(x - 1, y - 1)
                            && equal(to_size_t(x - 1), to_size_t(y - 1)))
        {
            --x, --y;
        }
#if 0
        std::cout << "backward: " << box
                  << ": (d, k, c) = (" << d << ", " << k << ", " << c << ")"
                  << ", (x, y) = (" << x << ", " << y << ")"
                  << ", (px, py) = (" << px << ", " << py << ")"
                  << "\n";
#endif
        vb[c] = y;

        if (box.has_even_delta() && -d <= k && k <= d && x <= vf[k])
        {
            assert(box.contains(x, y));
            assert(box.contains(px, py));

            auto const score =
                    (box.right() - x) + (box.bottom() - y) - std::abs(c);
#if 0
            std::cout << "mid snake candidate: "
                      << point_t { x, y } << " <- " << point_t { px, py }
                      << ", score = " << score << ", c = " << c << "\n";
#endif
            if (!snake || snake_score < score) {
                snake.emplace(x, y, px, py);
                snake_score = score;
            }
        }
    }

    return snake;
}

static std::optional<snake_t>
midpoint(box_t const& box, equal_fn const& equal)
{
    if (box.size() == 0) return {};

    auto const max = to_index_t(
                std::ceil(static_cast<double>(box.size()) / 2.0) );

    array_type vf { -max, max };
    vf[1] = box.left();

    array_type vb { -max, max };
    vb[1] = box.bottom();

    for (auto d = 0; d <= max; ++d) {
        auto const snake_f = forwards(box, equal, vf, vb, d);
        auto const snake_b = backward(box, equal, vf, vb, d);

        assert(!(snake_f && snake_b));

        if (snake_f || snake_b) {
            return snake_f ? snake_f : snake_b;
        }
    }

    //std::cout << "no midpoint:" << box << ", max = " << max << "\n";
    return {};
}

static bool
find_path(box_t const& box, equal_fn const& equal, std::vector<point_t>& result, int const level = 0)
{
    auto const snake = midpoint(box, equal);

    if (!snake) return false;

    //static boost::format fmt { "%4% - %1%: box: %2%, snake: %3%\n" };

    //std::cout << fmt % "head" % box % *snake % level;
    if (!find_path({ box.top_left(), snake->from() }, equal, result, level+1)) {
        //std::cout << "pushing snake from " << snake->from() << "\n";
        result.push_back(snake->from());
    }

    //std::cout << fmt % "tail" % box % *snake % level;
    if (!find_path({ snake->to(), box.bottom_right() }, equal, result, level+1)) {
        //std::cout << "pushing snake to " << snake->to() << "\n";
        result.push_back(snake->to());
    }

    return true;
}

static std::vector<point_t>
find_path(box_t const& box, equal_fn const& equal)
{
    std::vector<point_t> results;

    find_path(box, equal, results);

    return results;
}

template<typename Range>
auto each_cons(Range&& range)
{
    namespace rng = ranges;

    assert(rng::size(range) >= 2);

    auto const begin = rng::begin(range);
    auto const end = rng::end(range);

    auto from = rng::make_iterator_range(begin, end-1);
    auto to = rng::make_iterator_range(begin+1, end);

    return rng::view::zip(from, to);
}

static point_t
walk_diagonal(point_t const from, point_t const to,
              equal_fn const equal, visitor_t const& visitor)
{
    auto [x, y] = from;

    while (x < to.x() && y < to.y() && equal(to_size_t(x), to_size_t(y))) {
        visitor.unchanged(to_size_t(x), to_size_t(y));

        ++x, ++y;
    }

    return { x, y };
}

static void
walk_path(std::vector<point_t> const& path,
          equal_fn const equal, visitor_t const& visitor)
{
    for (auto const& [from, to]: each_cons(path)) {
        auto [x, y] = walk_diagonal(from, to, equal, visitor);

        auto const x_diff = to.x() - x;
        auto const y_diff = to.y() - y;

        if (x_diff > y_diff) {
            visitor.deleted(to_size_t(x));
            ++x;
        }
        else if (x_diff < y_diff) {
            visitor.inserted(to_size_t(y));
            ++y;
        }

        walk_diagonal({x, y}, to, equal, visitor);
    }
}

size_t
diff(size_t const size1_, size_t const size2_,
                         equal_fn const& equal, visitor_t const& visitor)
{
    auto const size1 = to_index_t(size1_);
    auto const size2 = to_index_t(size2_);

    if (size1 == 0 && size2 == 0) return 0;

    assert(std::numeric_limits<index_t>::max() - size1 >= size2);

    auto const path = find_path({ 0, 0, size1, size2 }, equal);

    walk_path(path, equal, visitor);

    return path.size();
}

} // namespace myers_diff
