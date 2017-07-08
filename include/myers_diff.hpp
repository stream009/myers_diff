#ifndef MYERS_DIFF_LINEAR_HPP
#define MYERS_DIFF_LINEAR_HPP

#include <cstdint>
#include <functional>
#include <iterator>

namespace myers_diff {

// types

using equal_fn = std::function<bool(size_t, size_t)>;

class visitor_t
{
public:
    virtual ~visitor_t() = default;

    virtual void unchanged(size_t, size_t) const {}
    virtual void deleted(size_t) const {}
    virtual void inserted(size_t) const {}
};

// functions

size_t diff(size_t const size1, size_t const size2,
                                equal_fn const&, visitor_t const&);
// precondition: size1 + size2 <= std::numeric_limits<std::intmax_t>::max()

template<typename Range>
size_t
diff(Range const& a, Range const& b, visitor_t const& visitor)
{
    auto equal = [&](auto const x, auto const y) {
                    return a[x] == b[y];
                 };

    return diff(std::size(a), std::size(b), equal, visitor);
}

} // namespace myers_diff

#endif // MYERS_DIFF_LINEAR_HPP
