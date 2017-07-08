#include <iostream>

#include <myers_diff.hpp>

int main()
{
    using namespace std::literals;
    using namespace myers_diff;

    //auto const a = "ABCABBA"sv;
    //auto const b = "CBABAC"sv;

    //auto const a = "1234567890ABCDEFGH"sv;
    //auto const b = "0ABCDEFGH123456789"sv;

    auto const a = "A{BC_D_E}_1{2_3_4}"sv;
    auto const b = "1{2_3_4}_A{BC_D_E}"sv;

    struct v : visitor_t {
        v(std::string_view const& a, std::string_view const& b)
            : m_a { a }, m_b { b } {}

        void unchanged(size_t const x, size_t) const override
        {
            std::cout << m_current << "  " << m_a[x] << "\n";
            ++m_current;
        }

        void deleted(size_t const x) const override
        {
            std::cout << m_current << "- " << m_a[x] << "\n";
        }

        void inserted(size_t const y) const override
        {
            std::cout << m_current << "+ " << m_b[y] << "\n";
            ++m_current;
        }

        std::string_view const& m_a;
        std::string_view const& m_b;
        mutable size_t m_current = 0;

    } visitor { a, b };

    diff(a, b, visitor);
}
