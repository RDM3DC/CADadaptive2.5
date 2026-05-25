#include "adaptivecad/geometry/GeometryAdapter.hpp"

#include <cassert>
#include <cmath>
#include <memory>
#include <numbers>
#include <string>

namespace {

bool near(double a, double b, double tol) {
    return std::abs(a - b) <= tol;
}

} // namespace

int main() {
    using adaptivecad::geometry::GeometryAdapter;
    using adaptivecad::geometry::create_preferred_geometry_adapter;
    using adaptivecad::geometry::create_stub_geometry_adapter;

    {
        const std::unique_ptr<GeometryAdapter> stub = create_stub_geometry_adapter();
        assert(stub->is_available());
        assert(stub->backend_name() == "StubEuclidean");
        assert(near(stub->circle_circumference(2.0), 4.0 * std::numbers::pi_v<double>, 1e-12));
        assert(near(stub->circle_area(3.0), 9.0 * std::numbers::pi_v<double>, 1e-12));
    }

    {
        const std::unique_ptr<GeometryAdapter> preferred = create_preferred_geometry_adapter(true);
        assert(preferred != nullptr);
        assert(!preferred->backend_name().empty());
        assert(near(preferred->circle_area(1.0), std::numbers::pi_v<double>, 1e-12));
    }

    return 0;
}
