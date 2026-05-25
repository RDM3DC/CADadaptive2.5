#include "adaptivecad/geometry/GeometryAdapter.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>

namespace adaptivecad::geometry {
namespace {

class StubGeometryAdapter final : public GeometryAdapter {
public:
    std::string backend_name() const override {
        return "StubEuclidean";
    }

    bool is_available() const override {
        return true;
    }

    Point3D make_point(double x, double y, double z) const override {
        return Point3D{x, y, z};
    }

    double circle_circumference(double radius) const override {
        if (radius <= 0.0) {
            throw std::invalid_argument("circle_circumference: radius must be > 0");
        }

        return 2.0 * std::numbers::pi_v<double> * radius;
    }

    double circle_area(double radius) const override {
        if (radius <= 0.0) {
            throw std::invalid_argument("circle_area: radius must be > 0");
        }

        return std::numbers::pi_v<double> * radius * radius;
    }
};

} // namespace

std::unique_ptr<GeometryAdapter> create_stub_geometry_adapter() {
    return std::make_unique<StubGeometryAdapter>();
}

} // namespace adaptivecad::geometry
