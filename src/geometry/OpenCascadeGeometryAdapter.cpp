#include "adaptivecad/geometry/GeometryAdapter.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

#ifdef ADAPTIVECAD_HAS_OPENCASCADE
#include <gp_Pnt.hxx>
#endif

namespace adaptivecad::geometry {
namespace {

class OpenCascadeGeometryAdapter final : public GeometryAdapter {
public:
    std::string backend_name() const override {
#ifdef ADAPTIVECAD_HAS_OPENCASCADE
        return "OpenCascade";
#else
        return "OpenCascadeUnavailable";
#endif
    }

    bool is_available() const override {
#ifdef ADAPTIVECAD_HAS_OPENCASCADE
        return true;
#else
        return false;
#endif
    }

    Point3D make_point(double x, double y, double z) const override {
#ifdef ADAPTIVECAD_HAS_OPENCASCADE
        const gp_Pnt point(x, y, z);
        return Point3D{point.X(), point.Y(), point.Z()};
#else
        return Point3D{x, y, z};
#endif
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

std::unique_ptr<GeometryAdapter> create_opencascade_geometry_adapter() {
    return std::make_unique<OpenCascadeGeometryAdapter>();
}

std::unique_ptr<GeometryAdapter> create_preferred_geometry_adapter(bool preferOpenCascade) {
    if (preferOpenCascade) {
        std::unique_ptr<GeometryAdapter> occAdapter = create_opencascade_geometry_adapter();
        if (occAdapter->is_available()) {
            return occAdapter;
        }
    }

    return create_stub_geometry_adapter();
}

} // namespace adaptivecad::geometry
