#pragma once

#include <memory>
#include <string>

namespace adaptivecad::geometry {

struct Point3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

class GeometryAdapter {
public:
    virtual ~GeometryAdapter() = default;

    virtual std::string backend_name() const = 0;
    virtual bool is_available() const = 0;

    virtual Point3D make_point(double x, double y, double z) const = 0;
    virtual double circle_circumference(double radius) const = 0;
    virtual double circle_area(double radius) const = 0;
};

std::unique_ptr<GeometryAdapter> create_stub_geometry_adapter();
std::unique_ptr<GeometryAdapter> create_opencascade_geometry_adapter();
std::unique_ptr<GeometryAdapter> create_preferred_geometry_adapter(bool preferOpenCascade = true);

} // namespace adaptivecad::geometry
