#include "adaptivecad/tool/ParametricCurve.hpp"

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace adaptivecad::tool {
namespace {

geometry::Point3D interpolate(geometry::Point3D a, geometry::Point3D b, double t) {
    return {std::lerp(a.x, b.x, t), std::lerp(a.y, b.y, t), std::lerp(a.z, b.z, t)};
}

double dot(geometry::Point3D a, geometry::Point3D b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

} // namespace

ParametricCurveDefinition ParametricCurveDefinition::line(geometry::Point3D start,
    geometry::Point3D end, std::size_t samples) {
    ParametricCurveDefinition definition;
    definition.control_points = {start, end, {}, {}};
    definition.sample_count = samples;
    definition.validate();
    return definition;
}

ParametricCurveDefinition ParametricCurveDefinition::cubic_bezier(geometry::Point3D p0,
    geometry::Point3D p1, geometry::Point3D p2, geometry::Point3D p3, std::size_t samples) {
    ParametricCurveDefinition definition;
    definition.kind = ParametricCurveKind::CubicBezier;
    definition.control_points = {p0, p1, p2, p3};
    definition.sample_count = samples;
    definition.validate();
    return definition;
}

ParametricCurveDefinition ParametricCurveDefinition::circle(geometry::Point3D center,
    geometry::Point3D unitU, geometry::Point3D unitV, double circleRadius, std::size_t samples) {
    ParametricCurveDefinition definition;
    definition.kind = ParametricCurveKind::Circle;
    definition.control_points = {center, unitU, unitV, {}};
    definition.sample_count = samples;
    definition.radius = circleRadius;
    definition.validate();
    return definition;
}

void ParametricCurveDefinition::validate() const {
    if (kind != ParametricCurveKind::Line && kind != ParametricCurveKind::CubicBezier &&
        kind != ParametricCurveKind::Circle) throw std::invalid_argument("unknown parametric curve kind");
    if (sample_count > maximum_samples) throw std::length_error("parametric sample limit exceeded");
    if (sample_count < (kind == ParametricCurveKind::Circle ? 3U : 2U)) {
        throw std::invalid_argument("too few parametric samples");
    }
    for (const auto& point : control_points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            throw std::invalid_argument("parametric control points must be finite");
        }
    }
    if (!std::isfinite(radius)) throw std::invalid_argument("parametric radius must be finite");
    if (kind != ParametricCurveKind::Circle) return;
    if (radius <= 0.0) throw std::invalid_argument("circle radius must be positive");
    const auto& u = control_points[1];
    const auto& v = control_points[2];
    if (std::abs(dot(u, u) - 1.0) > 1e-12 || std::abs(dot(v, v) - 1.0) > 1e-12 ||
        std::abs(dot(u, v)) > 1e-12) throw std::invalid_argument("circle basis must be orthonormal");
    // Conservative component bounds reject definitions that could overflow at
    // any requested sample, without evaluating or retaining all those samples.
    const auto safeComponent = [&](double center, double a, double b) {
        return std::abs(static_cast<long double>(center)) + static_cast<long double>(radius) *
            (std::abs(static_cast<long double>(a)) + std::abs(static_cast<long double>(b))) <=
            static_cast<long double>(std::numeric_limits<double>::max()) / 2.0L;
    };
    const auto& center = control_points[0];
    if (!safeComponent(center.x, u.x, v.x) || !safeComponent(center.y, u.y, v.y) ||
        !safeComponent(center.z, u.z, v.z)) throw std::invalid_argument("circle coordinate range is too large");
}

geometry::Point3D ParametricCurveDefinition::evaluate_validated(std::size_t index) const {
    const auto& p = control_points;
    if (kind == ParametricCurveKind::Circle) {
        const double angle = (2.0 * std::numbers::pi) *
            (static_cast<double>(index) / static_cast<double>(sample_count));
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);
        return {p[0].x + radius * (p[1].x * cosine + p[2].x * sine),
                p[0].y + radius * (p[1].y * cosine + p[2].y * sine),
                p[0].z + radius * (p[1].z * cosine + p[2].z * sine)};
    }
    // Explicit endpoints also preserve signed zero exactly at the control ends.
    if (index == 0) return p[0];
    if (index + 1 == sample_count) return kind == ParametricCurveKind::Line ? p[1] : p[3];
    const double t = static_cast<double>(index) / static_cast<double>(sample_count - 1);
    if (kind == ParametricCurveKind::Line) return interpolate(p[0], p[1], t);
    const auto a = interpolate(p[0], p[1], t);
    const auto b = interpolate(p[1], p[2], t);
    const auto c = interpolate(p[2], p[3], t);
    return interpolate(interpolate(a, b, t), interpolate(b, c, t), t);
}

geometry::Point3D ParametricCurveDefinition::evaluate(std::size_t index) const {
    validate();
    if (index >= sample_count) throw std::out_of_range("parametric sample index outside curve");
    return evaluate_validated(index);
}

void ParametricCurveDefinition::evaluate(std::span<const std::size_t> indices,
    std::span<geometry::Point3D> output) const {
    validate();
    if (indices.size() != output.size()) throw std::invalid_argument("parametric batch span sizes differ");
    for (const auto index : indices) {
        if (index >= sample_count) throw std::out_of_range("parametric sample index outside curve");
    }
    for (std::size_t i = 0; i < indices.size(); ++i) output[i] = evaluate_validated(indices[i]);
}

} // namespace adaptivecad::tool
