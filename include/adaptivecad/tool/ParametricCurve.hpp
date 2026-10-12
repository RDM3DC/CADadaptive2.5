#pragma once

#include "adaptivecad/geometry/GeometryAdapter.hpp"

#include <array>
#include <cstddef>
#include <span>

namespace adaptivecad::tool {

enum class ParametricCurveKind { Line, CubicBezier, Circle };

// Authoritative definitions for newly generated geometry. No fitting or
// replacement of imported sampled coordinates is performed. Evaluation is
// deterministic within the current binary/math runtime; libm trigonometry is
// not specified to produce identical bits across different platforms.
struct ParametricCurveDefinition {
    static constexpr std::size_t maximum_samples = 16 * 1024 * 1024;
    ParametricCurveKind kind = ParametricCurveKind::Line;
    // Line: start/end; Bezier: four control points; circle: center/unit U/unit V.
    std::array<geometry::Point3D, 4> control_points{};
    std::size_t sample_count = 2;
    double radius = 0.0;

    // Line and Bezier use t = index / (sample_count - 1), including both ends.
    static ParametricCurveDefinition line(geometry::Point3D start, geometry::Point3D end,
        std::size_t samples);
    static ParametricCurveDefinition cubic_bezier(geometry::Point3D p0, geometry::Point3D p1,
        geometry::Point3D p2, geometry::Point3D p3, std::size_t samples);
    // Full circle uses angle = 2*pi*index/sample_count, with no duplicate end.
    // U/V must be orthonormal within 1e-12; they are never silently normalized.
    static ParametricCurveDefinition circle(geometry::Point3D center, geometry::Point3D unitU,
        geometry::Point3D unitV, double radius, std::size_t samples);

    void validate() const;
    geometry::Point3D evaluate(std::size_t index) const;
    // Validates all indices and equal span lengths before writing any output.
    void evaluate(std::span<const std::size_t> indices, std::span<geometry::Point3D> output) const;

private:
    friend class CurveMemoryScene;
    geometry::Point3D evaluate_validated(std::size_t index) const;
};

} // namespace adaptivecad::tool
