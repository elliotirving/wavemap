#include "wavemap/integrator/ray_tracing/ray_tracing_integrator.h"
#include <ros/ros.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(RayTracingIntegratorConfig,
                      (min_range)
                      (max_range));

bool RayTracingIntegratorConfig::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(min_range, 0.f, verbose);
  is_valid &= IS_PARAM_GT(max_range, 0.f, verbose);
  is_valid &= IS_PARAM_LT(min_range, max_range, verbose);

  return is_valid;
}

void RayTracingIntegrator::integratePointcloud(
    const PosedPointcloud<>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }

  ROS_INFO_STREAM("RayTracingIntegrator::integratePointcloud called");

  const FloatingPoint min_cell_width = occupancy_map_->getMinCellWidth();
  const Point3D& W_start_point = pointcloud.getOrigin();

  MeasurementModelType measurement_model(min_cell_width);
  measurement_model.setStartPoint(W_start_point);

  for (const auto& W_end_point : pointcloud.getPointsGlobal()) {
    measurement_model.setEndPoint(W_end_point);

    if (!isMeasurementValid(W_end_point - W_start_point)) {
      continue;
    }

    const FloatingPoint measured_distance =
        (W_start_point - W_end_point).norm();
    const Point3D W_end_point_truncated = getEndPointOrMaxRange(
        W_start_point, W_end_point, measured_distance, config_.max_range);
    const Ray ray(W_start_point, W_end_point_truncated, measured_distance);
    for (const auto& index : ray) {
      const FloatingPoint update = measurement_model.computeUpdate(index);
      occupancy_map_->addToCellValue(index, update);
    }
  }
}
}  // namespace wavemap
