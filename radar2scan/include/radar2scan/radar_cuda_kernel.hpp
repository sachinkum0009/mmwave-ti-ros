#ifndef RADAR_CUDA_KERNEL_HPP_
#define RADAR_CUDA_KERNEL_HPP_

#include <cuda_runtime.h>
#include <cstddef>

namespace radar_cuda
{
    /**
     * @brief CUDA kernel wrapper for processing radar pointcloud to laserscan
     * 
     * @param d_data Device pointer to pointcloud data
     * @param num_points Number of points in the pointcloud
     * @param point_step Byte offset between consecutive points
     * @param x_offset Byte offset to x field
     * @param y_offset Byte offset to y field
     * @param intensity_offset Byte offset to intensity field (-1 if not present)
     * @param d_ranges Device pointer to output ranges array
     * @param d_intensities Device pointer to output intensities array
     * @param num_beams Number of laser beams
     * @param angle_min Minimum scan angle (radians)
     * @param angle_max Maximum scan angle (radians)
     * @param angle_increment Angular resolution (radians)
     * @param range_min Minimum valid range (meters)
     * @param range_max Maximum valid range (meters)
     * @param enable_intensity_filter Enable intensity filtering
     * @param intensity_threshold Minimum intensity threshold
     */
    void processPointcloudCUDA(
        const unsigned char* d_data,
        size_t num_points,
        size_t point_step,
        int x_offset,
        int y_offset,
        int intensity_offset,
        float* d_ranges,
        float* d_intensities,
        size_t num_beams,
        float angle_min,
        float angle_max,
        float angle_increment,
        float range_min,
        float range_max,
        bool enable_intensity_filter,
        float intensity_threshold
    );

    /**
     * @brief Check if CUDA is available
     */
    bool isCUDAAvailable();

} // namespace radar_cuda

#endif // RADAR_CUDA_KERNEL_HPP_
