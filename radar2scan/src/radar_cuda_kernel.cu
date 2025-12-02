#include "radar2scan/radar_cuda_kernel.hpp"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cmath>
#include <limits>
#include <iostream>

namespace radar_cuda
{
    // CUDA kernel for processing pointcloud to laserscan
    __global__ void processPointcloudKernel(
        const unsigned char* data,
        size_t num_points,
        size_t point_step,
        int x_offset,
        int y_offset,
        int intensity_offset,
        float* ranges,
        float* intensities,
        size_t num_beams,
        float angle_min,
        float angle_increment,
        float range_min,
        float range_max,
        bool enable_intensity_filter,
        float intensity_threshold
    )
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        
        if (idx >= num_points) return;
        
        // Calculate point offset
        size_t point_offset = idx * point_step;
        
        // Extract x, y coordinates
        float x = *reinterpret_cast<const float*>(&data[point_offset + x_offset]);
        float y = *reinterpret_cast<const float*>(&data[point_offset + y_offset]);
        
        // Calculate 2D range
        float range = sqrtf(x * x + y * y);
        
        // Filter by range
        if (range < range_min || range > range_max) return;
        
        // Get intensity if available
        float intensity = 0.0f;
        bool has_intensity = (intensity_offset >= 0);
        if (has_intensity) {
            intensity = *reinterpret_cast<const float*>(&data[point_offset + intensity_offset]);
            if (enable_intensity_filter && intensity < intensity_threshold) {
                return;
            }
        }
        
        // Calculate angle
        float angle = atan2f(y, x);
        
        // Check angle range
        if (angle < angle_min || angle > (angle_min + angle_increment * num_beams)) {
            return;
        }
        
        // Calculate beam index
        int beam_index = static_cast<int>((angle - angle_min) / angle_increment);
        
        if (beam_index >= 0 && beam_index < num_beams) {
            // Atomic min for ranges (using atomic CAS for float)
            unsigned int* range_as_int = (unsigned int*)&ranges[beam_index];
            unsigned int old_int = *range_as_int;
            unsigned int new_int;
            float old_range;
            
            do {
                old_int = *range_as_int;
                old_range = __uint_as_float(old_int);
                if (range >= old_range) break;
                new_int = __float_as_uint(range);
            } while (atomicCAS(range_as_int, old_int, new_int) != old_int);
            
            // Update intensity if this point is closest
            if (range < old_range && has_intensity) {
                intensities[beam_index] = intensity;
            }
        }
    }

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
    )
    {
        // Configure kernel launch parameters
        int blockSize = 256;
        int numBlocks = (num_points + blockSize - 1) / blockSize;
        
        // Launch kernel
        processPointcloudKernel<<<numBlocks, blockSize>>>(
            d_data,
            num_points,
            point_step,
            x_offset,
            y_offset,
            intensity_offset,
            d_ranges,
            d_intensities,
            num_beams,
            angle_min,
            angle_increment,
            range_min,
            range_max,
            enable_intensity_filter,
            intensity_threshold
        );
        
        // Wait for kernel to complete
        cudaDeviceSynchronize();
    }

    bool isCUDAAvailable()
    {
        int deviceCount = 0;
        cudaError_t error = cudaGetDeviceCount(&deviceCount);
        
        if (error != cudaSuccess || deviceCount == 0) {
            return false;
        }
        
        // Check if at least one device has compute capability >= 3.0
        for (int i = 0; i < deviceCount; i++) {
            cudaDeviceProp prop;
            cudaGetDeviceProperties(&prop, i);
            if (prop.major >= 3) {
                return true;
            }
        }
        
        return false;
    }

} // namespace radar_cuda
