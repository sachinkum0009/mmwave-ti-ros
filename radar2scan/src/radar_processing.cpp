#include "radar2scan/radar_processing.hpp"
#ifdef USE_CUDA
#include "radar2scan/radar_cuda_kernel.hpp"
#include <cuda_runtime.h>
#endif

namespace radar_processing
{
    RadarProcessor::RadarProcessor()
    : Node("radar_processor"),
      d_data_(nullptr), d_ranges_(nullptr), d_intensities_(nullptr),
      d_data_size_(0), d_ranges_size_(0)
    {
        // Declare and get parameters
        this->declare_parameter("angle_min", -M_PI);
        this->declare_parameter("angle_max", M_PI);
        this->declare_parameter("angle_increment", 0.005); // ~0.3 degrees
        this->declare_parameter("range_min", 0.1);
        this->declare_parameter("range_max", 10.0);
        this->declare_parameter("frame_id", "ti_mmwave_0");
        this->declare_parameter("enable_intensity_filter", false);
        this->declare_parameter("intensity_threshold", 50.0);
        this->declare_parameter("use_cuda", true);
        
        angle_min_ = this->get_parameter("angle_min").as_double();
        angle_max_ = this->get_parameter("angle_max").as_double();
        angle_increment_ = this->get_parameter("angle_increment").as_double();
        range_min_ = this->get_parameter("range_min").as_double();
        range_max_ = this->get_parameter("range_max").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        enable_intensity_filter_ = this->get_parameter("enable_intensity_filter").as_bool();
        intensity_threshold_ = this->get_parameter("intensity_threshold").as_double();
        use_cuda_ = this->get_parameter("use_cuda").as_bool();
        
        // Check CUDA availability
#ifdef USE_CUDA
        cuda_available_ = radar_cuda::isCUDAAvailable();
        if (use_cuda_ && !cuda_available_) {
            RCLCPP_WARN(this->get_logger(), "CUDA requested but not available, falling back to CPU");
            use_cuda_ = false;
        }
#else
        cuda_available_ = false;
        if (use_cuda_) {
            RCLCPP_WARN(this->get_logger(), "CUDA support not compiled, falling back to CPU");
            use_cuda_ = false;
        }
#endif
        
        // Create subscriber for radar pointcloud
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ti_mmwave/radar_scan_pcl",
            10,
            std::bind(&RadarProcessor::radarCallback, this, std::placeholders::_1)
        );
        
        // Create publisher for laserscan
        laserscan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/radar_scan",
            10
        );
        
        RCLCPP_INFO(this->get_logger(), "RadarProcessor node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /ti_mmwave/radar_scan_pcl");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /radar_scan");
        RCLCPP_INFO(this->get_logger(), "Processing mode: %s", use_cuda_ ? "CUDA GPU" : "CPU (OpenMP)");
        RCLCPP_INFO(this->get_logger(), "Angle range: [%.2f, %.2f] rad", angle_min_, angle_max_);
        RCLCPP_INFO(this->get_logger(), "Range limits: [%.2f, %.2f] m", range_min_, range_max_);
        if (enable_intensity_filter_) {
            RCLCPP_INFO(this->get_logger(), "Intensity filter ENABLED: threshold = %.2f", intensity_threshold_);
        } else {
            RCLCPP_INFO(this->get_logger(), "Intensity filter DISABLED");
        }
    }

    RadarProcessor::~RadarProcessor()
    {
#ifdef USE_CUDA
        // Free CUDA memory
        if (d_data_) cudaFree(d_data_);
        if (d_ranges_) cudaFree(d_ranges_);
        if (d_intensities_) cudaFree(d_intensities_);
#endif
    }
    
    void RadarProcessor::radarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Create LaserScan message
        sensor_msgs::msg::LaserScan laser_scan;
        
        // Process the pointcloud data
        processRadarData(*msg, laser_scan);
        
        RCLCPP_DEBUG(this->get_logger(), "Processed LaserScan with %zu ranges", laser_scan.ranges.size());
        // Publish the laserscan
        laserscan_pub_->publish(laser_scan);
    }

    void RadarProcessor::processRadarData(const sensor_msgs::msg::PointCloud2 & radar_data, 
                                         sensor_msgs::msg::LaserScan & laser_scan)
    {
        // Initialize LaserScan message header
        laser_scan.header = radar_data.header;
        laser_scan.header.frame_id = frame_id_;
        
        // Set LaserScan parameters
        laser_scan.angle_min = angle_min_;
        laser_scan.angle_max = angle_max_;
        laser_scan.angle_increment = angle_increment_;
        laser_scan.time_increment = 0.0;
        laser_scan.scan_time = 0.0;
        laser_scan.range_min = range_min_;
        laser_scan.range_max = range_max_;
        
        // Calculate number of laser beams
        size_t num_beams = std::ceil((angle_max_ - angle_min_) / angle_increment_);
        laser_scan.ranges.resize(num_beams, std::numeric_limits<float>::infinity());
        laser_scan.intensities.resize(num_beams, 0.0);
        
        // Check if intensity field exists
        bool has_intensity = false;
        int intensity_offset = -1;
        for (size_t i = 0; i < radar_data.fields.size(); ++i) {
            if (radar_data.fields[i].name == "intensity") {
                has_intensity = true;
                intensity_offset = radar_data.fields[i].offset;
                break;
            }
        }
        
        // Get field offsets for x, y, z
        int x_offset = -1, y_offset = -1, z_offset = -1;
        for (size_t i = 0; i < radar_data.fields.size(); ++i) {
            if (radar_data.fields[i].name == "x") x_offset = radar_data.fields[i].offset;
            if (radar_data.fields[i].name == "y") y_offset = radar_data.fields[i].offset;
            if (radar_data.fields[i].name == "z") z_offset = radar_data.fields[i].offset;
        }
        
        // Calculate number of points
        size_t point_step = radar_data.point_step;
        size_t num_points = radar_data.width * radar_data.height;
        
#ifdef USE_CUDA
        // Use CUDA GPU processing if available
        if (use_cuda_) {
            // Allocate or reallocate device memory if needed
            size_t data_size = radar_data.data.size();
            size_t ranges_size = num_beams;
            
            if (data_size > d_data_size_) {
                if (d_data_) cudaFree(d_data_);
                cudaMalloc(&d_data_, data_size);
                d_data_size_ = data_size;
            }
            
            if (ranges_size > d_ranges_size_) {
                if (d_ranges_) cudaFree(d_ranges_);
                if (d_intensities_) cudaFree(d_intensities_);
                cudaMalloc(&d_ranges_, ranges_size * sizeof(float));
                cudaMalloc(&d_intensities_, ranges_size * sizeof(float));
                d_ranges_size_ = ranges_size;
            }
            
            // Copy input data to device
            cudaMemcpy(d_data_, radar_data.data.data(), data_size, cudaMemcpyHostToDevice);
            
            // Initialize device arrays with infinity and zeros
            std::vector<float> inf_ranges(num_beams, std::numeric_limits<float>::infinity());
            std::vector<float> zero_intensities(num_beams, 0.0f);
            cudaMemcpy(d_ranges_, inf_ranges.data(), num_beams * sizeof(float), cudaMemcpyHostToDevice);
            cudaMemcpy(d_intensities_, zero_intensities.data(), num_beams * sizeof(float), cudaMemcpyHostToDevice);
            
            // Launch CUDA kernel
            radar_cuda::processPointcloudCUDA(
                d_data_,
                num_points,
                point_step,
                x_offset,
                y_offset,
                has_intensity ? intensity_offset : -1,
                d_ranges_,
                d_intensities_,
                num_beams,
                static_cast<float>(angle_min_),
                static_cast<float>(angle_max_),
                static_cast<float>(angle_increment_),
                static_cast<float>(range_min_),
                static_cast<float>(range_max_),
                enable_intensity_filter_,
                static_cast<float>(intensity_threshold_)
            );
            
            // Copy results back to host
            cudaMemcpy(laser_scan.ranges.data(), d_ranges_, num_beams * sizeof(float), cudaMemcpyDeviceToHost);
            cudaMemcpy(laser_scan.intensities.data(), d_intensities_, num_beams * sizeof(float), cudaMemcpyDeviceToHost);
            
            return;
        }
#endif
        
        // CPU fallback: Create temporary storage for each thread to avoid contention
        std::vector<float> local_ranges(num_beams, std::numeric_limits<float>::infinity());
        std::vector<float> local_intensities(num_beams, 0.0);
        
        // Process points in parallel using OpenMP with reduced critical sections
        #pragma omp parallel
        {
            // Thread-local storage to minimize critical section usage
            std::vector<float> thread_ranges(num_beams, std::numeric_limits<float>::infinity());
            std::vector<float> thread_intensities(num_beams, 0.0);
            
            #pragma omp for nowait
            for (size_t i = 0; i < num_points; ++i) {
                size_t point_offset = i * point_step;
                
                // Extract x, y coordinates
                float x = *reinterpret_cast<const float*>(&radar_data.data[point_offset + x_offset]);
                float y = *reinterpret_cast<const float*>(&radar_data.data[point_offset + y_offset]);
                
                // Calculate 2D range (ignoring z component for 2D scan)
                float range = std::sqrt(x * x + y * y);
                
                // Filter out points outside range limits
                if (range < range_min_ || range > range_max_) {
                    continue;
                }
                
                // Apply intensity filter if enabled
                float intensity = 0.0;
                if (has_intensity) {
                    intensity = *reinterpret_cast<const float*>(&radar_data.data[point_offset + intensity_offset]);
                    if (enable_intensity_filter_ && intensity < intensity_threshold_) {
                        continue;  // Skip low intensity points
                    }
                }
                
                // Calculate angle (atan2 returns angle in range [-pi, pi])
                float angle = std::atan2(y, x);
                
                // Check if angle is within scan range
                if (angle < angle_min_ || angle > angle_max_) {
                    continue;
                }
                
                // Calculate beam index
                int beam_index = static_cast<int>((angle - angle_min_) / angle_increment_);
                
                // Ensure beam index is within valid range
                if (beam_index >= 0 && beam_index < static_cast<int>(num_beams)) {
                    // Update thread-local storage (no locking needed)
                    if (range < thread_ranges[beam_index]) {
                        thread_ranges[beam_index] = range;
                        if (has_intensity) {
                            thread_intensities[beam_index] = intensity;
                        }
                    }
                }
            }
            
            // Merge thread results into local storage (single critical section per thread)
            #pragma omp critical
            {
                for (size_t j = 0; j < num_beams; ++j) {
                    if (thread_ranges[j] < local_ranges[j]) {
                        local_ranges[j] = thread_ranges[j];
                        local_intensities[j] = thread_intensities[j];
                    }
                }
            }
        }
        
        // Copy results to output
        laser_scan.ranges = local_ranges;
        laser_scan.intensities = local_intensities;
    }

} // namespace radar_processing
