# Isaac ROS Perception Pipeline Configurations

This directory contains sample configuration files for various Isaac ROS perception pipeline scenarios. These configurations demonstrate different approaches to perception system setup, optimization, and integration.

## Available Configurations

### 1. Basic Perception Pipeline
**File**: `basic_perception.yaml`

**Description**: A simple perception pipeline for object detection using YOLOv4-tiny with GPU acceleration.

**Key Features**:
- Basic image preprocessing (resize, normalize, color correction)
- YOLOv4-tiny object detection
- GPU acceleration with memory pooling
- Performance monitoring and logging
- ROS 2 integration with QoS settings

**Use Case**: Ideal for beginners or simple object detection tasks.

### 2. Advanced VSLAM Configuration
**File**: `advanced_vslam.yaml`

**Description**: An optimized VSLAM pipeline for indoor environments with loop closure detection.

**Key Features**:
- ORB feature detection with optimized parameters
- Loop closure detection and bundle adjustment
- Multi-threaded processing with GPU acceleration
- Adaptive environment parameters
- Comprehensive health monitoring

**Use Case**: Suitable for indoor navigation and mapping applications.

### 3. Perception-Navigation Integration
**File**: `perception_navigation.yaml`

**Description**: Configuration for bridging perception data with navigation systems.

**Key Features**:
- Costmap integration with obstacle processing
- Dynamic obstacle filtering
- TF frame management
- Safety parameters and emergency handling
- Multiple environment profiles

**Use Case**: Perfect for autonomous navigation applications requiring perception integration.

### 4. Performance Optimized Pipeline
**File**: `performance_optimized.yaml`

**Description**: High-performance perception pipeline with aggressive GPU acceleration.

**Key Features**:
- TensorRT optimization with FP16 precision
- CUDA graphs and pinned memory
- Multi-threaded processing with thread pools
- Adaptive performance strategies
- Zero-copy transport mechanisms

**Use Case**: Ideal for real-time applications requiring maximum performance.

## Configuration Structure

All configurations follow a similar hierarchical structure:

```yaml
# Main component configuration
component:
  name: "component_name"
  description: "Component description"
  version: "x.y.z"

  # Sub-component configurations
  subcomponent:
    parameter1: value1
    parameter2: value2

  # ROS 2 integration
  ros2:
    node_name: "node_name"
    namespace: "namespace"

  # Performance settings
  performance:
    use_gpu: true/false
    max_fps: number

  # Debug and logging
  debug:
    enabled: true/false
    logging:
      level: "info/warn/error"
```

## Usage Instructions

### Basic Usage

1. **Copy the desired configuration**:
   ```bash
   cp config/perception-pipelines/basic_perception.yaml ~/isaac_ws/src/isaac_tutorials/config/
   ```

2. **Update paths and parameters**:
   - Modify model paths to point to your actual model files
   - Adjust topic names to match your system
   - Tune parameters for your specific use case

3. **Launch with the configuration**:
   ```bash
   ros2 launch isaac_ros_tutorials perception.launch.py config_file:=path/to/your_config.yaml
   ```

### Configuration Tuning

#### Performance Tuning

1. **Start with basic configuration** and verify it works
2. **Gradually increase complexity** by enabling additional features
3. **Monitor performance** using built-in metrics
4. **Adjust parameters** based on your specific requirements

#### Common Parameters to Tune

- **GPU Settings**: `use_gpu`, `gpu_device_id`, memory limits
- **Performance**: `max_fps`, `batch_size`, threading settings
- **Model Parameters**: confidence thresholds, NMS settings
- **Image Processing**: resolution, normalization parameters

### Integration with Launch Files

Example launch file snippet:

```python
# In your launch file
vslam_config = LaunchConfiguration('vslam_config')

declare_vslam_config = DeclareLaunchArgument(
    'vslam_config',
    default_value=os.path.join(isaac_ros_dir, 'config', 'perception-pipelines', 'advanced_vslam.yaml'),
    description='Path to VSLAM configuration file'
)

vslam_node = Node(
    package='isaac_ros_visual_slam',
    executable='isaac_ros_vslam',
    name='isaac_ros_vslam',
    parameters=[vslam_config]
)
```

## Configuration Best Practices

### 1. Start Simple
- Begin with minimal configuration
- Verify basic functionality before adding complexity
- Gradually enable advanced features

### 2. Use Version Control
- Track configuration changes with git
- Document parameter changes and their effects
- Maintain different profiles for different scenarios

### 3. Monitor Performance
- Enable performance monitoring
- Set appropriate alert thresholds
- Log metrics for analysis

### 4. Validate Configurations
- Use configuration validation tools
- Test in simulation before hardware deployment
- Implement safety checks and limits

### 5. Document Changes
- Maintain changelogs for configurations
- Document parameter tuning rationale
- Keep records of performance benchmarks

## Configuration Reference

### Common Parameters

| Parameter | Type | Description | Typical Values |
|-----------|------|-------------|----------------|
| `use_gpu` | bool | Enable GPU acceleration | `true`, `false` |
| `max_fps` | float | Maximum frames per second | `15.0`, `30.0`, `60.0` |
| `confidence_threshold` | float | Detection confidence threshold | `0.3` to `0.7` |
| `nms_threshold` | float | Non-max suppression threshold | `0.3` to `0.5` |
| `enable_memory_pooling` | bool | Enable memory pooling | `true`, `false` |

### Performance Parameters

| Parameter | Description | Impact |
|-----------|-------------|--------|
| `gpu_device_id` | GPU device to use | Performance, memory usage |
| `num_worker_threads` | Number of worker threads | Throughput, CPU usage |
| `batch_size` | Batch size for processing | Latency, memory usage |
| `enable_fp16` | Enable FP16 precision | Performance, accuracy |
| `enable_tensor_cores` | Use tensor cores | Performance |

### Safety Parameters

| Parameter | Description | Recommended Values |
|-----------|-------------|-------------------|
| `max_gpu_memory` | Maximum GPU memory | 70-80% of available |
| `max_cpu_usage` | Maximum CPU usage | 70-90% |
| `auto_reset` | Auto reset on failure | `true` for production |
| `restart_delay` | Restart delay | `3.0` to `10.0` seconds |

## Troubleshooting

### Common Configuration Issues

1. **File Not Found Errors**:
   - Verify file paths are correct
   - Check file permissions
   - Use absolute paths for reliability

2. **Parameter Validation Errors**:
   - Check parameter types and ranges
   - Validate YAML syntax
   - Use configuration validation tools

3. **Performance Issues**:
   - Monitor GPU/CPU usage
   - Adjust batch sizes and threading
   - Reduce image resolution

4. **Memory Issues**:
   - Enable memory pooling
   - Reduce cache sizes
   - Limit concurrent pipelines

### Debugging Tools

```bash
# Validate YAML syntax
python3 -c "import yaml; yaml.safe_load(open('config.yaml'))"

# Check file paths
ls -la /path/to/your/config.yaml

# Monitor GPU usage
nvidia-smi --loop=1

# Check ROS 2 parameters
ros2 param list
ros2 param get /node_name parameter_name
```

## Advanced Configuration

### Environment-Specific Profiles

Create different profiles for various environments:

```yaml
profiles:
  indoor:
    feature_detector: "ORB"
    n_features: 1500
    lighting: "normal"

  outdoor:
    feature_detector: "SIFT"
    n_features: 2000
    lighting: "high"

  low_light:
    feature_detector: "ORB"
    n_features: 1200
    use_denoising: true
```

### Adaptive Configuration

Implement adaptive parameters that change based on conditions:

```yaml
adaptive:
  enabled: true
  update_interval: 10

  strategies:
    - name: "reduce_resolution"
      trigger: "high_latency"
      condition: "latency > 0.05"
      action: "reduce_resolution"
      params: {scale: 0.8}
```

### Configuration Inheritance

Use configuration inheritance to build on existing profiles:

```yaml
# Base configuration
base_config: "basic_perception.yaml"

# Overrides
overrides:
  performance:
    max_fps: 30
  model:
    confidence_threshold: 0.6
```

## Contributing

### Adding New Configurations

1. **Create a new YAML file** following the established structure
2. **Document the configuration** in this README
3. **Test thoroughly** in various scenarios
4. **Add examples** of usage
5. **Update the reference** section

### Configuration Guidelines

- Follow consistent naming conventions
- Use clear, descriptive parameter names
- Include comprehensive comments
- Provide example values
- Document performance implications

## License

These configuration files are provided under the same license as the main project. Refer to the project's LICENSE file for details.

## Support

For issues or questions regarding these configurations:

1. Check the main project documentation
2. Review the troubleshooting section
3. Consult the official Isaac ROS documentation
4. Ask on community forums with specific details

Include relevant configuration snippets and error messages when seeking help.