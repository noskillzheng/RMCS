#include <cmath>
#include <numbers>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::example {

class SineCosineGenerator
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SineCosineGenerator()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        
        // 从配置文件读取角频率 omega (rad/s)
        omega_ = get_parameter("omega").as_double();
        
        // 注册输入接口：获取时间戳和更新计数
        register_input("/predefined/timestamp", timestamp_);
        register_input("/predefined/update_count", update_count_);
        
        // 注册输出接口
        register_output("/example/sine", sine_output_, 0.0);
        register_output("/example/cosine", cosine_output_, 0.0);
        
        RCLCPP_INFO(get_logger(), "SineCosineGenerator initialized with omega = %.2f rad/s", omega_);
    }
    
    ~SineCosineGenerator() = default;
    
    void update() override {
        // 计算时间 t (秒)
        // 注意：RMCS更新频率为1000Hz，所以每次增加 0.001秒
        // 使用 update_count 来计算时间更准确
        double t = static_cast<double>(*update_count_) / 1000.0;
        
        // 计算 sin(ωt) 和 cos(ωt)
        *sine_output_ = std::sin(omega_ * t);
        *cosine_output_ = std::cos(omega_ * t);
    }

private:
    double omega_;  // 角频率 (rad/s)
    
    // 输入接口
    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    InputInterface<size_t> update_count_;
    
    // 输出接口
    OutputInterface<double> sine_output_;
    OutputInterface<double> cosine_output_;
};

} // namespace rmcs_core::controller::example

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::example::SineCosineGenerator, 
    rmcs_executor::Component)

