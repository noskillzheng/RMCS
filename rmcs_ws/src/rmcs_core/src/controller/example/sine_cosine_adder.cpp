#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::example {

class SineCosineAdder
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SineCosineAdder()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        
        // 注册输入接口：接收组件A的两个输出
        register_input("/example/sine", sine_input_);
        register_input("/example/cosine", cosine_input_);
        
        // 注册输出接口：输出两者之和
        register_output("/example/sum", sum_output_, 0.0);
        
        RCLCPP_INFO(get_logger(), "SineCosineAdder initialized");
    }
    
    ~SineCosineAdder() = default;
    
    void update() override {
        // 计算 sin(ωt) + cos(ωt)
        *sum_output_ = *sine_input_ + *cosine_input_;
    }

private:
    // 输入接口
    InputInterface<double> sine_input_;
    InputInterface<double> cosine_input_;
    
    // 输出接口
    OutputInterface<double> sum_output_;
};

} // namespace rmcs_core::controller::example

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::example::SineCosineAdder, 
    rmcs_executor::Component)

