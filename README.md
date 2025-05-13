我将原始的代码分成了多个模块（GpsPublisher、GpsSubscriber、GpsConverter），

并将每个模块的逻辑封装成了组件。通过 rclcpp_components进行调用。

pluginlib 定义插件接口，并在运行时加载插件

通过 launch 文件来启动整个系统，而不是在每个文件中都写 main 函数，增加了可配置性和灵活性。

最终完成了模块化

截图在screenshots里边
