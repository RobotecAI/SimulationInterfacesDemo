#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/vector.h>
#include <ImGuiBus.h>

#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

#include <rclcpp/publisher.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace SimulationInterfacesDemo
{

    class SampleComponent
        : public AZ::Component
        , public ImGui::ImGuiUpdateListenerBus::Handler
    {
    public:
        AZ_COMPONENT(SampleComponent, "{FF2907A3-AB67-4B61-A55A-FA337770D1D3}", AZ::Component);
        SampleComponent();
        ~SampleComponent() = default;
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

    private:
        void OnImGuiUpdate() override;

        AZStd::vector<AZ::EntityId> m_goalEntities;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> m_goalPublisher;
        geometry_msgs::msg::PoseStamped m_goalMessage;
        ROS2::TopicConfiguration m_goalTopicConfiguration;
    };
} // namespace SimulationInterfacesDemo
