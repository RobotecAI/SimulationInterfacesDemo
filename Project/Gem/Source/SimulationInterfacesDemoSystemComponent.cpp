
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include "SimulationInterfacesDemoSystemComponent.h"

namespace SimulationInterfacesDemo
{
    void SimulationInterfacesDemoSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<SimulationInterfacesDemoSystemComponent, AZ::Component>()
                ->Version(0)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<SimulationInterfacesDemoSystemComponent>("SimulationInterfacesDemo", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void SimulationInterfacesDemoSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("SimulationInterfacesDemoService"));
    }

    void SimulationInterfacesDemoSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("SimulationInterfacesDemoService"));
    }

    void SimulationInterfacesDemoSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void SimulationInterfacesDemoSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    SimulationInterfacesDemoSystemComponent::SimulationInterfacesDemoSystemComponent()
    {
        if (SimulationInterfacesDemoInterface::Get() == nullptr)
        {
            SimulationInterfacesDemoInterface::Register(this);
        }
    }

    SimulationInterfacesDemoSystemComponent::~SimulationInterfacesDemoSystemComponent()
    {
        if (SimulationInterfacesDemoInterface::Get() == this)
        {
            SimulationInterfacesDemoInterface::Unregister(this);
        }
    }

    void SimulationInterfacesDemoSystemComponent::Init()
    {
    }

    void SimulationInterfacesDemoSystemComponent::Activate()
    {
        SimulationInterfacesDemoRequestBus::Handler::BusConnect();
    }

    void SimulationInterfacesDemoSystemComponent::Deactivate()
    {
        SimulationInterfacesDemoRequestBus::Handler::BusDisconnect();
    }
}
