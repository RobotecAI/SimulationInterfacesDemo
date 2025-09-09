
#pragma once

#include <AzCore/Component/Component.h>

#include <SimulationInterfacesDemo/SimulationInterfacesDemoBus.h>

namespace SimulationInterfacesDemo
{
    class SimulationInterfacesDemoSystemComponent
        : public AZ::Component
        , protected SimulationInterfacesDemoRequestBus::Handler
    {
    public:
        AZ_COMPONENT(SimulationInterfacesDemoSystemComponent, "{3AF4793C-1EDD-4298-B58A-992F83101644}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        SimulationInterfacesDemoSystemComponent();
        ~SimulationInterfacesDemoSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // SimulationInterfacesDemoRequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////
    };
}
