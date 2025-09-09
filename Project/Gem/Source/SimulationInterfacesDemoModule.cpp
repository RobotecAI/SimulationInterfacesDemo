
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "SimulationInterfacesDemoSystemComponent.h"

namespace SimulationInterfacesDemo
{
    class SimulationInterfacesDemoModule
        : public AZ::Module
    {
    public:
        AZ_RTTI(SimulationInterfacesDemoModule, "{7FF968EA-F342-4BF4-951F-FDF53B6A17A9}", AZ::Module);
        AZ_CLASS_ALLOCATOR(SimulationInterfacesDemoModule, AZ::SystemAllocator, 0);

        SimulationInterfacesDemoModule()
            : AZ::Module()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                SimulationInterfacesDemoSystemComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<SimulationInterfacesDemoSystemComponent>(),
            };
        }
    };
}// namespace SimulationInterfacesDemo

AZ_DECLARE_MODULE_CLASS(Gem_SimulationInterfacesDemo, SimulationInterfacesDemo::SimulationInterfacesDemoModule)
