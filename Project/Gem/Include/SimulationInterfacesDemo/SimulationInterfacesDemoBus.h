
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace SimulationInterfacesDemo
{
    class SimulationInterfacesDemoRequests
    {
    public:
        AZ_RTTI(SimulationInterfacesDemoRequests, "{595D6C3C-B3F2-48F6-BE9A-1D63A9108855}");
        virtual ~SimulationInterfacesDemoRequests() = default;
        // Put your public methods here
    };

    class SimulationInterfacesDemoBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using SimulationInterfacesDemoRequestBus = AZ::EBus<SimulationInterfacesDemoRequests, SimulationInterfacesDemoBusTraits>;
    using SimulationInterfacesDemoInterface = AZ::Interface<SimulationInterfacesDemoRequests>;

} // namespace SimulationInterfacesDemo
