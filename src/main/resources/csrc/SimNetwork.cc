#include <vpi_user.h>
#include <svdpi.h>

#include "device.h"
#include "switch.h"

// TODO(sibanez): consolidate with LNICConsts
#define NIC_MAC_ADDR 0x081122334408

class NetworkSwitch *netsw = NULL;
class NetworkDevice *netdev = NULL;

extern "C" void network_init(const char *devname)
{
    netsw = new NetworkSwitch(devname);
    netdev = new NetworkDevice(NIC_MAC_ADDR);

    netsw->add_device(netdev);
}

extern "C" void network_tick(
        unsigned char out_valid,
        unsigned char *out_ready,
        long long     out_data,
        unsigned char out_last,

        unsigned char *in_valid,
        unsigned char in_ready,
        long long     *in_data,
        unsigned char *in_last)
{
    if (!netdev || !netsw) {
        *out_ready = 0;
        *in_valid = 0;
        *in_data = 0;
        *in_last = 0;
        return;
    }

    netdev->tick(out_valid, out_data, out_last, in_ready);
    netdev->switch_to_host();

    netsw->distribute();
    netsw->switch_to_worker();

    *out_ready = netdev->out_ready();
    *in_valid = netdev->in_valid();
    *in_data = netdev->in_data();
    *in_last = netdev->in_last();
}
