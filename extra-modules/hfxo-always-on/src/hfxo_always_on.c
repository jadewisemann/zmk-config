/*
 * Keep the nRF 32 MHz crystal running for the lifetime of the firmware.
 * See Kconfig KB_HFXO_ALWAYS_ON for why.
 */
#include <zephyr/init.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/sys/onoff.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hfxo_always_on, LOG_LEVEL_INF);

/* The client must outlive the request, so it is static and never released. */
static struct onoff_client hfxo_client;

static int hfxo_always_on_init(void) {
    struct onoff_manager *mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);

    if (mgr == NULL) {
        LOG_ERR("no HF clock manager");
        return -ENXIO;
    }

    sys_notify_init_spinwait(&hfxo_client.notify);

    int err = onoff_request(mgr, &hfxo_client);
    if (err < 0) {
        LOG_ERR("HFXO request failed: %d", err);
        return err;
    }

    return 0;
}

SYS_INIT(hfxo_always_on_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
