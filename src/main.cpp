#include "MC_PULL_calibration.h"
#include "ws2812.h"

#include "Flash_saves.h"
#include "Motion_control.h"
#include "ams_addressing.h"
#include "_bus_hardware.h"
#include "ams.h"
#include "ahub_bus.h"
#include "bambu_bus_ams.h"
#include "ADC_DMA.h"
#include "Debug_log.h"
#include "boot_restore.h"
#include "watchdog.h"
#include <string.h>

WS2812_class SYS_RGB;
WS2812_class RGBOUT[4];

void RGB_init()
{
    SYS_RGB.init(1, GPIOD, GPIO_Pin_1);
    RGBOUT[0].init(2, GPIOA, GPIO_Pin_11);
    RGBOUT[1].init(2, GPIOA, GPIO_Pin_8);
    RGBOUT[2].init(2, GPIOB, GPIO_Pin_1);
    RGBOUT[3].init(2, GPIOB, GPIO_Pin_0);
}

void RGB_update()
{
    if (!(SYS_RGB.is_dirty() ||
          RGBOUT[0].is_dirty() || RGBOUT[1].is_dirty() ||
          RGBOUT[2].is_dirty() || RGBOUT[3].is_dirty()))
        return;

    // Before the throttle: ws2812_throttle_due() stamps it when it returns true, and a pass in which
    // WS2812_class::updata() cannot draw (the bus busy, or the flash clock slowed by a background
    // flash step) must not use up the gap.
    if (time_hw_slow || !bus_background_ready()) return;

    static ws2812_throttle_t throttle = {0u, 0u};

    uint32_t min_gap = time_hw_tpms * 10u;
    if (!min_gap) min_gap = 1u;

    // This gap is also every strip's WS2812 reset time (ws2812_throttle_t), so updata() does not wait.
    const uint32_t now = time_ticks32();
    if (!ws2812_throttle_due(&throttle, now, min_gap))
        return;

    SYS_RGB.updata();
    RGBOUT[0].updata();
    RGBOUT[1].updata();
    RGBOUT[2].updata();
    RGBOUT[3].updata();
}

static uint8_t g_fil_dirty = 0;
static uint8_t g_loaded_ch = 0xFF;
static uint8_t g_state_dirty = 0;
static uint8_t g_state_write_value = 0xFF;
static uint8_t g_loaded_flash_pending = 0;
static boot_restore_t g_boot_restore = {0xFFu}; // loaded channel restored at boot
static constexpr bool kRestoreAtBoot = (BMCU_BOOT_RESTORE_LOADED != 0);

static inline void ram_to_flashinfo(uint8_t fil, Flash_FilamentInfo* o)
{
    const _filament* f = &ams[BMCU_LOCAL_AMS_INDEX].filament[fil];

    memcpy(o->bambubus_filament_id, f->bambubus_filament_id, sizeof(o->bambubus_filament_id));
    o->color_R = f->color_R;
    o->color_G = f->color_G;
    o->color_B = f->color_B;
    o->color_A = f->color_A;
    o->temperature_min = f->temperature_min;
    o->temperature_max = f->temperature_max;
    memcpy(o->name, f->name, sizeof(o->name));
}

static inline void flashinfo_to_ram(uint8_t fil, const Flash_FilamentInfo* i)
{
    _filament* f = &ams[BMCU_LOCAL_AMS_INDEX].filament[fil];

    memcpy(f->bambubus_filament_id, i->bambubus_filament_id, sizeof(i->bambubus_filament_id));
    f->color_R = i->color_R;
    f->color_G = i->color_G;
    f->color_B = i->color_B;
    f->color_A = i->color_A;
    f->temperature_min = i->temperature_min;
    f->temperature_max = i->temperature_max;

    memset(f->name, 0, sizeof(f->name));
    memcpy(f->name, i->name, sizeof(i->name));
    f->name[sizeof(f->name) - 1u] = 0;
}

bool ams_datas_read()
{
    bool any = false;

    for (uint8_t fil = 0; fil < 4u; fil++)
    {
        Flash_FilamentInfo fi;
        if (Flash_AMS_filament_read(fil, &fi))
        {
            flashinfo_to_ram(fil, &fi);
            any = true;
        }
    }

    return any;
}

void ams_datas_set_need_to_save()
{
    g_fil_dirty = 0x0Fu;
}

void ams_datas_set_need_to_save_filament(uint8_t filament_idx)
{
    if (filament_idx >= 4u) return;
    g_fil_dirty |= (uint8_t)(1u << filament_idx);
}

void ams_state_claim_loaded(uint8_t filament_ch)
{
    if (filament_ch >= 4u) return;
    if (g_loaded_ch != 0xFFu && g_loaded_ch != filament_ch) return;

    if (g_loaded_ch == 0xFFu)
    {
        g_loaded_ch = filament_ch;
        g_loaded_flash_pending = 1u;
    }
}

void ams_state_set_loaded(uint8_t filament_ch)
{
    if (filament_ch >= 4u) return;
    if (g_loaded_ch != 0xFFu && g_loaded_ch != filament_ch) return;

    if (g_loaded_ch == 0xFFu)
    {
        g_loaded_ch = filament_ch;
        g_loaded_flash_pending = 1u;
    }

    if (!g_loaded_flash_pending) return;

    g_state_write_value = filament_ch;
    g_state_dirty = 1u;
    g_loaded_flash_pending = 0u;
}

void ams_state_set_unloaded(uint8_t filament_ch)
{
    if (g_loaded_ch == 0xFFu) return;
    if (filament_ch < 4u && g_loaded_ch != filament_ch) return;
    g_loaded_ch = 0xFFu;
    g_loaded_flash_pending = 0u;
    g_state_write_value = 0xFFu;
    g_state_dirty = 1u;

    // On any path: a restored in-use state the printer has not referenced yet goes with it.
    boot_restore_on_unloaded(&g_boot_restore, &ams[BMCU_LOCAL_AMS_INDEX], kRestoreAtBoot);
}

uint8_t ams_state_get_loaded(void)
{
    return g_loaded_ch;
}

// Option 0 only: the loaded channel from the STA record is still waiting for the printer's first
// reference, so RAM does not show it in use yet (boot_restore.h).
bool ams_state_boot_restore_deferred(void)
{
    return !kRestoreAtBoot && boot_restore_pending(&g_boot_restore);
}

// set_motion, before it acts on a printer motion command (boot_restore.h).
void ams_state_printer_command(uint8_t read_num, uint8_t statu_flags, uint8_t motion_flag)
{
    boot_restore_on_command(&g_boot_restore, &ams[BMCU_LOCAL_AMS_INDEX], kRestoreAtBoot, read_num, statu_flags,
                            motion_flag);
}

// The two saves below run from the main loop while bus_background_ready(). Flash_AMS_state_write /
// Flash_AMS_filament_write only queue a background flash job (Flash_saves.h) that
// Flash_background_run() carries out one step per pass; they return true once flash holds the
// value, and only then is the dirty flag cleared. Until then every pass asks again.
static void ams_state_save_run()
{
    if (!g_state_dirty) return;

    if (Flash_AMS_state_write(g_state_write_value))
        g_state_dirty = 0u;
}

void ams_datas_save_run()
{
    if (!g_fil_dirty) return;

    uint8_t fil = 0xFFu;
    for (uint8_t i = 0; i < 4u; i++)
    {
        if (g_fil_dirty & (uint8_t)(1u << i))
        {
            fil = i;
            break;
        }
    }

    if (fil == 0xFFu) return;

    Flash_FilamentInfo now;
    ram_to_flashinfo(fil, &now);

    if (Flash_AMS_filament_write(fil, &now))
        g_fil_dirty &= (uint8_t)~(1u << fil);
}

// After a watchdog reset (a hang, or a trap: watchdog.cpp) the SYS LED flashes magenta before the
// boot goes on as usual (WDG_BOOT_FLASHES x WDG_BOOT_FLASH_MS on and off, 420 ms), and after a trap
// blue once more (490 ms; blackbox.h). The SYS LED is otherwise red, white-ish or off, never blue.
static_assert(wdg_boot_flash_ms(true) <= WDG_BOOT_FLASH_MAX_MS, "boot flash too long");

static void show_watchdog_reset(bool trap)
{
    for (uint32_t k = 0u; k < WDG_BOOT_FLASHES; k++)
    {
        SYS_RGB.set_RGB(0x10, 0x00, 0x10, 0);
        RGB_update();
        delay(WDG_BOOT_FLASH_MS);
        SYS_RGB.set_RGB(0x00, 0x00, 0x00, 0);
        RGB_update();
        delay(WDG_BOOT_FLASH_MS);
    }

    if (trap)
    {
        SYS_RGB.set_RGB(0x00, 0x00, 0x10, 0);
        RGB_update();
        delay(WDG_BOOT_FLASH_MS);
    }

    SYS_RGB.set_RGB(0x10, 0x00, 0x00, 0);
    RGB_update();
}

int main(void)
{
    SystemInit();
    SystemCoreClockUpdate();
    time_hw_init();

    const uint32_t reset_flags = watchdog_reset_flags_take();
    blackbox_boot(reset_flags); // phase BOOT; right after the clock setup: a trap from here on is recorded

    __enable_irq();

    WWDG_DeInit();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, DISABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);

    RGB_init();
    delay(10);

    SYS_RGB.set_RGB(0x10, 0x00, 0x00, 0);
    for (int i = 0; i < 4; i++) RGBOUT[i].set_RGB(0, 0, 0, 0);
    RGB_update();
    delay(50);

    const blackbox_flash boot_flash = blackbox_boot_flash(&g_blackbox_last, reset_flags);
    if (boot_flash != BLACKBOX_FLASH_NONE) show_watchdog_reset(boot_flash == BLACKBOX_FLASH_TRAP);

    DEBUG_init();
    ams_init();
    Flash_saves_init();

    ADC_DMA_init();
    ADC_DMA_wait_full();

    blackbox_phase_set(BLACKBOX_PHASE_CALIBRATION);
    MC_PULL_calibration_boot();
    blackbox_phase_set(BLACKBOX_PHASE_NVM_READ);

    // Not earlier: the first-boot calibration waits up to 30 s per step (watchdog_cfg.h lists what
    // feeds it from here on).
    watchdog_start();

    ams_datas_read();

    {
        uint8_t ch = 0xFFu;
        if (Flash_AMS_state_read(&ch))
        {
            g_loaded_ch = ch;

            // In use from boot (BMCU_BOOT_RESTORE_LOADED=1), or once the printer references it (0).
            boot_restore_init(&g_boot_restore, &ams[BMCU_LOCAL_AMS_INDEX], ch, kRestoreAtBoot);
        }
    }

    blackbox_phase_set(BLACKBOX_PHASE_MOTION_INIT);
    Motion_control_init();
    // V11: a loaded channel whose switches read empty, or whose calibration is not valid, is not
    // restored: it is set idle and unloaded (which queues the 'none' STA record and, through
    // boot_restore_on_unloaded, drops the pending restore). With BMCU_BOOT_RESTORE_LOADED=0 nothing
    // was restored into RAM, and these lines only write the idle state.
    if (g_loaded_ch < 4u && (!Motion_control_filament_present(g_loaded_ch) ||
        !MC_PULL_calibration_is_valid(g_loaded_ch))) {
        _ams* a = &ams[BMCU_LOCAL_AMS_INDEX];
        a->filament[g_loaded_ch].motion = _filament_motion::idle;
        a->now_filament_num = 0xFFu;
        a->filament_use_flag = 0u;
        a->pressure = 0xF9C6u;
        ams_state_set_unloaded(g_loaded_ch);
    }
    blackbox_phase_set(BLACKBOX_PHASE_BUS_INIT);
    bambubus_init();
    bus_init();

    DEBUG("START\n");

    blackbox_pass_begin(&g_blackbox_pass, time_ticks32());
    while (1)
    {
        watchdog_feed();
        blackbox_pass_mark(time_ticks32());

        blackbox_phase_set(BLACKBOX_PHASE_AHUB);
        bus_rx_poll();
        const ahubus_package_type   ahub_stu     = ahubus_run();
        blackbox_phase_set(BLACKBOX_PHASE_BAMBUBUS);
        const bambubus_package_type bambubus_stu = bambubus_run();
        blackbox_phase_set(BLACKBOX_PHASE_SEND);
        bus_port_to_host.send_package();

        // Only the protocol the host speaks decides (a lost BambuBus link is not masked by an AHUB
        // that never answers, nor the other way round); until the host has been seen the BMCU stays
        // offline, motors stopped. Each protocol latches its own link loss (bus_link.h, in
        // bambubus_run / ahubus_run).
        if (bambubus_stu != bambubus_package_type::none && bambubus_stu != bambubus_package_type::error)
            bus_host_device_type = host_device_type_ams;
        else if (ahub_stu != ahubus_package_type::none && ahub_stu != ahubus_package_type::error)
            bus_host_device_type = host_device_type_ahub;

        if (ahub_stu == ahubus_package_type::heartbeat)
            boot_restore_drop(&g_boot_restore); // the AHUB host sets the channel states itself

        const int error = bus_host_device_type == host_device_type_ams
            ? (bambubus_stu == bambubus_package_type::error ? -1 : 0)
            : bus_host_device_type == host_device_type_ahub
                ? (ahub_stu == ahubus_package_type::error ? -1 : 0) : -1;
        if (bambubus_arrange_wait()) SYS_RGB.set_RGB(0x10, 0x00, 0x00, 0);
        else if (error) SYS_RGB.set_RGB(0x10, 0x00, 0x00, 0);
        else SYS_RGB.set_RGB(0x38, 0x35, 0x32, 0);

        blackbox_phase_set(BLACKBOX_PHASE_NVM);
        if (g_state_dirty && bus_background_ready()) ams_state_save_run();
        if (g_fil_dirty && bus_background_ready()) ams_datas_save_run();

        blackbox_phase_set(BLACKBOX_PHASE_MOTION);
        Motion_control_run(error);
        blackbox_phase_set(BLACKBOX_PHASE_FLASH);
        Flash_background_run();
        blackbox_phase_set(BLACKBOX_PHASE_RGB);
        RGB_update();
    }
}
