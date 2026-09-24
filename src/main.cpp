#include "MC_PULL_calibration.h"
#include "ws2812.h"

#include "Flash_saves.h"
#include "Motion_control.h"
#include "_bus_hardware.h"
#include "ams.h"
#include "ahub_bus.h"
#include "bambu_bus_ams.h"
#include "ADC_DMA.h"
#include "Debug_log.h"
#include "nvm_save_sched.h"
#include "boot_restore.h"
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

static nvm_job g_fil_job[4];
static uint8_t g_loaded_ch = 0xFF;
static nvm_job g_state_job;
static nvm_wait g_nvm_wait;
static boot_restore_t g_boot_restore = {0xFFu}; // loaded channel restored at boot
static constexpr bool kRestoreAtBoot = (BMCU_BOOT_RESTORE_LOADED != 0);

static inline void ram_to_flashinfo(uint8_t fil, Flash_FilamentInfo* o)
{
    const _filament* f = &ams[BAMBU_BUS_AMS_NUM].filament[fil];

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
    _filament* f = &ams[BAMBU_BUS_AMS_NUM].filament[fil];

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
    const uint32_t now = time_ticks32();
    for (uint8_t i = 0; i < 4u; i++)
        nvm_job_changed(&g_fil_job[i], now);
}

void ams_datas_set_need_to_save_filament(uint8_t filament_idx)
{
    if (filament_idx >= 4u) return;
    nvm_job_changed(&g_fil_job[filament_idx], time_ticks32());
}

void ams_state_set_loaded(uint8_t filament_ch)
{
    if (filament_ch >= 4u) return;
    if (g_loaded_ch != 0xFFu) return;
    g_loaded_ch = filament_ch;
    nvm_job_changed(&g_state_job, time_ticks32());
}

void ams_state_set_unloaded(uint8_t filament_ch)
{
    if (g_loaded_ch == 0xFFu) return;
    if (filament_ch < 4u && g_loaded_ch != filament_ch) return;
    g_loaded_ch = 0xFFu;
    nvm_job_changed(&g_state_job, time_ticks32());

    // On any path: a restored in-use state the printer has not referenced yet goes with it.
    boot_restore_on_unloaded(&g_boot_restore, &ams[BAMBU_BUS_AMS_NUM], kRestoreAtBoot);
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
    boot_restore_on_command(&g_boot_restore, &ams[BAMBU_BUS_AMS_NUM], kRestoreAtBoot, read_num, statu_flags,
                            motion_flag);
}

static void ams_state_save_run(uint32_t now)
{
    nvm_job_result(&g_state_job, Flash_AMS_state_write(g_loaded_ch), now);
}

static void ams_datas_save_run(uint8_t fil, uint32_t now)
{
    Flash_FilamentInfo info;
    ram_to_flashinfo(fil, &info);

    nvm_job_result(&g_fil_job[fil], Flash_AMS_filament_write(fil, &info), now);
}

// Called on every main-loop pass: runs at most one NVM job, and only in a quiet bus window
// (nvm_save_sched.h), never in the pass that has just started a reply.
static void ams_nvm_save_run()
{
    const bool rx_idle = bus_port_to_host.rx_idle(time_ticks32());
    const bool tx_idle = bus_port_to_host.tx_idle();
    const uint32_t now = time_ticks32(); // after the idle checks, before the stamps
    const nvm_bus b = nvm_bus_from_samples(rx_idle, tx_idle, now, bus_port_to_host.last_rx_tick(),
                                           bus_port_to_host.last_tx_end_tick());

    const int job = nvm_pick_job(&g_state_job, g_fil_job, &g_nvm_wait, &b, now);
    if (job == NVM_JOB_STATE)
        ams_state_save_run(now);
    else if (job != NVM_JOB_NONE)
        ams_datas_save_run((uint8_t)job, now);
}

int main(void)
{
    SystemInit();
    SystemCoreClockUpdate();
    time_hw_init();

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

    DEBUG_init();
    ams_init();
    Flash_saves_init();

    ADC_DMA_init();
    ADC_DMA_wait_full();

    MC_PULL_calibration_boot();
    ams_datas_read();

    {
        uint8_t ch = 0xFFu;
        if (Flash_AMS_state_read(&ch))
        {
            g_loaded_ch = ch;

            // In use from boot (BMCU_BOOT_RESTORE_LOADED=1), or once the printer references it (0).
            boot_restore_init(&g_boot_restore, &ams[BAMBU_BUS_AMS_NUM], ch, kRestoreAtBoot);
        }
    }

    Motion_control_init();
    bambubus_init();
    bus_init();

    DEBUG("START\n");

    while (1)
    {
        const ahubus_package_type   ahub_stu     = ahubus_run();
        const bambubus_package_type bambubus_stu = bambubus_run();
        bus_port_to_host.send_package();

        if (bambubus_stu == bambubus_package_type::heartbeat)
            bus_host_device_type = host_device_type_ams;

        if (ahub_stu == ahubus_package_type::heartbeat)
        {
            bus_host_device_type = host_device_type_ahub;
            boot_restore_drop(&g_boot_restore); // the AHUB host sets the channel states itself
        }

        // Only the protocol the host speaks decides: the other one never gets a heartbeat and must
        // not mask a lost link. Before the first heartbeat the BMCU stays offline, motors stopped.
        bool offline = true;
        if (bus_host_device_type == host_device_type_ams)
            offline = (bambubus_stu == bambubus_package_type::error);
        else if (bus_host_device_type == host_device_type_ahub)
            offline = (ahub_stu == ahubus_package_type::error);

        int error = 0;

        if (!offline)
        {
            if (bambubus_stu == bambubus_package_type::heartbeat)
                SYS_RGB.set_RGB(0x38, 0x35, 0x32, 0);

            // Only while the host link is up, as before: pending writes wait out an offline spell.
            ams_nvm_save_run();
        }
        else
        {
            error = -1;
            SYS_RGB.set_RGB(0x10, 0x00, 0x00, 0);
        }

        Motion_control_run(error);
        RGB_update();
    }
}