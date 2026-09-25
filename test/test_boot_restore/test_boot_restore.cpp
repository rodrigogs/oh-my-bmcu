// Host tests for src/boot_restore.h: the loaded channel the BMCU restores at boot (NVM STA record)
// and what it reports for it until the printer references it. They run the header through copies
// of its callers: main.cpp's loaded-channel state (ams_state_*), the empty-switch check at the top
// of Motion_control_run, and set_motion from bambu_bus_ams.cpp, so the order in which the hooks run
// inside a printer command is covered too.
//
// BMCU_BOOT_RESTORE_LOADED=1 (default): the restored channel is reported in use from boot, as
// before, and still is after any number of idle polls; but when its switches read empty before the
// printer has referenced it, the restored state is undone and the channel reads idle. Once the
// printer has referenced it (a resume, a load, an unload), nothing changes: after a runout the
// printer's 0xFF/0x03 must still pull the channel back.
// BMCU_BOOT_RESTORE_LOADED=0 (A/B image for #148): idle from boot, g_loaded_ch kept, and from the
// printer's first reference on the same state, command by command, as with 1.

#include <stdint.h>
#include <string.h>
#include <unity.h>

#define BAMBU_BUS_AMS_NUM 0

#include "boot_restore.h"

// ---- Host stand-ins for the firmware globals set_motion uses ----
_ams ams[ams_max_number];
uint8_t bus_now_ams_num = 0;
uint8_t bambubus_ams_map[4] = {0, 1, 2, 3};
uint32_t time_hw_tpms = 18000u;  // SysTick ticks per ms (144 MHz / 8)
static uint32_t g_ticks = 0x10000000u;
static uint32_t time_ticks32(void) { return g_ticks; }

// ---- adapted from main.cpp: the loaded-channel state (NVM writes only counted) ----
static uint8_t g_loaded_ch = 0xFF;
static boot_restore_t g_boot_restore = {0xFFu};
static bool kRestoreAtBoot = true;  // BMCU_BOOT_RESTORE_LOADED != 0 (a constant in the firmware)
static int nvm_jobs;

void ams_state_set_loaded(uint8_t filament_ch)
{
    if (filament_ch >= 4u) return;
    if (g_loaded_ch != 0xFFu) return;
    g_loaded_ch = filament_ch;
    nvm_jobs++;
}

void ams_state_set_unloaded(uint8_t filament_ch)
{
    if (g_loaded_ch == 0xFFu) return;
    if (filament_ch < 4u && g_loaded_ch != filament_ch) return;
    g_loaded_ch = 0xFFu;
    nvm_jobs++;

    // On any path: a restored in-use state the printer has not referenced yet goes with it.
    boot_restore_on_unloaded(&g_boot_restore, &ams[BAMBU_BUS_AMS_NUM], kRestoreAtBoot);
}

uint8_t ams_state_get_loaded(void)
{
    return g_loaded_ch;
}

bool ams_state_boot_restore_deferred(void)
{
    return !kRestoreAtBoot && boot_restore_pending(&g_boot_restore);
}

void ams_state_printer_command(uint8_t read_num, uint8_t statu_flags, uint8_t motion_flag)
{
    boot_restore_on_command(&g_boot_restore, &ams[BAMBU_BUS_AMS_NUM], kRestoreAtBoot, read_num, statu_flags,
                            motion_flag);
}

// ---- bambu_bus_ams.cpp at this commit: get_filament_left_char and set_motion, verbatim ----
uint8_t get_filament_left_char(const _ams *ams)
{
    uint8_t data = 0u;

    for (uint8_t i = 0; i < 4u; i++)
    {
        if (!ams->filament[i].online) continue;

        data |= (uint8_t)(1u << (i << 1));
        if (ams->filament[i].motion != _filament_motion::idle)
            data |= (uint8_t)(2u << (i << 1));
    }

    return data;
}

static uint32_t time_sendout_onuse_ticks[4] = {};
static uint8_t last_before_on_use_motion_flag = 0x00;
static uint8_t count_on_use = 0u;
bool set_motion(unsigned char read_num, unsigned char statu_flags, unsigned char fliment_motion_flag, uint8_t ams_num)
{
    const uint8_t fixed_ams_num = (uint8_t)BAMBU_BUS_AMS_NUM;
    if (ams_num != fixed_ams_num) return false;

    _ams *ams_ptr = &ams[bambubus_ams_map[fixed_ams_num]];

    // Hands the loaded channel restored at boot to the printer on its first reference (main.cpp).
    ams_state_printer_command(read_num, statu_flags, fliment_motion_flag);

    if (read_num < 4)
    {
        const uint8_t ch = (uint8_t)read_num;

        const bool is_send_out      = ((statu_flags == 0x03) && (fliment_motion_flag == 0x00));
        const bool is_before_on_use = ((statu_flags == 0x09) && ((fliment_motion_flag == 0x7F) || (fliment_motion_flag == 0xA5)));
        const bool is_stop_on_use   = ((statu_flags == 0x07) && (fliment_motion_flag == 0x00));
        const bool is_on_use        = ((statu_flags == 0x07) && (fliment_motion_flag == 0x7F));
        const bool is_before_pullb  = ((statu_flags == 0x09) && (fliment_motion_flag == 0x3F));

        uint32_t &t_sendout_onuse = time_sendout_onuse_ticks[ch];

        const uint8_t loaded = ams_state_get_loaded();
        const bool allow_any = (loaded == 0xFFu) || (loaded == ch);
        const bool allow_stop = (loaded == ch);

        const bool accept =
            (is_send_out) ||
            (is_before_on_use && allow_any) ||
            (is_on_use        && allow_any) ||
            (is_stop_on_use   && allow_stop) ||
            (is_before_pullb  && allow_stop);

        if (accept)
        {
            if (ams_ptr->now_filament_num != ch)
            {
                if (ams_ptr->now_filament_num < 4)
                {
                    const uint8_t prev = ams_ptr->now_filament_num;
                    ams_ptr->filament[prev].motion = _filament_motion::idle;
                    ams_ptr->filament_use_flag = 0x00;
                    ams_ptr->pressure = 0xF9C6;
                    time_sendout_onuse_ticks[prev] = 0u;
                }
                bus_now_ams_num = bambubus_ams_map[fixed_ams_num];
                ams_ptr->now_filament_num = ch;
            }
        }

        if (is_send_out)
        {
            t_sendout_onuse = 0u;
            count_on_use = 0u;

            const _filament_motion prev = ams_ptr->filament[ch].motion;

            if (prev != _filament_motion::send_out && ams_state_get_loaded() != 0xFFu)
                ams_state_set_unloaded(0xFFu);

            ams_ptr->filament[ch].motion = _filament_motion::send_out;
            ams_ptr->filament_use_flag = 0x02;
            ams_ptr->pressure = 0x4700;
        }
        else if (is_before_on_use)
        {
            t_sendout_onuse = 0u;
            count_on_use = 0u;

            if (!allow_any) return true;

            last_before_on_use_motion_flag = fliment_motion_flag;

            const _filament_motion prev = ams_ptr->filament[ch].motion;

            ams_ptr->filament[ch].motion = _filament_motion::before_on_use;
            ams_ptr->filament_use_flag = 0x04;

            if (fliment_motion_flag == 0x7F)
            {
                ams_ptr->pressure = 0x1E34;
            }
            else
            {
                ams_ptr->pressure = (prev == _filament_motion::send_out) ? 0x4700 : 0x2B00;
            }

            ams_state_set_loaded(ch);
        }
        else if (is_stop_on_use)
        {
            t_sendout_onuse = 0u;

            if (!allow_stop) return true;

            const _filament_motion prev = ams_ptr->filament[ch].motion;

            if (prev == _filament_motion::on_use ||
                prev == _filament_motion::before_on_use)
            {
                ams_ptr->filament[ch].motion = _filament_motion::stop_on_use;
            }

            ams_ptr->filament_use_flag = 0x04;
            ams_ptr->pressure = 0x2B00;

            if (prev == _filament_motion::before_on_use && last_before_on_use_motion_flag == 0x7F)
            {
                ams_ptr->pressure = 0x1E34;
            }
        }
        else if (is_on_use)
        {
            if (!allow_any) return true;

            const _filament_motion prev = ams_ptr->filament[ch].motion;

            if (prev == _filament_motion::send_out)
            {
                if (time_hw_tpms != 0u)
                {
                    const uint32_t now = time_ticks32();
                    if (t_sendout_onuse == 0u) t_sendout_onuse = now;

                    const uint32_t dt = (uint32_t)(now - t_sendout_onuse);
                    const uint32_t lim = 15000u * (uint32_t)time_hw_tpms;

                    if (dt < lim)
                    {
                        ams_ptr->filament_use_flag = 0x04;
                        ams_ptr->pressure = 0x2B00;
                        return true;
                    }

                    t_sendout_onuse = 0u;
                }
                else
                {
                    ams_ptr->filament_use_flag = 0x04;
                    ams_ptr->pressure = 0x2B00;
                    return true;
                }
            }

            t_sendout_onuse = 0u;

            ams_ptr->filament[ch].motion = _filament_motion::on_use;
            ams_ptr->filament_use_flag = 0x04;

            if (ams_ptr->pressure != 0xF06Fu) ams_ptr->pressure = 0x2B00;

            if (last_before_on_use_motion_flag == 0x7F && count_on_use < 5)
            {
                count_on_use++;
                ams_ptr->pressure = 0x1E34;
            }

            ams_state_set_loaded(ch);
        }
        else if (is_before_pullb)
        {
            t_sendout_onuse = 0u;

            if (!allow_stop) return true;

            const _filament_motion prev = ams_ptr->filament[ch].motion;

            if (prev == _filament_motion::on_use ||
                prev == _filament_motion::before_on_use ||
                prev == _filament_motion::stop_on_use)
            {
                ams_ptr->filament[ch].motion = _filament_motion::before_pull_back;
            }

            ams_ptr->filament_use_flag = 0x04;
            ams_ptr->pressure = 0x2B00;

            ams_state_set_unloaded(ch);
        }
        else if (statu_flags == 0x09)
        {
            t_sendout_onuse = 0u;
            ams_ptr->filament_use_flag = 0x04;
            ams_ptr->pressure = 0x2B00;
        }
    }
    else if (read_num == 0xFF)
    {
        if ((statu_flags == 0x03) && (fliment_motion_flag == 0x00))
        {
            if (ams_ptr->now_filament_num < 4)
            {
                const uint8_t ch = ams_ptr->now_filament_num;
                const _filament_motion m = ams_ptr->filament[ch].motion;

                time_sendout_onuse_ticks[ch] = 0u;

                if (m == _filament_motion::before_pull_back ||
                    m == _filament_motion::on_use ||
                    m == _filament_motion::before_on_use ||
                    m == _filament_motion::stop_on_use)
                {
                    ams_ptr->filament[ch].motion = _filament_motion::pull_back;
                    ams_ptr->filament_use_flag = 0x02;
                }

                ams_ptr->pressure = 0x4700;
                ams_state_set_unloaded(ch);
            }
        }
        else if (statu_flags == 0x01)
        {
            const uint8_t ch = ams_ptr->now_filament_num;
            if (ch < 4 && ams_ptr->filament_use_flag != 0x04)
                ams_state_set_unloaded(ch);
        }
        else
        {
            if (ams_ptr->now_filament_num < 4)
            {
                const uint8_t ch = ams_ptr->now_filament_num;
                const _filament_motion m = ams_ptr->filament[ch].motion;

                if (m == _filament_motion::on_use ||
                    m == _filament_motion::before_on_use ||
                    m == _filament_motion::stop_on_use)
                {
                    return true;
                }
            }

            // With BMCU_BOOT_RESTORE_LOADED=0 the restored channel is not in use in RAM yet, so the
            // check above cannot keep this reset from dropping it; keep it until the printer
            // references it, as the default image does through that check.
            if (ams_state_boot_restore_deferred())
                return true;

            for (uint8_t i = 0; i < 4; i++)
            {
                ams_ptr->filament[i].motion = _filament_motion::idle;
                time_sendout_onuse_ticks[i] = 0u;
            }

            ams_ptr->filament_use_flag = 0x00;
            ams_ptr->pressure = 0xF9C6;
            ams_ptr->now_filament_num = 0xFF;
            ams_state_set_unloaded(0xFFu);
        }
    }

    return true;
}
// ---- end of the bambu_bus_ams.cpp copy ----

// ---- Test harness ----
static const _filament_motion IDLE = _filament_motion::idle;
static const _filament_motion SEND_OUT = _filament_motion::send_out;
static const _filament_motion ON_USE = _filament_motion::on_use;
static const _filament_motion BEFORE_PULL_BACK = _filament_motion::before_pull_back;
static const _filament_motion PULL_BACK = _filament_motion::pull_back;
static const _filament_motion STOP_ON_USE = _filament_motion::stop_on_use;

static _ams &A = ams[BAMBU_BUS_AMS_NUM];
static bool filament[4];  // MC_ONLINE_key_stu[ch] != 0: filament at the channel's switches

// ---- Motion_control.cpp at this commit: kChCount, verbatim ----
static constexpr uint8_t  kChCount = 4;
// ---- end of the Motion_control.cpp copy ----
static uint8_t MC_ONLINE_key_stu[4];

// ---- adapted from main.cpp: power-on, ams_init() and the STA record's restore (Motion_control_init sets online) ----
// Power-on: RAM as after ams_init(), then main.cpp's restore. sta_ch: the STA record (0xFF: none
// loaded, or no valid record: g_loaded_ch then stays 0xFF and the restore does nothing either).
static void boot(bool restore_at_boot, uint8_t sta_ch)
{
    for (uint8_t i = 0; i < ams_max_number; i++) ams[i].init();
    A.online = true;  // Motion_control_init
    bus_now_ams_num = 0;
    memset(time_sendout_onuse_ticks, 0, sizeof(time_sendout_onuse_ticks));
    last_before_on_use_motion_flag = 0x00;
    count_on_use = 0u;

    kRestoreAtBoot = restore_at_boot;
    g_boot_restore.ch = 0xFFu;  // static initialiser
    nvm_jobs = 0;

    g_loaded_ch = sta_ch;
    boot_restore_init(&g_boot_restore, &A, sta_ch, kRestoreAtBoot);
}

// Motion_control_run up to the empty-switch check, once per main-loop pass (with or without link).
static void pass(void)
{
// ---- adapted from Motion_control.cpp: MC_PULL_ONLINE_read's key state and Motion_control_run's online flags ----
    for (uint8_t i = 0; i < 4u; i++)
    {
        MC_ONLINE_key_stu[i] = filament[i] ? 1u : 0u;
        A.filament[i].online = filament[i];
    }

// ---- Motion_control.cpp at this commit: Motion_control_run's empty-switch check, verbatim ----
    const uint8_t loaded_ch = ams_state_get_loaded();
    if ((loaded_ch < kChCount) && (MC_ONLINE_key_stu[loaded_ch] == 0u))
        ams_state_set_unloaded(loaded_ch);
// ---- end of the Motion_control.cpp copy ----
}

// One printer motion command (0x03 or 0x04 packet), then a main-loop pass.
static void cmd(uint8_t read_num, uint8_t statu, uint8_t motion)
{
    TEST_ASSERT_TRUE(set_motion(read_num, statu, motion, BAMBU_BUS_AMS_NUM));
    g_ticks += 18000u * 300u;  // the printer polls every few hundred ms
    pass();
}

static void idle_poll(void) { cmd(0xFF, 0x01, 0x00); }
static void unload(void) { cmd(0xFF, 0x03, 0x00); }  // pull back the current channel
static void reset_to_idle(void) { cmd(0xFF, 0x00, 0x00); }
static void send_out(uint8_t ch) { cmd(ch, 0x03, 0x00); }
static void before_on_use(uint8_t ch) { cmd(ch, 0x09, 0xA5); }
static void on_use(uint8_t ch) { cmd(ch, 0x07, 0x7F); }
static void stop_on_use(uint8_t ch) { cmd(ch, 0x07, 0x00); }
static void before_pull_back(uint8_t ch) { cmd(ch, 0x09, 0x3F); }

// ---- adapted from bambu_bus_ams.cpp: the channel, use and state fields of get_package_motion's reply ----
// What get_package_motion / get_package_stu_motion report for the current state.
typedef struct
{
    uint8_t channel;   // filament_channel
    uint8_t use_flag;  // filament_use_flag
    uint8_t stu_flag;  // filament_stu_flag: bit 2ch present, bit 2ch+1 moving
} reply_t;

static reply_t reply(void)
{
    reply_t r;
    r.channel = A.now_filament_num;
    r.use_flag = (A.now_filament_num == 0xFF) ? 0x00 : A.filament_use_flag;
    r.stu_flag = get_filament_left_char(&A);
    return r;
}

static void assert_reports_idle(void)
{
    const reply_t r = reply();
    TEST_ASSERT_EQUAL_HEX8(0xFF, r.channel);
    TEST_ASSERT_EQUAL_HEX8(0x00, r.use_flag);
    for (uint8_t i = 0; i < 4u; i++)
        TEST_ASSERT_EQUAL_HEX8(0u, r.stu_flag & (2u << (i << 1)));  // nothing moving
}

static void assert_reports_in_use(uint8_t ch)
{
    const reply_t r = reply();
    TEST_ASSERT_EQUAL_HEX8(ch, r.channel);
    TEST_ASSERT_EQUAL_HEX8(0x04, r.use_flag);
    TEST_ASSERT_EQUAL_HEX8(3u << (ch << 1), r.stu_flag & (3u << (ch << 1)));  // present and moving
}

static void assert_motion(uint8_t ch, _filament_motion m)
{
    TEST_ASSERT_EQUAL_UINT8((uint8_t)m, (uint8_t)A.filament[ch].motion);
}

// Everything set_motion reads or writes, to compare two runs.
typedef struct
{
    uint8_t  now, use, loaded, last_flag, count, bus_ams;
    uint8_t  motion[4];
    uint16_t pressure;
    uint32_t t_sendout[4];
} snap_t;

static snap_t snap(void)
{
    snap_t s;
    memset(&s, 0, sizeof(s));
    s.now = A.now_filament_num;
    s.use = A.filament_use_flag;
    s.loaded = g_loaded_ch;
    s.last_flag = last_before_on_use_motion_flag;
    s.count = count_on_use;
    s.bus_ams = bus_now_ams_num;
    for (uint8_t i = 0; i < 4u; i++)
    {
        s.motion[i] = (uint8_t)A.filament[i].motion;
        s.t_sendout[i] = time_sendout_onuse_ticks[i];
    }
    s.pressure = A.pressure;
    return s;
}

static const uint8_t S = 1u;  // the channel loaded at power-off
static const uint8_t X = 2u;  // another channel with filament

void setUp(void)
{
    g_ticks = 0x10000000u;
    for (uint8_t i = 0; i < 4u; i++) filament[i] = false;
    filament[S] = true;
    filament[X] = true;
}

void tearDown(void) {}

// ---- The header on its own ----

static void test_references_every_command_for_the_channel_and_the_unload_only(void)
{
    static const uint8_t status[] = {0x00, 0x01, 0x03, 0x07, 0x09, 0x0B};
    static const uint8_t flags[] = {0x00, 0x3F, 0x7F, 0xA5};

    for (uint8_t ch = 0; ch < 4u; ch++)
    {
        for (unsigned rn = 0; rn <= 0xFFu; rn++)
        {
            for (unsigned i = 0; i < sizeof(status); i++)
            {
                for (unsigned j = 0; j < sizeof(flags); j++)
                {
                    const bool unload_cmd = (rn == 0xFFu) && (status[i] == 0x03) && (flags[j] == 0x00);
                    const bool expect = (rn == ch) || unload_cmd;
                    TEST_ASSERT_EQUAL(expect, boot_restore_references(ch, (uint8_t)rn, status[i], flags[j]));
                }
            }
        }
    }
}

static void test_init_restores_on_use_at_boot_as_before(void)
{
    for (uint8_t ch = 0; ch < 4u; ch++)
    {
        boot(true, ch);
        TEST_ASSERT_EQUAL_HEX8(ch, g_boot_restore.ch);
        TEST_ASSERT_EQUAL_HEX8(ch, A.now_filament_num);
        TEST_ASSERT_EQUAL_HEX8(0x04, A.filament_use_flag);
        TEST_ASSERT_EQUAL_HEX16(0x2B00, A.pressure);
        for (uint8_t i = 0; i < 4u; i++) assert_motion(i, (i == ch) ? ON_USE : IDLE);
        TEST_ASSERT_EQUAL(0, nvm_jobs);
    }
}

static void test_init_deferred_keeps_the_channel_but_leaves_ram_idle(void)
{
    for (uint8_t ch = 0; ch < 4u; ch++)
    {
        boot(false, ch);
        TEST_ASSERT_EQUAL_HEX8(ch, g_boot_restore.ch);
        TEST_ASSERT_EQUAL_HEX8(ch, g_loaded_ch);
        TEST_ASSERT_EQUAL_HEX8(0xFF, A.now_filament_num);
        TEST_ASSERT_EQUAL_HEX8(0x00, A.filament_use_flag);
        TEST_ASSERT_EQUAL_HEX16(0xFFFF, A.pressure);
        for (uint8_t i = 0; i < 4u; i++) assert_motion(i, IDLE);
    }
}

static void test_init_without_a_loaded_channel_does_nothing(void)
{
    static const uint8_t none[] = {0x04, 0x7F, 0xFE, 0xFF};
    for (int mode = 0; mode < 2; mode++)
    {
        for (unsigned k = 0; k < sizeof(none); k++)
        {
            boot(mode != 0, none[k]);
            TEST_ASSERT_EQUAL_HEX8(0xFF, g_boot_restore.ch);
            TEST_ASSERT_EQUAL_HEX8(0xFF, A.now_filament_num);
            TEST_ASSERT_EQUAL_HEX8(0x00, A.filament_use_flag);
            for (uint8_t i = 0; i < 4u; i++) assert_motion(i, IDLE);

            // No command and no unload touches the RAM state on the restore's behalf.
            boot_restore_on_command(&g_boot_restore, &A, mode != 0, S, 0x07, 0x7F);
            boot_restore_on_unloaded(&g_boot_restore, &A, mode != 0);
            TEST_ASSERT_EQUAL_HEX8(0xFF, A.now_filament_num);
            assert_motion(S, IDLE);
        }
    }
}

// ---- BMCU_BOOT_RESTORE_LOADED=1 (default) ----

static void test_default_idle_polls_still_see_the_channel_in_use(void)
{
    boot(true, S);
    pass();
    for (int i = 0; i < 50; i++)
    {
        idle_poll();
        assert_reports_in_use(S);
        assert_motion(S, ON_USE);
        TEST_ASSERT_EQUAL_HEX8(S, g_loaded_ch);
    }
    TEST_ASSERT_EQUAL(0, nvm_jobs);
}

static void test_default_channel_empty_at_boot_reports_idle(void)
{
    filament[S] = false;  // pulled out while the printer was off
    boot(true, S);
    pass();  // first main-loop pass, before the printer's first heartbeat

    TEST_ASSERT_EQUAL_HEX8(0xFF, g_loaded_ch);
    TEST_ASSERT_EQUAL_HEX8(0xFF, A.now_filament_num);
    TEST_ASSERT_EQUAL_HEX8(0x00, A.filament_use_flag);
    assert_motion(S, IDLE);
    assert_reports_idle();

    for (int i = 0; i < 10; i++)
    {
        idle_poll();
        assert_reports_idle();
    }
    TEST_ASSERT_EQUAL(1, nvm_jobs);  // the unloaded record, as before
}

static void test_default_channel_pulled_out_after_idle_polls_reports_idle(void)
{
    boot(true, S);
    for (int i = 0; i < 5; i++) idle_poll();
    assert_reports_in_use(S);

    filament[S] = false;  // the #125 workaround: pull the filament out of the BMCU
    pass();
    TEST_ASSERT_EQUAL_HEX8(0xFF, g_loaded_ch);
    TEST_ASSERT_EQUAL_HEX8(0x00, A.filament_use_flag);
    assert_motion(S, IDLE);
    assert_reports_idle();

    // The printer can then use another channel, which the stale channel used to block.
    before_on_use(X);
    on_use(X);
    assert_reports_in_use(X);
    TEST_ASSERT_EQUAL_HEX8(X, g_loaded_ch);
}

static void test_default_other_channel_commands_leave_the_restore_pending(void)
{
    boot(true, S);
    on_use(X);  // dropped: S is loaded
    stop_on_use(X);
    before_pull_back(X);
    cmd(X, 0x09, 0x00);
    reset_to_idle();  // ignored while S is on_use
    assert_reports_in_use(S);
    TEST_ASSERT_EQUAL_HEX8(S, g_boot_restore.ch);

    filament[S] = false;
    pass();
    assert_reports_idle();
    on_use(X);
    assert_reports_in_use(X);
}

static void test_default_runout_after_resume_keeps_the_state_for_the_pull_back(void)
{
    boot(true, S);
    idle_poll();
    on_use(S);  // print resumed after the power cut
    TEST_ASSERT_EQUAL_HEX8(0xFF, g_boot_restore.ch);
    assert_reports_in_use(S);

    filament[S] = false;  // runout: the end passes the switches
    pass();
    TEST_ASSERT_EQUAL_HEX8(0xFF, g_loaded_ch);
    TEST_ASSERT_EQUAL_HEX8(S, A.now_filament_num);  // unchanged, as before
    TEST_ASSERT_EQUAL_HEX8(0x04, A.filament_use_flag);
    assert_motion(S, ON_USE);

    for (int i = 0; i < 5; i++)
    {
        on_use(S);  // the extruder still draws the rest
        TEST_ASSERT_EQUAL_HEX8(S, A.now_filament_num);
    }

    unload();
    assert_motion(S, PULL_BACK);  // the rest is still pulled back
    TEST_ASSERT_EQUAL_HEX8(0x02, A.filament_use_flag);
}

static void test_default_first_commands_after_boot_act_as_before(void)
{
    // send_out for the restored channel: loads it (the hook runs before set_motion clears
    // g_loaded_ch inside the send_out branch).
    boot(true, S);
    send_out(S);
    TEST_ASSERT_EQUAL_HEX8(S, A.now_filament_num);
    assert_motion(S, SEND_OUT);
    TEST_ASSERT_EQUAL_HEX8(0x02, A.filament_use_flag);
    TEST_ASSERT_EQUAL_HEX8(0xFF, g_loaded_ch);

    // send_out for another channel: loads that one, the restored channel goes idle.
    boot(true, S);
    send_out(X);
    TEST_ASSERT_EQUAL_HEX8(X, A.now_filament_num);
    assert_motion(X, SEND_OUT);
    assert_motion(S, IDLE);
    TEST_ASSERT_EQUAL_HEX8(0x02, A.filament_use_flag);
    TEST_ASSERT_EQUAL_HEX8(0xFF, g_boot_restore.ch);
    filament[S] = false;
    pass();
    TEST_ASSERT_EQUAL_HEX8(X, A.now_filament_num);
    assert_motion(X, SEND_OUT);

    // Unload after the power cut: before_pull_back, then pull back.
    setUp();
    boot(true, S);
    idle_poll();
    before_pull_back(S);
    assert_motion(S, BEFORE_PULL_BACK);
    unload();
    assert_motion(S, PULL_BACK);

    // Unload with only 0xFF/0x03.
    setUp();
    boot(true, S);
    unload();
    assert_motion(S, PULL_BACK);
    TEST_ASSERT_EQUAL_HEX8(0xFF, g_loaded_ch);
}

static void test_default_restored_state_is_not_undone_once_referenced(void)
{
    static const uint8_t refs[][3] = {
        {S, 0x07, 0x7F}, {S, 0x07, 0x00}, {S, 0x09, 0xA5}, {S, 0x09, 0x7F}, {S, 0x09, 0x00},
    };
    for (unsigned k = 0; k < sizeof(refs) / sizeof(refs[0]); k++)
    {
        setUp();
        boot(true, S);
        cmd(refs[k][0], refs[k][1], refs[k][2]);
        const snap_t before = snap();

        filament[S] = false;
        pass();
        snap_t after = snap();
        TEST_ASSERT_EQUAL_HEX8(0xFF, after.loaded);
        after.loaded = before.loaded;
        TEST_ASSERT_EQUAL_MEMORY(&before, &after, sizeof(before));  // only g_loaded_ch changed
    }
}

// In the default image the handover only ends the restore: the printer's first reference finds the
// state as the BMCU left it, for example with the jam (0xF06F) its on_use control latched on the
// restored channel after the first heartbeat.
static void test_default_handover_changes_nothing(void)
{
    static const uint8_t refs[][3] = {
        {S, 0x07, 0x7F}, {S, 0x07, 0x00}, {S, 0x09, 0xA5}, {S, 0x09, 0x3F}, {S, 0x03, 0x00}, {0xFF, 0x03, 0x00},
    };
    for (unsigned k = 0; k < sizeof(refs) / sizeof(refs[0]); k++)
    {
        boot(true, S);
        A.pressure = 0xF06F;           // Motion_control_run while the jam latch is set
        A.filament[S].meters = 12.5f;  // AS5600 odometer
        uint8_t before[sizeof(_ams)];
        memcpy(before, &A, sizeof(A));

        boot_restore_on_command(&g_boot_restore, &A, true, refs[k][0], refs[k][1], refs[k][2]);
        TEST_ASSERT_EQUAL_HEX8(0xFF, g_boot_restore.ch);
        TEST_ASSERT_EQUAL_MEMORY(before, &A, sizeof(A));
    }

    // Through set_motion: the resume still reports the jam.
    boot(true, S);
    A.pressure = 0xF06F;
    on_use(S);
    TEST_ASSERT_EQUAL_HEX16(0xF06F, A.pressure);
}

static void test_default_ahub_host_keeps_the_old_behaviour(void)
{
    boot(true, S);
    boot_restore_drop(&g_boot_restore);  // first AHUB heartbeat

    // all_filament_stu: the AHUB host writes the channel states itself.
    A.now_filament_num = S;
    A.filament[S].motion = ON_USE;

    filament[S] = false;
    pass();
    TEST_ASSERT_EQUAL_HEX8(0xFF, g_loaded_ch);
    TEST_ASSERT_EQUAL_HEX8(S, A.now_filament_num);
    TEST_ASSERT_EQUAL_HEX8(0x04, A.filament_use_flag);
    assert_motion(S, ON_USE);
}

// ---- BMCU_BOOT_RESTORE_LOADED=0 (A/B image) ----

static void test_ab_boot_reports_idle_and_keeps_the_loaded_channel(void)
{
    boot(false, S);
    pass();
    for (int i = 0; i < 50; i++)
    {
        idle_poll();
        assert_reports_idle();
        TEST_ASSERT_EQUAL_HEX8(S, g_loaded_ch);
        TEST_ASSERT_EQUAL_HEX8(S, g_boot_restore.ch);
    }
    TEST_ASSERT_EQUAL(0, nvm_jobs);

    // The gating still sees S loaded: commands for another channel are dropped as before.
    on_use(X);
    stop_on_use(X);
    before_pull_back(X);
    assert_reports_idle();
    TEST_ASSERT_EQUAL_HEX8(S, g_loaded_ch);
    TEST_ASSERT_EQUAL_HEX8(S, g_boot_restore.ch);
}

// From the printer's first reference to S on, the A/B image must hold the same state as the default
// one after every command. Each first reference is followed by the rest of a resume, a print end,
// and an unload.
static void test_ab_matches_the_default_from_the_first_reference(void)
{
    static const uint8_t first[][3] = {
        {S, 0x07, 0x7F},     // on_use (resume)
        {S, 0x07, 0x00},     // stop_on_use
        {S, 0x09, 0xA5},     // before_on_use
        {S, 0x09, 0x7F},     // before_on_use 0x7F
        {S, 0x09, 0x3F},     // before_pull_back
        {S, 0x03, 0x00},     // send_out
        {S, 0x09, 0x00},     // other 0x09
        {0xFF, 0x03, 0x00},  // pull back the current channel
    };
    static const uint8_t rest[][3] = {
        {S, 0x09, 0xA5}, {S, 0x07, 0x7F}, {S, 0x07, 0x7F}, {0xFF, 0x01, 0x00}, {S, 0x07, 0x00},
        {S, 0x07, 0x7F}, {S, 0x09, 0x3F}, {0xFF, 0x03, 0x00}, {0xFF, 0x01, 0x00}, {0xFF, 0x00, 0x00},
    };

    for (unsigned k = 0; k < sizeof(first) / sizeof(first[0]); k++)
    {
        for (int polls = 0; polls < 3; polls += 2)
        {
            snap_t ref[1 + sizeof(rest) / sizeof(rest[0])];
            for (int mode = 1; mode >= 0; mode--)
            {
                setUp();
                boot(mode != 0, S);
                pass();
                for (int p = 0; p < polls; p++) idle_poll();

                cmd(first[k][0], first[k][1], first[k][2]);
                TEST_ASSERT_EQUAL_HEX8(0xFF, g_boot_restore.ch);
                if (first[k][1] == 0x07 && first[k][2] == 0x00) assert_motion(S, STOP_ON_USE);
                snap_t s = snap();
                if (mode) ref[0] = s;
                else TEST_ASSERT_EQUAL_MEMORY(&ref[0], &s, sizeof(s));

                for (unsigned n = 0; n < sizeof(rest) / sizeof(rest[0]); n++)
                {
                    cmd(rest[n][0], rest[n][1], rest[n][2]);
                    s = snap();
                    if (mode) ref[1 + n] = s;
                    else TEST_ASSERT_EQUAL_MEMORY(&ref[1 + n], &s, sizeof(s));
                }
            }
        }
    }
}

static void test_ab_unload_after_the_power_cut_still_pulls_back(void)
{
    boot(false, S);
    idle_poll();
    before_pull_back(S);
    assert_motion(S, BEFORE_PULL_BACK);
    TEST_ASSERT_EQUAL_HEX8(S, A.now_filament_num);
    unload();
    assert_motion(S, PULL_BACK);
    TEST_ASSERT_EQUAL_HEX8(0x02, A.filament_use_flag);

    boot(false, S);
    idle_poll();
    unload();
    assert_motion(S, PULL_BACK);
    TEST_ASSERT_EQUAL_HEX8(0xFF, g_loaded_ch);
}

static void test_ab_resume_reports_the_channel_in_use(void)
{
    boot(false, S);
    idle_poll();
    on_use(S);
    assert_reports_in_use(S);
    assert_motion(S, ON_USE);
    TEST_ASSERT_EQUAL_HEX8(S, g_loaded_ch);
}

static void test_ab_restore_dropped_when_the_loaded_channel_is_cleared(void)
{
    // The switches read empty before the printer referenced S.
    boot(false, S);
    idle_poll();
    filament[S] = false;
    pass();
    TEST_ASSERT_EQUAL_HEX8(0xFF, g_loaded_ch);
    TEST_ASSERT_EQUAL_HEX8(0xFF, g_boot_restore.ch);
    unload();  // nothing to pull back
    assert_motion(S, IDLE);
    assert_reports_idle();

    // A load of another channel.
    setUp();
    boot(false, S);
    send_out(X);
    TEST_ASSERT_EQUAL_HEX8(0xFF, g_boot_restore.ch);
    TEST_ASSERT_EQUAL_HEX8(X, A.now_filament_num);
    assert_motion(X, SEND_OUT);
    assert_motion(S, IDLE);

    // The printer's reset to idle before it has referenced S: the default image ignores it because
    // S is on_use in RAM; the A/B image keeps g_loaded_ch and the pending restore as well (RAM is
    // still idle there), so the STA record survives until the printer takes the channel over.
    setUp();
    boot(false, S);
    reset_to_idle();
    TEST_ASSERT_EQUAL_HEX8(S, g_loaded_ch);
    TEST_ASSERT_EQUAL_HEX8(S, g_boot_restore.ch);
    TEST_ASSERT_EQUAL_INT(0, nvm_jobs);
    assert_reports_idle();
    boot(true, S);
    reset_to_idle();
    TEST_ASSERT_EQUAL_HEX8(S, g_loaded_ch);
    assert_reports_in_use(S);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_references_every_command_for_the_channel_and_the_unload_only);
    RUN_TEST(test_init_restores_on_use_at_boot_as_before);
    RUN_TEST(test_init_deferred_keeps_the_channel_but_leaves_ram_idle);
    RUN_TEST(test_init_without_a_loaded_channel_does_nothing);
    RUN_TEST(test_default_idle_polls_still_see_the_channel_in_use);
    RUN_TEST(test_default_channel_empty_at_boot_reports_idle);
    RUN_TEST(test_default_channel_pulled_out_after_idle_polls_reports_idle);
    RUN_TEST(test_default_other_channel_commands_leave_the_restore_pending);
    RUN_TEST(test_default_runout_after_resume_keeps_the_state_for_the_pull_back);
    RUN_TEST(test_default_first_commands_after_boot_act_as_before);
    RUN_TEST(test_default_restored_state_is_not_undone_once_referenced);
    RUN_TEST(test_default_handover_changes_nothing);
    RUN_TEST(test_default_ahub_host_keeps_the_old_behaviour);
    RUN_TEST(test_ab_boot_reports_idle_and_keeps_the_loaded_channel);
    RUN_TEST(test_ab_matches_the_default_from_the_first_reference);
    RUN_TEST(test_ab_unload_after_the_power_cut_still_pulls_back);
    RUN_TEST(test_ab_resume_reports_the_channel_in_use);
    RUN_TEST(test_ab_restore_dropped_when_the_loaded_channel_is_cleared);
    return UNITY_END();
}
