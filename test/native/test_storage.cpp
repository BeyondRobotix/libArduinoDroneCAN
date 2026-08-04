/*
    Host tests for the log-structured parameter storage in storage.cpp.

    Run via test/native/run_tests.ps1, which builds this twice: once with the
    H7 geometry (128 KB sectors, 32-byte flash words) and once with the L4
    geometry (2 KB pages, 8-byte double words). Everything below is written
    against the geometry constants so both passes share one suite.

    These exercise the parts that are painful to reach on hardware: log
    replay, compaction when a page fills, and recovery from power loss at
    each step of a sector switch. The flash model aborts on any write that
    violates the write-once-per-word rule, so an ECC-illegal access on H7
    fails loudly here rather than faulting on target.
*/

#include "../../src/storage.h"
#include "flash_model.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

using namespace flashmodel;

// Provided by storage.cpp under -DSTORAGE_HOST_TEST: clears the cached log
// position, which on target only happens across a reset.
extern void dronecan_storage_test_reset_cache();

static int g_failures;
static int g_checks;
static const char *g_current_test;

#define CHECK(cond)                                                            \
    do {                                                                       \
        g_checks++;                                                            \
        if (!(cond)) {                                                         \
            g_failures++;                                                      \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                      \
    } while (0)

#define CHECK_FEQ(a, b)                                                        \
    do {                                                                       \
        g_checks++;                                                            \
        if (fabsf((a) - (b)) > 1e-6f) {                                        \
            g_failures++;                                                      \
            printf("  FAIL %s:%d: %s (%f) != %s (%f)\n", __FILE__, __LINE__,   \
                   #a, (double)(a), #b, (double)(b));                          \
        }                                                                      \
    } while (0)

static void begin_test(const char *name)
{
    g_current_test = name;
    printf("== %s\n", name);
    reset();
    dronecan_storage_test_reset_cache();
}

// Simulate a power cycle: flash keeps its contents, cached state is lost.
static void reboot()
{
    restore_power();
    dronecan_storage_test_reset_cache();
}

// A representative parameter set: name list -> signature.
static const char *kNames[] = {"NODEID", "BITRATE", "CANFD", "RATE_HZ", "GAIN"};
static constexpr size_t kCount = sizeof(kNames) / sizeof(kNames[0]);

static uint32_t sig_for(const char *const *names, size_t count)
{
    uint32_t sig = DroneCAN_Storage::signature_begin();
    for (size_t i = 0; i < count; i++) {
        sig = DroneCAN_Storage::signature_feed(sig, names[i]);
    }
    return sig;
}

// Geometry, mirroring storage.cpp. A record is one flash word; a state slot
// is >= 12 bytes rounded up to a whole number of words (16 on L4, 32 on H7).
static constexpr size_t kRecord = WRITE_UNIT;
static constexpr size_t kSlot   = (WRITE_UNIT >= 16) ? WRITE_UNIT : 16;
static constexpr size_t kHeader = 3 * kSlot;
// Flash-word writes consumed by one state-slot write (2 on L4, 1 on H7).
static constexpr long kSlotWords = (long)(kSlot / WRITE_UNIT);
// Records that fit in one page after the header.
static constexpr int kSlots = (int)((PAGE_SIZE - kHeader) / kRecord);

// The pair is (base, base+1); use the last two pages of the model.
static constexpr uint32_t kPage = NUM_PAGES - 2;

// ---------------------------------------------------------------------------

static void test_first_boot_returns_defaults()
{
    begin_test("first boot: blank flash keeps code defaults");
    const uint32_t sig = sig_for(kNames, kCount);

    float values[kCount] = {1, 2, 3, 4, 5};
    // Blank flash -> load() must report "nothing stored" and not touch values.
    CHECK(DroneCAN_Storage::load(values, kCount, kPage, 0, sig) == false);
    CHECK_FEQ(values[0], 1.0f);
    CHECK_FEQ(values[4], 5.0f);
    CHECK(DroneCAN_Storage::healthy());
}

static void test_save_all_then_load()
{
    begin_test("save_all then load round-trips");
    const uint32_t sig = sig_for(kNames, kCount);

    float written[kCount] = {100.0f, 1000000.0f, 1.0f, 50.0f, 0.25f};
    CHECK(DroneCAN_Storage::save_all(written, kCount, kPage, 0, sig));

    float read[kCount] = {0, 0, 0, 0, 0};
    CHECK(DroneCAN_Storage::load(read, kCount, kPage, 0, sig));
    for (size_t i = 0; i < kCount; i++) {
        CHECK_FEQ(read[i], written[i]);
    }
}

static void test_single_save_last_write_wins()
{
    begin_test("repeated single saves: last write wins");
    const uint32_t sig = sig_for(kNames, kCount);

    float initial[kCount] = {100.0f, 1000000.0f, 1.0f, 50.0f, 0.25f};
    CHECK(DroneCAN_Storage::save_all(initial, kCount, kPage, 0, sig));

    for (int i = 0; i < 20; i++) {
        CHECK(DroneCAN_Storage::save(0, 42.0f + i, kCount, kPage, 0, sig));
    }

    float read[kCount] = {};
    CHECK(DroneCAN_Storage::load(read, kCount, kPage, 0, sig));
    CHECK_FEQ(read[0], 42.0f + 19);   // last value for index 0
    CHECK_FEQ(read[1], 1000000.0f);   // untouched indices survive
    CHECK_FEQ(read[4], 0.25f);
}

static void test_write_path_never_erases()
{
    begin_test("steady-state writes never erase");
    const uint32_t sig = sig_for(kNames, kCount);

    float values[kCount] = {1, 2, 3, 4, 5};
    CHECK(DroneCAN_Storage::save_all(values, kCount, kPage, 0, sig));
    CHECK(DroneCAN_Storage::load(values, kCount, kPage, 0, sig));

    // From here on, nothing should erase: this is the whole point of the
    // log-structured design on H7.
    reset_counters();
    for (int i = 0; i < 200; i++) {
        CHECK(DroneCAN_Storage::save(i % kCount, (float)i, kCount, kPage, 0, sig));
    }
    CHECK(erase_count() == 0);
    // One record per save. The layout record was already emitted by the
    // save_all above, so the steady state costs exactly one word each.
    CHECK(word_writes() == 200);
}

static void test_signature_change_rejects_stored_values()
{
    begin_test("changed parameter layout invalidates stored values");
    const uint32_t sig_old = sig_for(kNames, kCount);

    float written[kCount] = {11, 22, 33, 44, 55};
    CHECK(DroneCAN_Storage::save_all(written, kCount, kPage, 0, sig_old));

    // Firmware update reorders the parameter list, keeping the same count.
    const char *reordered[] = {"BITRATE", "NODEID", "CANFD", "RATE_HZ", "GAIN"};
    const uint32_t sig_new = sig_for(reordered, kCount);
    CHECK(sig_old != sig_new);

    float defaults[kCount] = {1, 2, 3, 4, 5};
    // Must refuse the stale log rather than assigning 11 to BITRATE.
    CHECK(DroneCAN_Storage::load(defaults, kCount, kPage, 0, sig_new) == false);
    CHECK_FEQ(defaults[0], 1.0f);
    CHECK_FEQ(defaults[1], 2.0f);
}

static void test_compaction_when_sector_fills()
{
    begin_test("log fills -> compaction into spare sector");
    const uint32_t sig = sig_for(kNames, kCount);

    float values[kCount] = {1, 2, 3, 4, 5};
    CHECK(DroneCAN_Storage::save_all(values, kCount, kPage, 0, sig));

    const int slots = kSlots;

    reset_counters();
    for (int i = 0; i < slots + 10; i++) {
        CHECK(DroneCAN_Storage::save(2, (float)i, kCount, kPage, 0, sig));
    }

    // Exactly one compaction should have happened, and it must not have
    // erased anything inline (the spare was pre-erased at load()).
    CHECK(erase_count() == 0);

    float read[kCount] = {};
    CHECK(DroneCAN_Storage::load(read, kCount, kPage, 0, sig));
    CHECK_FEQ(read[2], (float)(slots + 9));
    CHECK_FEQ(read[0], 1.0f);   // values carried across the compaction
    CHECK_FEQ(read[4], 5.0f);
}

static void test_power_loss_during_sector_switch()
{
    begin_test("power loss mid-compaction recovers on next boot");
    const uint32_t sig = sig_for(kNames, kCount);

    float values[kCount] = {7, 8, 9, 10, 11};
    CHECK(DroneCAN_Storage::save_all(values, kCount, kPage, 0, sig));
    CHECK(DroneCAN_Storage::load(values, kCount, kPage, 0, sig));

    // save_all above wrote kCount value records plus one layout record, so
    // this leaves exactly one free slot -- which the marker save below takes,
    // making the save after it the one that must compact.
    const int slots = kSlots;
    for (int i = 0; i < slots - (int)kCount - 2; i++) {
        CHECK(DroneCAN_Storage::save(2, (float)i, kCount, kPage, 0, sig));
    }
    CHECK(DroneCAN_Storage::save(2, 1234.0f, kCount, kPage, 0, sig));

    // The next save must compact. Cut power partway through the copy.
    reset_counters();
    arm_power_loss(3);
    DroneCAN_Storage::save(2, 4321.0f, kCount, kPage, 0, sig);
    CHECK(!DroneCAN_Storage::healthy());   // failure was reported, not swallowed

    // Reboot.
    reboot();
    float read[kCount] = {0, 0, 0, 0, 0};
    const bool loaded = DroneCAN_Storage::load(read, kCount, kPage, 0, sig);

    // The interrupted switch must not lose the previously committed set.
    CHECK(loaded);
    CHECK_FEQ(read[0], 7.0f);
    CHECK_FEQ(read[2], 1234.0f);   // last value committed before the cut
    CHECK_FEQ(read[4], 11.0f);
    CHECK(DroneCAN_Storage::healthy());

    // And storage must be usable again afterwards.
    CHECK(DroneCAN_Storage::save(1, 99.0f, kCount, kPage, 0, sig));
    float read2[kCount] = {};
    CHECK(DroneCAN_Storage::load(read2, kCount, kPage, 0, sig));
    CHECK_FEQ(read2[1], 99.0f);
    CHECK_FEQ(read2[2], 1234.0f);
}

static void test_power_loss_during_first_save()
{
    begin_test("power loss during first save leaves defaults intact");
    const uint32_t sig = sig_for(kNames, kCount);

    // Measure the word writes that precede the first value record -- claiming
    // the page IN_USE, then the instance's layout record. Measured rather than
    // hard-coded: the header slot spans two flash words on L4 but the driver
    // skips the all-0xFF half, so a computed figure would be wrong there.
    reset_counters();
    CHECK(DroneCAN_Storage::save(0, 1.0f, kCount, kPage, 0, sig));
    const long preamble = (long)word_writes() - 1;   // minus the value record

    // Start clean, then cut power so values 0 and 1 land but value 2 does not.
    reset();
    dronecan_storage_test_reset_cache();

    float values[kCount] = {5, 6, 7, 8, 9};
    arm_power_loss(preamble + 2);
    DroneCAN_Storage::save_all(values, kCount, kPage, 0, sig);

    reboot();
    float read[kCount] = {1, 2, 3, 4, 5};
    // A partial log is still a valid log -- whatever committed is readable,
    // and nothing may be corrupt. Indices that never landed keep defaults.
    DroneCAN_Storage::load(read, kCount, kPage, 0, sig);
    CHECK_FEQ(read[0], 5.0f);   // record 0 committed
    CHECK_FEQ(read[1], 6.0f);   // record 1 committed
    CHECK_FEQ(read[2], 3.0f);   // record 2 never landed -> default kept
    CHECK_FEQ(read[4], 5.0f);

    // Storage remains writable.
    CHECK(DroneCAN_Storage::save_all(values, kCount, kPage, 0, sig));
    float read2[kCount] = {};
    CHECK(DroneCAN_Storage::load(read2, kCount, kPage, 0, sig));
    CHECK_FEQ(read2[2], 7.0f);
}

static void test_reboot_erases_stale_sector()
{
    begin_test("boot erases the sector left FULL by a compaction");
    const uint32_t sig = sig_for(kNames, kCount);

    float values[kCount] = {1, 2, 3, 4, 5};
    CHECK(DroneCAN_Storage::save_all(values, kCount, kPage, 0, sig));
    CHECK(DroneCAN_Storage::load(values, kCount, kPage, 0, sig));

    const int slots = kSlots;
    for (int i = 0; i < slots + 5; i++) {
        CHECK(DroneCAN_Storage::save(3, (float)i, kCount, kPage, 0, sig));
    }

    // A compaction has run, so one sector is FULL and not yet erased. The
    // erase is deferred to the next *boot*, so simulate one -- within a power
    // cycle load() reuses the resolved log position and does no recovery.
    reboot();
    reset_counters();
    float read[kCount] = {};
    CHECK(DroneCAN_Storage::load(read, kCount, kPage, 0, sig));
    CHECK(erase_count() == 1);   // the deferred erase is paid here, at boot

    // Second boot has nothing left to clean up.
    reboot();
    reset_counters();
    CHECK(DroneCAN_Storage::load(read, kCount, kPage, 0, sig));
    CHECK(erase_count() == 0);
}

static void test_oversized_parameter_set_rejected()
{
    begin_test("parameter count beyond MAX_PARAMS is refused, not overflowed");
    const uint32_t sig = sig_for(kNames, kCount);
    const size_t too_many = DroneCAN_Storage::MAX_PARAMS + 1;

    static float big[DroneCAN_Storage::MAX_PARAMS + 8] = {};
    CHECK(DroneCAN_Storage::save(0, 1.0f, too_many, kPage, 0, sig) == false);
    (void)big;
}


// ---------------------------------------------------------------------------
// Two instances sharing one page pair. A dual-CAN node runs two DroneCAN
// objects whose parameters live in the same log, told apart by the instance
// tag -- so PORT2 costs no extra flash. These cover the ways that sharing
// could go wrong.
// ---------------------------------------------------------------------------

// A deliberately different parameter list for instance 1: different names AND
// a different count, so a bug that ignores the instance tag shows up as
// crossed values rather than silently matching.
static const char *kNames2[] = {"NODEID", "MODE", "SCALE"};
static constexpr size_t kCount2 = sizeof(kNames2) / sizeof(kNames2[0]);

static void test_two_instances_are_independent()
{
    begin_test("two instances share a page pair without colliding");
    const uint32_t sig0 = sig_for(kNames, kCount);
    const uint32_t sig1 = sig_for(kNames2, kCount2);

    float v0[kCount]  = {10, 20, 30, 40, 50};
    float v1[kCount2] = {70, 80, 90};
    CHECK(DroneCAN_Storage::save_all(v0, kCount, kPage, 0, sig0));
    CHECK(DroneCAN_Storage::save_all(v1, kCount2, kPage, 1, sig1));

    // Interleave further writes, as two live nodes would.
    CHECK(DroneCAN_Storage::save(1, 21.0f, kCount, kPage, 0, sig0));
    CHECK(DroneCAN_Storage::save(1, 81.0f, kCount2, kPage, 1, sig1));
    CHECK(DroneCAN_Storage::save(1, 22.0f, kCount, kPage, 0, sig0));

    reboot();

    float r0[kCount]  = {};
    float r1[kCount2] = {};
    CHECK(DroneCAN_Storage::load(r0, kCount, kPage, 0, sig0));
    CHECK(DroneCAN_Storage::load(r1, kCount2, kPage, 1, sig1));

    CHECK_FEQ(r0[0], 10.0f);
    CHECK_FEQ(r0[1], 22.0f);   // last write for instance 0 index 1
    CHECK_FEQ(r0[4], 50.0f);
    CHECK_FEQ(r1[0], 70.0f);
    CHECK_FEQ(r1[1], 81.0f);   // last write for instance 1 index 1
    CHECK_FEQ(r1[2], 90.0f);
}

static void test_instance_layout_change_spares_the_other()
{
    begin_test("changing one instance layout leaves the other intact");
    const uint32_t sig0 = sig_for(kNames, kCount);
    const uint32_t sig1 = sig_for(kNames2, kCount2);

    float v0[kCount]  = {10, 20, 30, 40, 50};
    float v1[kCount2] = {70, 80, 90};
    CHECK(DroneCAN_Storage::save_all(v0, kCount, kPage, 0, sig0));
    CHECK(DroneCAN_Storage::save_all(v1, kCount2, kPage, 1, sig1));

    // Firmware update reorders instance 0's parameters only.
    const char *reordered[] = {"BITRATE", "NODEID", "CANFD", "RATE_HZ", "GAIN"};
    const uint32_t sig0_new = sig_for(reordered, kCount);

    reboot();

    float d0[kCount] = {1, 2, 3, 4, 5};
    CHECK(DroneCAN_Storage::load(d0, kCount, kPage, 0, sig0_new) == false);
    CHECK_FEQ(d0[0], 1.0f);   // instance 0 falls back to code defaults

    float r1[kCount2] = {};
    CHECK(DroneCAN_Storage::load(r1, kCount2, kPage, 1, sig1));
    CHECK_FEQ(r1[0], 70.0f);  // instance 1 untouched by the other's change
    CHECK_FEQ(r1[2], 90.0f);
}

static void test_compaction_preserves_both_instances()
{
    begin_test("compaction driven by one instance keeps the other values");
    const uint32_t sig0 = sig_for(kNames, kCount);
    const uint32_t sig1 = sig_for(kNames2, kCount2);

    float v0[kCount]  = {10, 20, 30, 40, 50};
    float v1[kCount2] = {70, 80, 90};
    CHECK(DroneCAN_Storage::save_all(v0, kCount, kPage, 0, sig0));
    CHECK(DroneCAN_Storage::save_all(v1, kCount2, kPage, 1, sig1));
    CHECK(DroneCAN_Storage::load(v0, kCount, kPage, 0, sig0));

    // Fill the log using ONLY instance 0, so compaction is triggered by an
    // instance that knows nothing about instance 1's values. If compaction
    // used the caller's array instead of a full replay, instance 1 would be
    // silently lost here.
    reset_counters();
    for (int i = 0; i < kSlots + 10; i++) {
        CHECK(DroneCAN_Storage::save(2, (float)i, kCount, kPage, 0, sig0));
    }
    CHECK(erase_count() == 0);   // spare was pre-erased; no inline erase

    reboot();

    float r0[kCount]  = {};
    float r1[kCount2] = {};
    CHECK(DroneCAN_Storage::load(r0, kCount, kPage, 0, sig0));
    CHECK(DroneCAN_Storage::load(r1, kCount2, kPage, 1, sig1));

    CHECK_FEQ(r0[2], (float)(kSlots + 9));
    CHECK_FEQ(r0[0], 10.0f);
    CHECK_FEQ(r1[0], 70.0f);   // survived a compaction it never took part in
    CHECK_FEQ(r1[1], 80.0f);
    CHECK_FEQ(r1[2], 90.0f);
}

static void test_power_loss_mid_switch_keeps_both_instances()
{
    begin_test("power loss mid-compaction keeps both instances");
    const uint32_t sig0 = sig_for(kNames, kCount);
    const uint32_t sig1 = sig_for(kNames2, kCount2);

    float v0[kCount]  = {10, 20, 30, 40, 50};
    float v1[kCount2] = {70, 80, 90};
    CHECK(DroneCAN_Storage::save_all(v0, kCount, kPage, 0, sig0));
    CHECK(DroneCAN_Storage::save_all(v1, kCount2, kPage, 1, sig1));
    CHECK(DroneCAN_Storage::load(v0, kCount, kPage, 0, sig0));

    // Fill close to full, leaving room for the marker write below.
    const int headroom = kSlots - (int)(kCount + kCount2) - 4;
    for (int i = 0; i < headroom; i++) {
        CHECK(DroneCAN_Storage::save(2, (float)i, kCount, kPage, 0, sig0));
    }
    CHECK(DroneCAN_Storage::save(2, 555.0f, kCount, kPage, 0, sig0));

    // Cut power partway through the compaction that the next writes force.
    arm_power_loss(2);
    for (int i = 0; i < 10; i++) {
        if (!DroneCAN_Storage::save(2, 999.0f, kCount, kPage, 0, sig0)) {
            break;
        }
    }
    CHECK(!DroneCAN_Storage::healthy());

    reboot();

    float r0[kCount]  = {};
    float r1[kCount2] = {};
    CHECK(DroneCAN_Storage::load(r0, kCount, kPage, 0, sig0));
    CHECK(DroneCAN_Storage::load(r1, kCount2, kPage, 1, sig1));

    // The state committed before the cut must survive, for both instances.
    CHECK_FEQ(r0[0], 10.0f);
    CHECK_FEQ(r1[0], 70.0f);
    CHECK_FEQ(r1[2], 90.0f);
    CHECK(DroneCAN_Storage::healthy());

    // And storage stays usable.
    CHECK(DroneCAN_Storage::save(3, 44.0f, kCount, kPage, 0, sig0));
    CHECK(DroneCAN_Storage::save(0, 71.0f, kCount2, kPage, 1, sig1));
    float c0[kCount] = {}, c1[kCount2] = {};
    CHECK(DroneCAN_Storage::load(c0, kCount, kPage, 0, sig0));
    CHECK(DroneCAN_Storage::load(c1, kCount2, kPage, 1, sig1));
    CHECK_FEQ(c0[3], 44.0f);
    CHECK_FEQ(c1[0], 71.0f);
}

static void test_unwritten_index_keeps_caller_default()
{
    begin_test("an index never saved keeps the caller default");
    const uint32_t sig = sig_for(kNames, kCount);

    // Only index 1 is ever written.
    CHECK(DroneCAN_Storage::save(1, 123.0f, kCount, kPage, 0, sig));
    reboot();

    float read[kCount] = {9, 9, 9, 9, 9};
    CHECK(DroneCAN_Storage::load(read, kCount, kPage, 0, sig));
    CHECK_FEQ(read[1], 123.0f);
    CHECK_FEQ(read[0], 9.0f);   // untouched -> caller default preserved
    CHECK_FEQ(read[4], 9.0f);
}

// ---------------------------------------------------------------------------

int main()
{
    printf("storage.cpp log-structured tests -- %s geometry\n",
           (WRITE_UNIT == 32) ? "H7 (128 KB sector, 32 B word)"
                              : "L4 (2 KB page, 8 B word)");
    printf("  page %zu B, record %zu B, header %zu B -> %d records per page\n\n",
           PAGE_SIZE, kRecord, kHeader, kSlots);

    test_first_boot_returns_defaults();
    test_save_all_then_load();
    test_single_save_last_write_wins();
    test_write_path_never_erases();
    test_signature_change_rejects_stored_values();
    test_compaction_when_sector_fills();
    test_power_loss_during_sector_switch();
    test_power_loss_during_first_save();
    test_reboot_erases_stale_sector();
    test_oversized_parameter_set_rejected();
    test_two_instances_are_independent();
    test_instance_layout_change_spares_the_other();
    test_compaction_preserves_both_instances();
    test_power_loss_mid_switch_keeps_both_instances();
    test_unwritten_index_keeps_caller_default();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
