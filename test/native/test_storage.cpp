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
    CHECK(DroneCAN_Storage::load(values, kCount, kPage, sig) == false);
    CHECK_FEQ(values[0], 1.0f);
    CHECK_FEQ(values[4], 5.0f);
    CHECK(DroneCAN_Storage::healthy());
}

static void test_save_all_then_load()
{
    begin_test("save_all then load round-trips");
    const uint32_t sig = sig_for(kNames, kCount);

    float written[kCount] = {100.0f, 1000000.0f, 1.0f, 50.0f, 0.25f};
    CHECK(DroneCAN_Storage::save_all(written, kCount, kPage, sig));

    float read[kCount] = {0, 0, 0, 0, 0};
    CHECK(DroneCAN_Storage::load(read, kCount, kPage, sig));
    for (size_t i = 0; i < kCount; i++) {
        CHECK_FEQ(read[i], written[i]);
    }
}

static void test_single_save_last_write_wins()
{
    begin_test("repeated single saves: last write wins");
    const uint32_t sig = sig_for(kNames, kCount);

    float initial[kCount] = {100.0f, 1000000.0f, 1.0f, 50.0f, 0.25f};
    CHECK(DroneCAN_Storage::save_all(initial, kCount, kPage, sig));

    for (int i = 0; i < 20; i++) {
        CHECK(DroneCAN_Storage::save(0, 42.0f + i, kCount, kPage, sig));
    }

    float read[kCount] = {};
    CHECK(DroneCAN_Storage::load(read, kCount, kPage, sig));
    CHECK_FEQ(read[0], 42.0f + 19);   // last value for index 0
    CHECK_FEQ(read[1], 1000000.0f);   // untouched indices survive
    CHECK_FEQ(read[4], 0.25f);
}

static void test_write_path_never_erases()
{
    begin_test("steady-state writes never erase");
    const uint32_t sig = sig_for(kNames, kCount);

    float values[kCount] = {1, 2, 3, 4, 5};
    CHECK(DroneCAN_Storage::save_all(values, kCount, kPage, sig));
    CHECK(DroneCAN_Storage::load(values, kCount, kPage, sig));

    // From here on, nothing should erase: this is the whole point of the
    // log-structured design on H7.
    reset_counters();
    for (int i = 0; i < 200; i++) {
        CHECK(DroneCAN_Storage::save(i % kCount, (float)i, kCount, kPage, sig));
    }
    CHECK(erase_count() == 0);
    CHECK(word_writes() == 200);   // exactly one 32-byte word per save
}

static void test_signature_change_rejects_stored_values()
{
    begin_test("changed parameter layout invalidates stored values");
    const uint32_t sig_old = sig_for(kNames, kCount);

    float written[kCount] = {11, 22, 33, 44, 55};
    CHECK(DroneCAN_Storage::save_all(written, kCount, kPage, sig_old));

    // Firmware update reorders the parameter list, keeping the same count.
    const char *reordered[] = {"BITRATE", "NODEID", "CANFD", "RATE_HZ", "GAIN"};
    const uint32_t sig_new = sig_for(reordered, kCount);
    CHECK(sig_old != sig_new);

    float defaults[kCount] = {1, 2, 3, 4, 5};
    // Must refuse the stale log rather than assigning 11 to BITRATE.
    CHECK(DroneCAN_Storage::load(defaults, kCount, kPage, sig_new) == false);
    CHECK_FEQ(defaults[0], 1.0f);
    CHECK_FEQ(defaults[1], 2.0f);
}

static void test_compaction_when_sector_fills()
{
    begin_test("log fills -> compaction into spare sector");
    const uint32_t sig = sig_for(kNames, kCount);

    float values[kCount] = {1, 2, 3, 4, 5};
    CHECK(DroneCAN_Storage::save_all(values, kCount, kPage, sig));

    const int slots = kSlots;

    reset_counters();
    for (int i = 0; i < slots + 10; i++) {
        CHECK(DroneCAN_Storage::save(2, (float)i, kCount, kPage, sig));
    }

    // Exactly one compaction should have happened, and it must not have
    // erased anything inline (the spare was pre-erased at load()).
    CHECK(erase_count() == 0);

    float read[kCount] = {};
    CHECK(DroneCAN_Storage::load(read, kCount, kPage, sig));
    CHECK_FEQ(read[2], (float)(slots + 9));
    CHECK_FEQ(read[0], 1.0f);   // values carried across the compaction
    CHECK_FEQ(read[4], 5.0f);
}

static void test_power_loss_during_sector_switch()
{
    begin_test("power loss mid-compaction recovers on next boot");
    const uint32_t sig = sig_for(kNames, kCount);

    float values[kCount] = {7, 8, 9, 10, 11};
    CHECK(DroneCAN_Storage::save_all(values, kCount, kPage, sig));
    CHECK(DroneCAN_Storage::load(values, kCount, kPage, sig));

    const int slots = kSlots;
    // Fill the log to one record short of full.
    for (int i = 0; i < slots - kCount - 1; i++) {
        CHECK(DroneCAN_Storage::save(2, (float)i, kCount, kPage, sig));
    }
    CHECK(DroneCAN_Storage::save(2, 1234.0f, kCount, kPage, sig));

    // The next save must compact. Cut power partway through the copy.
    reset_counters();
    arm_power_loss(3);
    DroneCAN_Storage::save(2, 4321.0f, kCount, kPage, sig);
    CHECK(!DroneCAN_Storage::healthy());   // failure was reported, not swallowed

    // Reboot.
    reboot();
    float read[kCount] = {0, 0, 0, 0, 0};
    const bool loaded = DroneCAN_Storage::load(read, kCount, kPage, sig);

    // The interrupted switch must not lose the previously committed set.
    CHECK(loaded);
    CHECK_FEQ(read[0], 7.0f);
    CHECK_FEQ(read[2], 1234.0f);   // last value committed before the cut
    CHECK_FEQ(read[4], 11.0f);
    CHECK(DroneCAN_Storage::healthy());

    // And storage must be usable again afterwards.
    CHECK(DroneCAN_Storage::save(1, 99.0f, kCount, kPage, sig));
    float read2[kCount] = {};
    CHECK(DroneCAN_Storage::load(read2, kCount, kPage, sig));
    CHECK_FEQ(read2[1], 99.0f);
    CHECK_FEQ(read2[2], 1234.0f);
}

static void test_power_loss_during_first_save()
{
    begin_test("power loss during first save leaves defaults intact");
    const uint32_t sig = sig_for(kNames, kCount);

    float values[kCount] = {5, 6, 7, 8, 9};
    // The IN_USE header slot costs kSlotWords flash-word writes, so records 0
    // and 1 land after it and record 2 is the one that dies.
    arm_power_loss(kSlotWords + 2);
    DroneCAN_Storage::save_all(values, kCount, kPage, sig);

    reboot();
    float read[kCount] = {1, 2, 3, 4, 5};
    // A partial log is still a valid log -- whatever committed is readable,
    // and nothing may be corrupt. Indices that never landed keep defaults.
    DroneCAN_Storage::load(read, kCount, kPage, sig);
    CHECK_FEQ(read[0], 5.0f);   // record 0 committed
    CHECK_FEQ(read[1], 6.0f);   // record 1 committed
    CHECK_FEQ(read[2], 3.0f);   // record 2 never landed -> default kept
    CHECK_FEQ(read[4], 5.0f);

    // Storage remains writable.
    CHECK(DroneCAN_Storage::save_all(values, kCount, kPage, sig));
    float read2[kCount] = {};
    CHECK(DroneCAN_Storage::load(read2, kCount, kPage, sig));
    CHECK_FEQ(read2[2], 7.0f);
}

static void test_reboot_erases_stale_sector()
{
    begin_test("boot erases the sector left FULL by a compaction");
    const uint32_t sig = sig_for(kNames, kCount);

    float values[kCount] = {1, 2, 3, 4, 5};
    CHECK(DroneCAN_Storage::save_all(values, kCount, kPage, sig));
    CHECK(DroneCAN_Storage::load(values, kCount, kPage, sig));

    const int slots = kSlots;
    for (int i = 0; i < slots + 5; i++) {
        CHECK(DroneCAN_Storage::save(3, (float)i, kCount, kPage, sig));
    }

    // A compaction has run, so one sector is FULL and not yet erased.
    reset_counters();
    float read[kCount] = {};
    CHECK(DroneCAN_Storage::load(read, kCount, kPage, sig));
    CHECK(erase_count() == 1);   // the deferred erase is paid here, at boot

    // Second boot has nothing left to clean up.
    reset_counters();
    CHECK(DroneCAN_Storage::load(read, kCount, kPage, sig));
    CHECK(erase_count() == 0);
}

static void test_oversized_parameter_set_rejected()
{
    begin_test("parameter count beyond MAX_PARAMS is refused, not overflowed");
    const uint32_t sig = sig_for(kNames, kCount);
    const size_t too_many = DroneCAN_Storage::MAX_PARAMS + 1;

    static float big[DroneCAN_Storage::MAX_PARAMS + 8] = {};
    CHECK(DroneCAN_Storage::save(0, 1.0f, too_many, kPage, sig) == false);
    (void)big;
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

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
