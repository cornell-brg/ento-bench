#ifndef MULTI_HARNESS_H
#define MULTI_HARNESS_H

// MultiHarness — drive N back-to-back microbenchmarks under a single trigger
// window so STLINK-V3PWR (or similar) records one continuous capture per
// firmware image. Mirrors the LATENCY_MEASUREMENT delay choreography in
// Harness::run() (harness.h), hoisted from per-rep granularity to per-binary
// granularity so the energy analyzer can map ROI edges to bench names.
//
// Each entry in the BenchEntry table points at a parameter-less kernel
// function (typically a raw `kernel_<name>` from src/generated/*.S). The
// harness emits a parseable BENCHES: line at startup, drives the trigger pin
// HIGH once, runs each kernel under start_roi()/end_roi(), then drives the
// trigger pin LOW once at teardown.

#include <array>
#include <cstddef>
#include <cstdio>
#include <ento-bench/roi.h>

#ifdef STM32_BUILD
#include <ento-mcu/timing.h>
#endif

namespace EntoBench {

struct BenchEntry {
    const char* name;
    void (*kernel)(void);
    bool needs_warmup;       // true for ART loop-capacity benches (iter_count < ~100)
    bool needs_nvic_reset;   // true for exception-mutating benches
};

template<std::size_t N>
class MultiHarness {
public:
    constexpr MultiHarness(const std::array<BenchEntry, N>& benches,
                           const char* binary_name)
        : benches_(benches), name_(binary_name) {}

    void setup()
    {
        print_bench_list();

#if defined(STM32_BUILD) && !defined(GEM5_SEMIHOSTING)
        init_roi_tracking();
#else
        init_roi_tracking();
#endif

#if defined(STM32_BUILD) && defined(LATENCY_MEASUREMENT)
        // Mirror Harness::run() at harness.h:268 — let the trigger/latency
        // pins settle before any further state mutation.
        __asm__ volatile("" ::: "memory");
        Delay::ms(5);

        // Mirror harness.h:289-290 — quiet window before driving trigger
        // high so the analyzer's leading-edge detector has clean baseline.
        Delay::ms(100);

        // Drive the trigger pin HIGH once per binary; STLINK-V3PWR is now
        // armed for the entire run. trigger_pin_low() in teardown() ends
        // the recording.
        trigger_pin_high();

        // Mirror harness.h:319 — settle time after trigger asserts before
        // any ROI edges land on the trace.
        Delay::ms(50);
#endif
    }

    void run()
    {
        for (std::size_t i = 0; i < N; ++i) {
            const BenchEntry& b = benches_[i];

            // One line per bench so the post-processor can map ROI index
            // to bench name (output ordering is identical to ROI ordering).
            printf("BENCH %zu: %s\n", i, b.name);

            if (b.needs_nvic_reset) {
                nvic_reset();
            }

            if (b.needs_warmup) {
                // Warmup outside the ROI window — first-iteration cold-cache
                // bias on small-iter-count benches (e.g. loop_art_64 with 32
                // iterations) is non-trivial. See microbench_design.md §4.
                b.kernel();
            }

            __asm__ volatile("" ::: "memory");
            start_roi();
            b.kernel();
            end_roi();
            __asm__ volatile("" ::: "memory");

#if defined(STM32_BUILD) && defined(LATENCY_MEASUREMENT)
            // Mirror harness.h:332 — gap between consecutive ROIs so the
            // analyzer's edge-pair extractor cleanly separates them.
            Delay::ms(10);
#endif
        }
    }

    void teardown()
    {
#if defined(STM32_BUILD) && defined(LATENCY_MEASUREMENT)
        // Mirror harness.h:537 — settle window before deasserting trigger
        // so the trailing ROI's energy isn't truncated.
        Delay::ms(50);
        trigger_pin_low();
#endif
    }

private:
    // Zero NVIC ISER/ICER/ISPR/ICPR/IPR registers. Cheap (~10 cycles) and
    // prevents cross-contamination from exception-mutating benches into
    // whatever bench runs next. ARMv7-M ARM B3.4 — Nested Vectored Interrupt
    // Controller register file at SCS_BASE + 0x100/0x180/0x200/0x280/0x400.
    static void nvic_reset()
    {
#if defined(STM32_BUILD)
        // 8 ISER/ICER/ISPR/ICPR words (max 240 IRQs supported by ARMv7-M).
        // STM32G4 uses ~100 IRQs so 4 words suffice, but writing all 8 is
        // harmless on devices that alias unused entries to RES0.
        constexpr uintptr_t SCS = 0xE000E000UL;
        volatile uint32_t* iser = reinterpret_cast<volatile uint32_t*>(SCS + 0x100);
        volatile uint32_t* icer = reinterpret_cast<volatile uint32_t*>(SCS + 0x180);
        volatile uint32_t* ispr = reinterpret_cast<volatile uint32_t*>(SCS + 0x200);
        volatile uint32_t* icpr = reinterpret_cast<volatile uint32_t*>(SCS + 0x280);
        volatile uint32_t* ipr  = reinterpret_cast<volatile uint32_t*>(SCS + 0x400);
        for (std::size_t k = 0; k < 8; ++k) {
            // ICER/ICPR are write-1-to-clear; writing 0xFFFFFFFF disables/clears.
            // Then zero ISER/ISPR explicitly so the visible state is "no IRQ
            // enabled, none pending." Order: clear-pending then disable.
            icpr[k] = 0xFFFFFFFFUL;
            icer[k] = 0xFFFFFFFFUL;
            iser[k] = 0;
            ispr[k] = 0;
        }
        for (std::size_t k = 0; k < 60; ++k) {
            ipr[k] = 0;
        }
        __asm__ volatile("dsb 0xF":::"memory");
        __asm__ volatile("isb 0xF":::"memory");
#endif
    }

    void print_bench_list()
    {
        // Single line, comma-separated, BENCHES: prefix. Energy-analyzer
        // splits on ", " and matches against subsequent BENCH N: <name> lines.
        printf("BENCHES: ");
        for (std::size_t i = 0; i < N; ++i) {
            printf("%s%s", benches_[i].name, (i + 1 == N) ? "\n" : ", ");
        }
        printf("MULTIBENCH_NAME: %s\n", name_);
        printf("MULTIBENCH_COUNT: %zu\n", N);
    }

    std::array<BenchEntry, N> benches_;
    const char* name_;
};

// CTAD deduction guide — lets `MultiHarness h(table, "name")` work without
// spelling out the array length.
template<std::size_t N>
MultiHarness(const std::array<BenchEntry, N>&, const char*) -> MultiHarness<N>;

}  // namespace EntoBench

#endif  // MULTI_HARNESS_H
