#ifndef ARMV7E_M_UBENCH_HH
#define ARMV7E_M_UBENCH_HH

namespace assembly
{
  
/* Definitions */
// Integer Instructions
template<int N>
inline void add() __attribute__((always_inline));

template<int N>
inline void mul() __attribute__((always_inline));

template<int N>
inline void muls() __attribute__((always_inline));

template<int N>
inline void mla() __attribute__((always_inline));

template<int N>
inline void sdiv() __attribute__((always_inline));

template<int N>
inline void mla() __attribute__((always_inline));

template<int N>
inline void mls() __attribute__((always_inline));

template<int N>
inline void smull() __attribute__((always_inline));

template<int N>
inline void umull() __attribute__((always_inline));

template<int N>
inline void smlal() __attribute__((always_inline));

template<int N>
inline void umlal() __attribute__((always_inline));

template<int N>
inline void sdiv() __attribute__((always_inline));

template<int N>
inline void udiv() __attribute__((always_inline));

template<int N>
inline void ssat() __attribute__((always_inline));

template<int N>
inline void usat() __attribute__((always_inline));

template<int N>
inline void cmp() __attribute__((always_inline));

template<int N>
inline void cmn() __attribute__((always_inline));

template<int N>
inline void and_inst() __attribute__((always_inline));

template<int N>
inline void eor_inst() __attribute__((always_inline));

template<int N>
inline void or_inst() __attribute__((always_inline));

template<int N>
inline void bic_inst() __attribute__((always_inline));

template<int N>
inline void lsl() __attribute__((always_inline));

template<int N>
inline void lsr() __attribute__((always_inline));

template<int N>
inline void asr() __attribute__((always_inline));

template<int N>
inline void ror() __attribute__((always_inline));

template<int N>
inline void clz() __attribute__((always_inline));

template<int N>
inline void ubfx() __attribute__((always_inline));

template<int N>
inline void sbfx() __attribute__((always_inline));

template<int N>
inline void bfc() __attribute__((always_inline));

template<int N>
inline void bfi() __attribute__((always_inline));

template<int N>
inline void rev() __attribute__((always_inline));

template<int N>
inline void rev16() __attribute__((always_inline));

template<int N>
inline void rbit() __attribute__((always_inline));

// DSP Instructions


// FPU Instructions
template<int N>
inline void vldr_f32() __attribute__((always_inline));

template<int N>
inline void vldm_f32() __attribute__((always_inline));

template<int N>
inline void vmul_f32() __attribute__((always_inline));

template<int N>
inline void vadd_f32() __attribute__((always_inline));

template<int N>
inline void vsub_f32() __attribute__((always_inline));

template<int N>
inline void vmla_f32() __attribute__((always_inline));

template<int N>
inline void vmls_f32() __attribute__((always_inline));

template<int N>
inline void vnmla_f32() __attribute__((always_inline));

template<int N>
inline void vnmls_f32() __attribute__((always_inline));

template<int N>
inline void vfma_f32() __attribute__((always_inline));

template<int N>
inline void vfms_f32() __attribute__((always_inline));

template<int N>
inline void vfnma_f32() __attribute__((always_inline));

template<int N>
inline void vfnms_f32() __attribute__((always_inline));

template<int N>
inline void vneg_f32() __attribute__((always_inline));

template<int N>
inline void vnmul_f32() __attribute__((always_inline));

template<int N>
inline void vstm_f32() __attribute__((always_inline));

template<int N>
inline void vstr_f32() __attribute__((always_inline));

template<int N>
inline void vsqrt_f32() __attribute__((always_inline));

/* Implementation */

template<int N>
inline void add() {
  //for (int i = 0; i < N; i++)
  if constexpr (N > 0)
  {
    asm volatile (
      "add r0, r0, r0 \n\t"
      "add r1, r1, r1 \n\t"
      "add r2, r2, r2 \n\t"
      "add r3, r3, r3 \n\t"
      "add r4, r4, r4 \n\t"
      "add r5, r5, r5 \n\t"
      "add r6, r6, r6 \n\t"
      "add r7, r7, r7 \n\t"
      : // No outputs
      : // No inputs
      : "r0", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
    );
    add<N-1>();
  }
}

template<int N>
inline void mul() {
  //for (int i = 0; i < N; i++)
  if constexpr (N > 0)
  {
    asm volatile (
      "mul r0, r0, r0 \n\t"
      "mul r1, r1, r1 \n\t"
      "mul r2, r2, r2 \n\t"
      "mul r3, r3, r3 \n\t"
      "mul r4, r4, r4 \n\t"
      "mul r5, r5, r5 \n\t"
      "mul r6, r6, r6 \n\t"
      "mul r7, r7, r7 \n\t"
      : // No outputs
      : // No inputs
      : "c0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
    );
    mul<N-1>();
  }
}

template<int N>
inline void muls() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "muls r0, r0, r0 \n\t"
    "muls r1, r1, r1 \n\t"
    "muls r2, r2, r2 \n\t"
    "muls r3, r3, r3 \n\t"
    "muls r4, r4, r4 \n\t"
    "muls r5, r5, r5 \n\t"
    "muls r6, r6, r6 \n\t"
    "muls r7, r7, r7 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void mla() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "mla r0, r1, r2, r3 \n\t"
    "mla r1, r2, r3, r0 \n\t"
    "mla r2, r3, r0, r1 \n\t"
    "mla r3, r0, r1, r2 \n\t"
    "mla r4, r5, r6, r7 \n\t"
    "mla r5, r6, r7, r4 \n\t"
    "mla r6, r7, r4, r5 \n\t"
    "mla r7, r4, r5, r6 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void mls() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "mls r0, r1, r2, r3 \n\t"
    "mls r1, r2, r3, r0 \n\t"
    "mls r2, r3, r0, r1 \n\t"
    "mls r3, r0, r1, r2 \n\t"
    "mls r4, r5, r6, r7 \n\t"
    "mls r5, r6, r7, r4 \n\t"
    "mls r6, r7, r4, r5 \n\t"
    "mls r7, r4, r5, r6 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void smull() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "smull r0, r1, r2, r3 \n\t"
    "smull r1, r2, r3, r0 \n\t"
    "smull r2, r3, r0, r1 \n\t"
    "smull r3, r0, r1, r2 \n\t"
    "smull r4, r5, r6, r7 \n\t"
    "smull r5, r6, r7, r4 \n\t"
    "smull r6, r7, r4, r5 \n\t"
    "smull r7, r4, r5, r6 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void umull() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "umull r0, r1, r2, r3 \n\t"
    "umull r1, r2, r3, r0 \n\t"
    "umull r2, r3, r0, r1 \n\t"
    "umull r3, r0, r1, r2 \n\t"
    "umull r4, r5, r6, r7 \n\t"
    "umull r5, r6, r7, r4 \n\t"
    "umull r6, r7, r4, r5 \n\t"
    "umull r7, r4, r5, r6 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void smlal() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "smlal r0, r1, r2, r3 \n\t"
    "smlal r1, r2, r3, r0 \n\t"
    "smlal r2, r3, r0, r1 \n\t"
    "smlal r3, r0, r1, r2 \n\t"
    "smlal r4, r5, r6, r7 \n\t"
    "smlal r5, r6, r7, r4 \n\t"
    "smlal r6, r7, r4, r5 \n\t"
    "smlal r7, r4, r5, r6 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void umlal() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "umlal r0, r1, r2, r3 \n\t"
    "umlal r1, r2, r3, r0 \n\t"
    "umlal r2, r3, r0, r1 \n\t"
    "umlal r3, r0, r1, r2 \n\t"
    "umlal r4, r5, r6, r7 \n\t"
    "umlal r5, r6, r7, r4 \n\t"
    "umlal r6, r7, r4, r5 \n\t"
    "umlal r7, r4, r5, r6 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7"
  );
}


template<int N>
inline void sdiv() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "sdiv r0, r1, r2 \n\t"
    "sdiv r1, r2, r3 \n\t"
    "sdiv r2, r3, r0 \n\t"
    "sdiv r3, r0, r1 \n\t"
    "sdiv r4, r5, r6 \n\t"
    "sdiv r5, r6, r7 \n\t"
    "sdiv r6, r7, r4 \n\t"
    "sdiv r7, r4, r5 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void udiv() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "udiv r0, r1, r2 \n\t"
    "udiv r1, r2, r3 \n\t"
    "udiv r2, r3, r0 \n\t"
    "udiv r3, r0, r1 \n\t"
    "udiv r4, r5, r6 \n\t"
    "udiv r5, r6, r7 \n\t"
    "udiv r6, r7, r4 \n\t"
    "udiv r7, r4, r5 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void ssat() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "ssat r0, #8, r1 \n\t"
    "ssat r1, #8, r2 \n\t"
    "ssat r2, #8, r3 \n\t"
    "ssat r3, #8, r4 \n\t"
    "ssat r4, #8, r5 \n\t"
    "ssat r5, #8, r6 \n\t"
    "ssat r6, #8, r7 \n\t"
    "ssat r7, #8, r0 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void usat() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "usat r0, #8, r1 \n\t"
    "usat r1, #8, r2 \n\t"
    "usat r2, #8, r3 \n\t"
    "usat r3, #8, r4 \n\t"
    "usat r4, #8, r5 \n\t"
    "usat r5, #8, r6 \n\t"
    "usat r6, #8, r7 \n\t"
    "usat r7, #8, r0 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void cmp() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "cmp r0, r1 \n\t"
    "cmp r1, r2 \n\t"
    "cmp r2, r3 \n\t"
    "cmp r3, r4 \n\t"
    "cmp r4, r5 \n\t"
    "cmp r5, r6 \n\t"
    "cmp r6, r7 \n\t"
    "cmp r7, r0 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void cmn() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "cmn r0, r1 \n\t"
    "cmn r1, r2 \n\t"
    "cmn r2, r3 \n\t"
    "cmn r3, r4 \n\t"
    "cmn r4, r5 \n\t"
    "cmn r5, r6 \n\t"
    "cmn r6, r7 \n\t"
    "cmn r7, r0 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void and_inst() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "and r0, r1, r2 \n\t"
    "and r1, r2, r3 \n\t"
    "and r2, r3, r4 \n\t"
    "and r3, r4, r5 \n\t"
    "and r4, r5, r6 \n\t"
    "and r5, r6, r7 \n\t"
    "and r6, r7, r0 \n\t"
    "and r7, r0, r1 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void eor_inst() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "eor r0, r1, r2 \n\t"
    "eor r1, r2, r3 \n\t"
    "eor r2, r3, r4 \n\t"
    "eor r3, r4, r5 \n\t"
    "eor r4, r5, r6 \n\t"
    "eor r5, r6, r7 \n\t"
    "eor r6, r7, r0 \n\t"
    "eor r7, r0, r1 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void or_inst() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "orr r0, r1, r2 \n\t"
    "orr r1, r2, r3 \n\t"
    "orr r2, r3, r4 \n\t"
    "orr r3, r4, r5 \n\t"
    "orr r4, r5, r6 \n\t"
    "orr r5, r6, r7 \n\t"
    "orr r6, r7, r0 \n\t"
    "orr r7, r0, r1 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void bic_inst() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "bic r0, r1, r2 \n\t"
    "bic r1, r2, r3 \n\t"
    "bic r2, r3, r4 \n\t"
    "bic r3, r4, r5 \n\t"
    "bic r4, r5, r6 \n\t"
    "bic r5, r6, r7 \n\t"
    "bic r6, r7, r0 \n\t"
    "bic r7, r0, r1 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7"
  );
}

template<int N>
inline void lsl() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "lsl r0, r0, #1 \n\t"
    "lsl r1, r1, #1 \n\t"
    "lsl r2, r2, #1 \n\t"
    "lsl r3, r3, #1 \n\t"
    "lsl r4, r4, #1 \n\t"
    "lsl r5, r5, #1 \n\t"
    "lsl r6, r6, #1 \n\t"
    "lsl r7, r7, #1 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void lsr() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "lsr r0, r0, #1 \n\t"
    "lsr r1, r1, #1 \n\t"
    "lsr r2, r2, #1 \n\t"
    "lsr r3, r3, #1 \n\t"
    "lsr r4, r4, #1 \n\t"
    "lsr r5, r5, #1 \n\t"
    "lsr r6, r6, #1 \n\t"
    "lsr r7, r7, #1 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void asr() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "asr r0, r0, #1 \n\t"
    "asr r1, r1, #1 \n\t"
    "asr r2, r2, #1 \n\t"
    "asr r3, r3, #1 \n\t"
    "asr r4, r4, #1 \n\t"
    "asr r5, r5, #1 \n\t"
    "asr r6, r6, #1 \n\t"
    "asr r7, r7, #1 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void ror() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "ror r0, r0, #1 \n\t"
    "ror r1, r1, #1 \n\t"
    "ror r2, r2, #1 \n\t"
    "ror r3, r3, #1 \n\t"
    "ror r4, r4, #1 \n\t"
    "ror r5, r5, #1 \n\t"
    "ror r6, r6, #1 \n\t"
    "ror r7, r7, #1 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void clz() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "clz r0, r0 \n\t"
    "clz r1, r1 \n\t"
    "clz r2, r2 \n\t"
    "clz r3, r3 \n\t"
    "clz r4, r4 \n\t"
    "clz r5, r5 \n\t"
    "clz r6, r6 \n\t"
    "clz r7, r7 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void ubfx() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "ubfx r0, r1, #0, #8 \n\t"
    "ubfx r1, r2, #0, #8 \n\t"
    "ubfx r2, r3, #0, #8 \n\t"
    "ubfx r3, r4, #0, #8 \n\t"
    "ubfx r4, r5, #0, #8 \n\t"
    "ubfx r5, r6, #0, #8 \n\t"
    "ubfx r6, r7, #0, #8 \n\t"
    "ubfx r7, r0, #0, #8 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void sbfx() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "sbfx r0, r1, #0, #8 \n\t"
    "sbfx r1, r2, #0, #8 \n\t"
    "sbfx r2, r3, #0, #8 \n\t"
    "sbfx r3, r4, #0, #8 \n\t"
    "sbfx r4, r5, #0, #8 \n\t"
    "sbfx r5, r6, #0, #8 \n\t"
    "sbfx r6, r7, #0, #8 \n\t"
    "sbfx r7, r0, #0, #8 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void bfc() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "bfc r0, #0, #8 \n\t"
    "bfc r1, #0, #8 \n\t"
    "bfc r2, #0, #8 \n\t"
    "bfc r3, #0, #8 \n\t"
    "bfc r4, #0, #8 \n\t"
    "bfc r5, #0, #8 \n\t"
    "bfc r6, #0, #8 \n\t"
    "bfc r7, #0, #8 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void bfi() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "bfi r0, r1, #0, #8 \n\t"
    "bfi r1, r2, #0, #8 \n\t"
    "bfi r2, r3, #0, #8 \n\t"
    "bfi r3, r4, #0, #8 \n\t"
    "bfi r4, r5, #0, #8 \n\t"
    "bfi r5, r6, #0, #8 \n\t"
    "bfi r6, r7, #0, #8 \n\t"
    "bfi r7, r0, #0, #8 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void rev() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "rev r0, r0 \n\t"
    "rev r1, r1 \n\t"
    "rev r2, r2 \n\t"
    "rev r3, r3 \n\t"
    "rev r4, r4 \n\t"
    "rev r5, r5 \n\t"
    "rev r6, r6 \n\t"
    "rev r7, r7 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void rev16() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "rev16 r0, r0 \n\t"
    "rev16 r1, r1 \n\t"
    "rev16 r2, r2 \n\t"
    "rev16 r3, r3 \n\t"
    "rev16 r4, r4 \n\t"
    "rev16 r5, r5 \n\t"
    "rev16 r6, r6 \n\t"
    "rev16 r7, r7 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}

template<int N>
inline void rbit() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "rbit r0, r0 \n\t"
    "rbit r1, r1 \n\t"
    "rbit r2, r2 \n\t"
    "rbit r3, r3 \n\t"
    "rbit r4, r4 \n\t"
    "rbit r5, r5 \n\t"
    "rbit r6, r6 \n\t"
    "rbit r7, r7 \n\t"
    : // No outputs
    : // No inputs
    : "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7" // Clobber list
  );
}


/* FPU Implementations */

template<int N>
inline void vldr_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vldr s0, [r0] \n\t"
    "vldr s1, [r0] \n\t"
    "vldr s2, [r0] \n\t"
    "vldr s3, [r0] \n\t"
    "vldr s4, [r0] \n\t"
    "vldr s5, [r0] \n\t"
    "vldr s6, [r0] \n\t"
    "vldr s7, [r0] \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vldm_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vldm r0!, {s0-s7} \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vmul_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vmul.f32 s0, s0, s0 \n\t"
    "vmul.f32 s1, s1, s1 \n\t"
    "vmul.f32 s2, s2, s2 \n\t"
    "vmul.f32 s3, s3, s3 \n\t"
    "vmul.f32 s4, s4, s4 \n\t"
    "vmul.f32 s5, s5, s5 \n\t"
    "vmul.f32 s6, s6, s6 \n\t"
    "vmul.f32 s7, s7, s7 \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vadd_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vadd.f32 s0, s0, s0 \n\t"
    "vadd.f32 s1, s1, s1 \n\t"
    "vadd.f32 s2, s2, s2 \n\t"
    "vadd.f32 s3, s3, s3 \n\t"
    "vadd.f32 s4, s4, s4 \n\t"
    "vadd.f32 s5, s5, s5 \n\t"
    "vadd.f32 s6, s6, s6 \n\t"
    "vadd.f32 s7, s7, s7 \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vsub_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vsub.f32 s0, s0, s0 \n\t"
    "vsub.f32 s1, s1, s1 \n\t"
    "vsub.f32 s2, s2, s2 \n\t"
    "vsub.f32 s3, s3, s3 \n\t"
    "vsub.f32 s4, s4, s4 \n\t"
    "vsub.f32 s5, s5, s5 \n\t"
    "vsub.f32 s6, s6, s6 \n\t"
    "vsub.f32 s7, s7, s7 \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vmla_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vmla.f32 s0, s0, s0 \n\t"
    "vmla.f32 s1, s1, s1 \n\t"
    "vmla.f32 s2, s2, s2 \n\t"
    "vmla.f32 s3, s3, s3 \n\t"
    "vmla.f32 s4, s4, s4 \n\t"
    "vmla.f32 s5, s5, s5 \n\t"
    "vmla.f32 s6, s6, s6 \n\t"
    "vmla.f32 s7, s7, s7 \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vmls_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vmls.f32 s0, s0, s0 \n\t"
    "vmls.f32 s1, s1, s1 \n\t"
    "vmls.f32 s2, s2, s2 \n\t"
    "vmls.f32 s3, s3, s3 \n\t"
    "vmls.f32 s4, s4, s4 \n\t"
    "vmls.f32 s5, s5, s5 \n\t"
    "vmls.f32 s6, s6, s6 \n\t"
    "vmls.f32 s7, s7, s7 \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vnmla_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vnmla.f32 s0, s0, s0 \n\t"
    "vnmla.f32 s1, s1, s1 \n\t"
    "vnmla.f32 s2, s2, s2 \n\t"
    "vnmla.f32 s3, s3, s3 \n\t"
    "vnmla.f32 s4, s4, s4 \n\t"
    "vnmla.f32 s5, s5, s5 \n\t"
    "vnmla.f32 s6, s6, s6 \n\t"
    "vnmla.f32 s7, s7, s7 \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vnmls_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vnmls.f32 s0, s0, s0 \n\t"
    "vnmls.f32 s1, s1, s1 \n\t"
    "vnmls.f32 s2, s2, s2 \n\t"
    "vnmls.f32 s3, s3, s3 \n\t"
    "vnmls.f32 s4, s4, s4 \n\t"
    "vnmls.f32 s5, s5, s5 \n\t"
    "vnmls.f32 s6, s6, s6 \n\t"
    "vnmls.f32 s7, s7, s7 \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vfma_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vfma.f32 s0, s0, s0 \n\t"
    "vfma.f32 s1, s1, s1 \n\t"
    "vfma.f32 s2, s2, s2 \n\t"
    "vfma.f32 s3, s3, s3 \n\t"
    "vfma.f32 s4, s4, s4 \n\t"
    "vfma.f32 s5, s5, s5 \n\t"
    "vfma.f32 s6, s6, s6 \n\t"
    "vfma.f32 s7, s7, s7 \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vfms_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vfms.f32 s0, s0, s0 \n\t"
    "vfms.f32 s1, s1, s1 \n\t"
    "vfms.f32 s2, s2, s2 \n\t"
    "vfms.f32 s3, s3, s3 \n\t"
    "vfms.f32 s4, s4, s4 \n\t"
    "vfms.f32 s5, s5, s5 \n\t"
    "vfms.f32 s6, s6, s6 \n\t"
    "vfms.f32 s7, s7, s7 \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vfnma_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vfnma.f32 s0, s0, s0 \n\t"
    "vfnma.f32 s1, s1, s1 \n\t"
    "vfnma.f32 s2, s2, s2 \n\t"
    "vfnma.f32 s3, s3, s3 \n\t"
    "vfnma.f32 s4, s4, s4 \n\t"
    "vfnma.f32 s5, s5, s5 \n\t"
    "vfnma.f32 s6, s6, s6 \n\t"
    "vfnma.f32 s7, s7, s7 \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vfnms_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vfnms.f32 s0, s0, s0 \n\t"
    "vfnms.f32 s1, s1, s1 \n\t"
    "vfnms.f32 s2, s2, s2 \n\t"
    "vfnms.f32 s3, s3, s3 \n\t"
    "vfnms.f32 s4, s4, s4 \n\t"
    "vfnms.f32 s5, s5, s5 \n\t"
    "vfnms.f32 s6, s6, s6 \n\t"
    "vfnms.f32 s7, s7, s7 \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vneg_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vneg.f32 s0, s0 \n\t"
    "vneg.f32 s1, s1 \n\t"
    "vneg.f32 s2, s2 \n\t"
    "vneg.f32 s3, s3 \n\t"
    "vneg.f32 s4, s4 \n\t"
    "vneg.f32 s5, s5 \n\t"
    "vneg.f32 s6, s6 \n\t"
    "vneg.f32 s7, s7 \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vnmul_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vnmul.f32 s0, s0, s0 \n\t"
    "vnmul.f32 s1, s1, s1 \n\t"
    "vnmul.f32 s2, s2, s2 \n\t"
    "vnmul.f32 s3, s3, s3 \n\t"
    "vnmul.f32 s4, s4, s4 \n\t"
    "vnmul.f32 s5, s5, s5 \n\t"
    "vnmul.f32 s6, s6, s6 \n\t"
    "vnmul.f32 s7, s7, s7 \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vstm_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vstm r0!, {s0-s7} \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vstr_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vstr s0, [r0] \n\t"
    "vstr s1, [r0] \n\t"
    "vstr s2, [r0] \n\t"
    "vstr s3, [r0] \n\t"
    "vstr s4, [r0] \n\t"
    "vstr s5, [r0] \n\t"
    "vstr s6, [r0] \n\t"
    "vstr s7, [r0] \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

template<int N>
inline void vsqrt_f32() {
  for (int i = 0; i < N; i++)
  asm volatile (
    "vsqrt.f32 s0, s0 \n\t"
    "vsqrt.f32 s1, s1 \n\t"
    "vsqrt.f32 s2, s2 \n\t"
    "vsqrt.f32 s3, s3 \n\t"
    "vsqrt.f32 s4, s4 \n\t"
    "vsqrt.f32 s5, s5 \n\t"
    "vsqrt.f32 s6, s6 \n\t"
    "vsqrt.f32 s7, s7 \n\t"
    : // No outputs
    : // No inputs
    : "s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7" // Clobber list
  );
}

} // End namespace assembly

#endif // ARMV7E_M_UBENCH_HH
