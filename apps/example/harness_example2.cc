#include <cstdio>
#include <bench/harness.hh>
#include <workload/microbench/assembly/armv7e-m_ubenchmarks.hh>

/* Benchmark Harness with lambda callables with captured inputs,
 * no outputs.
 *
 */
int main() {
  using namespace bench;

  int x, y, z;
  x = 1; y = 2; z = 3;

  // Pass by value
  // 
  auto funcA = [x, y, z]() -> void {
    // z = x + y; // This is illegal. Z is read-only
    // x = z + 2; // Same with x, read-only
    int w = x + y + z;
    int v = w + 2;
    w += v;
  };

  // Pass by reference
  auto funcB = [&x, &y, &z]() -> void {
    z = x + y;
    x = z + 2;
  };

  // Check the current values of x, y, z
  auto funcC = [x, y, z]() -> void {
    printf("x, y, z after passing by reference: %i, %i, %i\n",
           x, y, z);
  };
  
  
  // What happens if we use volatile pass by value inputs?
  volatile int vx, vy, vz;
  vx = 1; vy = 2; vz = 3;
  auto funcD = [vx, vy, vz]() -> void {
    int vw = vx + vy + vz;
    int vv = vz + vw;
    vw += vv;
  };

  // What about volatile pass by reference?
  auto funcE = [&vx, &vy, &vz]() -> void {
    vz = vx + vy;
    vx = vz + 2;
  };

  // Let's double check the effect of always_inline 
  auto funcF = [x, y, z]() -> void __attribute__((always_inline)) {
    int w = x + y + z;
    int v = w + 2;
    w += v;
  };

  // This time using pass by reference
  auto funcG = [&x, &y, &z]() -> void __attribute__((always_inline)) {
    z = x + y;
    x = z + 2;
  };

  // And what about noinline with pass by value?
  auto funcH = [x, y, z]() -> void __attribute__((noinline)) {
    int w = x + y + z;
    int v = w + 2;
    w += v;
  };

  // And again, pass by reference
  auto funcI = [&x, &y, &z]() -> void __attribute__((noinline)) {
    z = x + y;
    x = z + 2;
  };

  // Lastly what about volatiles
  auto funcJ = [vx, vy, vz]() -> void __attribute__((always_inline)) {
    int vw = vx + vy + vz;
    int vv = vz + vw;
    vw += vv;
  };

  auto funcK = [vx, vy, vz]() -> void __attribute__((noinline)) {
    int vw = vx + vy + vz;
    int vv = vz + vw;
    vw += vv;
  };

  auto funcL = [&vx, &vy, &vz]() -> void __attribute__((always_inline)) {
    vz = vx + vy;
    vx = vz + 2;
  };

  auto funcM = [&vx, &vy, &vz]() -> void __attribute__((noinline)) {
    vz = vx + vy;
    vx = vz + 2;
  };

  MultiHarness harness1(funcA, funcB, funcC, funcD, funcE, funcF,
                        funcG, funcH, funcI, funcJ, funcK, funcL,
                        funcM);
  harness1.run();

  return 0;
}
