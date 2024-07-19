#include "armv7e-m_ubench.hh"

int main()
{
  auto func = []() -> void __attribute__((always_inline))
  {
    assembly::add<8>();
    assembly::mul<8>();
  };
  func();
  return 0;
  
}
