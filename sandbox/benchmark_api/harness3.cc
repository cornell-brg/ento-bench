#include <cstdio>
#include <array>
#include <concepts>
#include <functional>
#include "armv7e-m_ubench.hh"
#include <Eigen/Dense>

// Define a concept for callables with no arguments
template<typename T>
concept NoArg = requires(T t) {
    t();
};

void start_roi()
{
  printf("Starting ROI!");
}

void end_roi()
{
  printf("Ending ROI!");
}

// WorkloadWrapper class
template<typename Callable>
class WorkloadWrapper {
public:
    using ReturnType = decltype(std::declval<Callable>()());

    WorkloadWrapper(Callable& callable) : callable_{callable} {}

    ReturnType operator()() const {
        if constexpr (std::is_void_v<ReturnType>) {
            callable_();
        } else {
            return callable_();
        }
    }

private:
    Callable& callable_;
};

// BenchmarkHarness class
template <std::size_t N, NoArg... Callable>
class BenchmarkHarness {
public:
    BenchmarkHarness(Callable... funcs) : functions_{WorkloadWrapper<Callable>(funcs)...} {}

    void run() {
        run_impl(std::make_index_sequence<N>{});
    }

private:
    template<std::size_t... I>
    void run_impl(std::index_sequence<I...>) {
        (void(std::initializer_list<int>{
              (start_roi(), std::get<I>(functions_)(), end_roi(), 0)
              }), ...);
    }

    std::tuple<WorkloadWrapper<Callable>...> functions_;
};


template <NoArg... Callables>
BenchmarkHarness(Callables...) -> BenchmarkHarness<sizeof...(Callables), Callables...>;

inline int test_function()
{
  volatile int a = 0;
  volatile int b = 256;

  volatile int c = a + b + 2;
  volatile float d = 0.0f;
  volatile float e = d + 2.0f + 1000.0f;

  return c;
}

template <typename Scalar, int maxN, int StorageOrder>
Eigen::Matrix<Scalar, 4, 4>
example_pnp_function_dynamic(const Eigen::Matrix<Scalar, Eigen::Dynamic, 2,
                                         StorageOrder, maxN, 2>& xprime, 
                             const Eigen::Matrix<Scalar, Eigen::Dynamic, 3,
                                         StorageOrder, maxN, 3>& x) {
  return Eigen::Matrix<Scalar, 4, 4>::Zero(4, 4);
}

template <typename Scalar, int N, int StorageOrder>
Eigen::Matrix<Scalar, 4, 4>
example_pnp_function_fixed(const Eigen::Matrix<Scalar, N, 2,
                                               StorageOrder, N, 2>& xprime,
                           const Eigen::Matrix<Scalar, N, 3,
                                               StorageOrder, N, 3>& x) {
  return Eigen::Matrix<Scalar, 4, 4>::Zero(4, 4);
}


int main() {
    using namespace assembly;
    // Callables without inputs or outputs
    auto func1 = []() -> void __attribute__ ((always_inline)) { add<8>(); };
    auto func2 = []() -> void __attribute__ ((always_inline)) { add<16>(); };
    auto func3 = []() -> void __attribute__ ((always_inline)) { add<8>(); mul<8>(); };

    //BenchmarkHarness harness1(func1, func2, func3);
    //harness1.run();

    volatile int z = 0;
    int x = 10;
    int y = 20;

    float a = 1.0f;
    float b = 2.0f;

    // Function pointers with arguments
    auto funcA = [&z, &x, &y]() -> void __attribute__((always_inline)) {
        z = x + y;
    };
    auto funcB = [&a, &b]() -> float __attribute__ ((always_inline)) {
        add<8>();
        volatile float c = a + 1.0f + b + 1.0f;
        return c;
    };

    BenchmarkHarness harness2(funcA, funcB);
    harness2.run();

    mul<16>();
    // Testing with a Function Pointer that has no args
    BenchmarkHarness harness3(test_function);
    harness3.run();

    auto funcTest = []() -> void
    {
        test_function();
    };
    BenchmarkHarness harness4(funcTest);
    harness4.run();


    // ==== EIGEN TESTING =======
    using Scalar = float;
    constexpr int maxN = 100;
    constexpr int N = 50;  
    constexpr int storageOrder = Eigen::ColMajor;

    using Matrix_100dr_2c = Eigen::Matrix<Scalar, Eigen::Dynamic, 2, storageOrder, maxN, 2>;
    using Matrix_100dr_3c = Eigen::Matrix<Scalar, Eigen::Dynamic, 3, storageOrder, maxN, 3>;
    using Matrix_50r_2c  = Eigen::Matrix<Scalar, N, 2, storageOrder, 50, 2>;
    using Matrix_50r_3c  = Eigen::Matrix<Scalar, N, 3, storageOrder, 50, 3>;

    // Create example data
    //Eigen::Matrix<Scalar, Eigen::Dynamic, 2, storageOrder, maxN, 2> xprime_max = Eigen::Matrix<Scalar, maxN, 2>::Zero();
    //Eigen::Matrix<Scalar, Eigen::Dynamic, 3, storageOrder, maxN, 3> x_max = Eigen::Matrix<Scalar, maxN, 3>::Zero();
    auto x_100max = Matrix_100dr_3c::Zero(maxN, 3);
    auto xprime_100max = Matrix_100dr_2c::Zero(maxN, 2);
    auto x_runtime = x_100max.topRows(N);
    auto xprime_runtime = xprime_100max.topRows(N);
    auto x50 = Matrix_50r_3c::Zero();
    auto xprime50 = Matrix_50r_2c::Zero();


    // Single PnP function with fixed rows
    //auto func = example_pnp_function_fixed<Scalar, 50, storageOrder>;
    auto pnp_fixed = [&xprime50, &x50]() -> Eigen::Matrix<Scalar, 4, 4>
    {
        return example_pnp_function_fixed<Scalar, 50, storageOrder>(xprime50, x50);
    };

    //auto harness_singleFuncFixedRows = PnPHarness(func);
    BenchmarkHarness pnpFixedHarness(pnp_fixed);
    pnpFixedHarness.run();
    //harness_singleFuncFixedRows.run(xprime50, x50);
    // Example value; in a real scenario, this would be determined at runtime
    // Create blocks with the actual number of rows to use

    // Array of PnP functions with dynamic rows
    //
    auto pnp_runtime = [&xprime_runtime, &x_runtime]() -> Eigen::Matrix<Scalar, 4, 4>
    {
      return example_pnp_function_dynamic<Scalar, maxN, storageOrder>(xprime_runtime, x_runtime);
    };

    BenchmarkHarness pnpRuntimeHarness(pnp_runtime);
    pnpRuntimeHarness.run();
    //auto functions = std::array{example_pnp_function_dynamic<Scalar, maxN, storageOrder>,
    //                            example_pnp_function_dynamic<Scalar, maxN, storageOrder>};
    //auto harness_multipleFuncsRuntimeRows = PnPHarness(functions);
    //harness_multipleFuncsRuntimeRows.run(xprime_runtime, x_runtime);
    return 0;
}
