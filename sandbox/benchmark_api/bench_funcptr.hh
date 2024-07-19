#include <array>
#include <utility>
#include <type_traits>

//#define EIGEN_NO_MALLOC
//#define EIGEN_RUNTIME_NO_MALLOC
//#define EIGEN_NO_DEBUG
#include <Eigen/Dense>

template <typename Func>
concept NoInputNoOutputFunction = requires(Func f) {
  { f() } -> std::same_as<void>;
};

template <typename Func, size_t NumFuncs>
class BenchmarkHarness {
public:
  // Constructor for a single function
  BenchmarkHarness(Func func) : functions_{func}, num_functions_(1) {}

  // Constructor for an array of functions
  BenchmarkHarness(const std::array<Func, NumFuncs>& functions) : functions_(functions), num_functions_(NumFuncs) {}

  template <typename... Args>
  inline void run(Args&&... args) {
    for (size_t i = 0; i < this->num_functions_; ++i)
    {
      //start_roi();
      functions_[i](std::forward<Args>(args)...);
      //end_roi();
    }
  }

protected:
  std::array<Func, NumFuncs> functions_;
  size_t num_functions_;
};


// Type traits to extract necessary types from function type
template <typename Func>
struct PnPFunctionTraits;

// Specialization for function pointers
template <typename Scalar, int N, int StorageOrder, int MaxN>
struct PnPFunctionTraits<Eigen::Matrix<Scalar, 4, 4> (*)(const Eigen::Matrix<Scalar, N, 2, StorageOrder, MaxN, 2>&, const Eigen::Matrix<Scalar, N, 3, StorageOrder, MaxN, 3>&)> {
    using Scalar_ = Scalar;
    static constexpr int N_ = N;
    static constexpr int StorageOrder_ = StorageOrder;
    static constexpr int MaxN_ = MaxN;
};

template <typename Func, typename Scalar, int N, int StorageOrder, int MaxN>
concept PnPFunction =
  requires(const Eigen::Matrix<Scalar, N, 2, StorageOrder, MaxN, 2>& xprime,
           const Eigen::Matrix<Scalar, N, 3, StorageOrder, MaxN, 3>& x) {
    { std::declval<Func>()(xprime, x) } -> std::same_as<Eigen::Matrix<Scalar, 4, 4>>;
};


template<int StorageOrder>
concept ValidStorageOrder = (StorageOrder == Eigen::RowMajor || StorageOrder == Eigen::ColMajor);

template <typename Func,
          size_t NumFuncs,
          typename Scalar,
          int N,
          int StorageOrder,
          int MaxN>
requires PnPFunction<Func, Scalar, N, StorageOrder, MaxN> && ValidStorageOrder<StorageOrder>
class PnPHarness :
  public BenchmarkHarness<Func,
                          NumFuncs> {
public:
  using Base = BenchmarkHarness<Func, NumFuncs>;
  using Base::Base;


  /*void run_impl(const Eigen::Matrix<Scalar, N, 2, StorageOrder, MaxN, 2>& xprime,
                const Eigen::Matrix<Scalar, N, 3, StorageOrder, MaxN, 3>& x) {
      for (size_t i = 0; i < this->num_functions_; ++i) {
          this->functions_[i](xprime, x);
      }
  }*/
  void run_impl(const auto& xprime, const auto& x)
  {
    for (size_t i = 0; i < this->num_functions_; ++i)
    {
      this->functions_[i](xprime, x);
    }
  }


  /*void run_impl(const Eigen::Matrix<Scalar, Eigen::Dynamic, 2,
                                    StorageOrder, MaxN, 2>& xprime,
                const Eigen::Matrix<Scalar, Eigen::Dynamic, 3,
                                    StorageOrder, MaxN, 3>& x)
  {
      for (size_t i = 0; i < this->num_functions_; ++i) {
          this->functions_[i](xprime, x);
      }
  }*/
};

// Template deduction guide for  PnP Harness given a Func
template <typename Func>
PnPHarness(Func) -> PnPHarness<
    Func,
    1,
    typename PnPFunctionTraits<Func>::Scalar_,
    PnPFunctionTraits<Func>::N_,
    PnPFunctionTraits<Func>::StorageOrder_,
    PnPFunctionTraits<Func>::MaxN_
>;

/*template <typename Func, size_t NumFuncs>
PnPHarness(const std::array<Func, NumFuncs>&) -> PnPHarness<
    typename PnPFunctionTraits<Func>::Scalar_,
    Func,
    PnPFunctionTraits<Func>::N_,
    PnPFunctionTraits<Func>::StorageOrder_,
    PnPFunctionTraits<Func>::MaxN_,
    NumFuncs
>*/

template <typename Func, size_t NumFuncs>
PnPHarness(const std::array<Func, NumFuncs>&) -> PnPHarness<
    Func,
    NumFuncs,
    typename PnPFunctionTraits<typename std::decay<Func>::type>::Scalar_,
    PnPFunctionTraits<typename std::decay<Func>::type>::N_,
    PnPFunctionTraits<typename std::decay<Func>::type>::StorageOrder_,
    PnPFunctionTraits<typename std::decay<Func>::type>::MaxN_
>;

template <NoInputNoOutputFunction Func, size_t NumFuncs=1>
class AssemblyHarness : public BenchmarkHarness<Func, NumFuncs> {
public:
  using Base = BenchmarkHarness<Func, NumFuncs>;
  // Inherit constructors from the base class with proper template
  using Base::BenchmarkHarness;

  inline void run_impl() const __attribute__((always_inline)){
    for (size_t i = 0; i < this->num_functions_; ++i) {
      this->functions_[i]();
    }
  }
};

template <NoInputNoOutputFunction Func>
inline auto make_no_io_lambda(Func func)
{
  return [func]()
  {
    func();
  };
}



template <NoInputNoOutputFunction... Funcs>
inline auto make_no_io_lambda(Funcs... funcs)
{
  return std::array{make_no_io_lambda(funcs)...};
}

template <NoInputNoOutputFunction Func>
auto make_assembly_harness(Func func) {
  return AssemblyHarness<Func, 1>(std::array<Func, 1>{func});
}

template <NoInputNoOutputFunction Func, size_t NumFuncs>
auto make_assembly_harness(const std::array<Func, NumFuncs>& functions)
{
  return AssemblyHarness<Func, NumFuncs>(functions);
}

/*template <typename Scalar, int N, int MaxN, int StorageOrder, typename Func, size_t NumFuncs>
requires PnPFunction<Func, Scalar, N, StorageOrder, MaxN> && ValidStorageOrder<StorageOrder>
auto make_pnp_harness(const std::array<Func, NumFuncs>& functions)
{
  return PnPHarness<Scalar, Func, N, StorageOrder, N, NumFuncs>(functions);
}*/
