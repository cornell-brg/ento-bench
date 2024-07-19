#include <variant>
#include <iostream>
#include <utility>
#include <array>

// Helper type traits to detect function pointers
template<typename T>
struct is_function_pointer : std::integral_constant<
    bool,
    std::is_pointer<T>::value && std::is_function<typename std::remove_pointer<T>::type>::value
> {};

// Helper to wrap function pointers in lambdas
template <typename F>
auto wrap_callable(F func) {
    if constexpr (is_function_pointer<F>::value) {
        return [func](auto&&... args) { return func(std::forward<decltype(args)>(args)...); };
    } else {
        return func;
    }
}

// BenchmarkHarness class
template <typename... Funcs>
class BenchmarkHarness {
public:
    using CallableType = std::function<void()>;

    // Constructor for a single function pointer
    template <typename F>
    requires std::is_pointer_v<F>
    BenchmarkHarness(F func) : functions_{wrap_callable(func)} {}

    // Constructor for multiple function pointers
    template <typename F, typename... Fs>
    requires ((std::is_same_v<F, Fs> && ...) && std::is_pointer_v<F>)
    BenchmarkHarness(F first, Fs... funcs) : functions_{wrap_callables(first, funcs...)} {}

    // Constructor for a single lambda
    BenchmarkHarness(CallableType func) : functions_{func} {}

    void run() {
        for (const auto& func : functions_) {
            func();
        }
    }

private:
    std::array<CallableType, sizeof...(Funcs)> functions_;
    size_t num_functions_ = sizeof...(Funcs);

    template <typename F>
    auto wrap_callable(F func) {
        return [func](auto&&... args) { func(std::forward<decltype(args)>(args)...); };
    }

    template <typename F, typename... Fs>
    auto wrap_callables(F first, Fs... funcs) {
        return std::array<CallableType, sizeof...(Funcs)>{wrap_callable(first), wrap_callable(funcs)...};
    }
};
// Template argument deduction guides
template <typename... Fs>
BenchmarkHarness(Fs...) -> BenchmarkHarness<Fs...>;

template <typename F, size_t N>
BenchmarkHarness(std::array<F, N>) -> BenchmarkHarness<F>;

template <typename... Fs>
BenchmarkHarness(std::initializer_list<std::variant<Fs...>>) -> BenchmarkHarness<Fs...>;
