#ifndef TEST_PROBLEM_H
#define TEST_PROBLEM_H

#include <sstream>
#ifdef NATIVE
#include <string>
#endif

#include <ento-util/containers.h>
#include <ento-bench/problem.h>


namespace EntoBench
{

template <typename Callable, typename T, size_t M = 0, size_t N = 0>
class TestProblem : public EntoProblem<TestProblem<Callable, T, M, N>>
{
private:
  Callable callable_;
  EntoUtil::EntoContainer<T, M> a_;
  EntoUtil::EntoContainer<T, M> b_;
  EntoUtil::EntoContainer<T, N> c_;
  EntoUtil::EntoContainer<T, N> c_gt_;

  size_t m_ = M;
  size_t n_ = N;
public:
  static constexpr bool RequiresDataset_ = true;
  static constexpr bool SaveResults_ = true;

#ifdef NATIVE
  std::string serialize_impl() const
  {
    std::ostringstream oss;
    int n = c_.size();
    for (int i = 0; i < n; ++i)
    {
      if ( i < n - 1)
        oss << c_[i] << ",";
      else
        oss << c_[i];
    }
    return oss.str();
  }

  bool deserialize_impl(const std::string &line)
  {
    std::istringstream iss(line);
    
    size_t m, n;
    char comma;
    if ( !( iss >> m >> comma ) || ( comma != ',' ) )
    {
      return false;
    }

    if ( !( iss >> n >> comma ) || ( comma != ',' ) )
    {
      return false;
    }

    
    if (M == 0)
    {
      m_ = m;
      a_.resize(m);
      b_.resize(m);
    }
    else if (m_ != m) return false;
    
    if (N == 0)
    {
      n_ = n;
      c_.resize(n);
      c_gt_.resize(n);
    }
    else if (n_ != n) return false;

  

    //a_.clear();
    //b_.clear();
    //c_gt_.clear();
    //c_.clear();

    int ai, bi, ci_gt;
    for (int i = 0; i < m_; ++i)
    {
      if ( !( iss >> ai >> comma ) || ( comma != ',' ) ) return false; 
      a_.push_back(ai);
    }

    for (int i = 0; i < m_; ++i)
    {
      if ( !( iss >> bi  >> comma ) || ( comma != ',' ) ) return false; 
      b_.push_back(bi);
    }

    for (int i = 0; i < n_; ++i)
    {
      if (i != n_ - 1)
      {
        if ( !( iss >> ci_gt >> comma ) || ( comma != ',' ) )
          return false; 
      }
      else
      {
        if ( !( iss >> ci_gt ) ) return false;
      }
      c_gt_.push_back(ci_gt);
    }

    return true;
  }
#endif
  bool deserialize_impl(const char* line)
  {
    
  }
  void solve_impl()
  {
    callable_(a_, b_, c_);
  }

  bool validate_impl()
  {
    size_t n = c_.size();
    for (size_t i = 0; i < n; ++i)
    {
      if (c_[i] != c_gt_[i]) return false;
    }
    return true;
  }

  void clear_impl()
  {
    a_.clear();
    b_.clear();
    c_.clear();
    c_gt_.clear();
  }

  static constexpr const char* header_impl() { return "M, N, Ai, Bi, Cj_gt"; }

  TestProblem(Callable callable) : callable_(std::move(callable)) {};
};

} // namespace TEST_PROBLEM_H

#endif // TEST_PROBLEM_H
