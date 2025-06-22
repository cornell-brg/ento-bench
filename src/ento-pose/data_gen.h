#ifndef DATA_GEN_H
#define DATA_GEN_H

#include <ento-pose/pose_util.h>
#include <ento-pose/prob_gen.h>
#include <ento-pose/abs-pose/dlt.h>
#include <ento-pose/abs-pose/p3p.h>
#include <ento-pose/abs-pose/up2p.h>
#include <ento-pose/abs-pose/up2p.h>
#include <ento-pose/abs-pose/p4p.h>
#include <ento-pose/rel-pose/eight_pt.h>
#include <ento-pose/rel-pose/five_pt_nister.h>
#include <ento-pose/rel-pose/upright_planar_two_pt.h>
#include <ento-pose/rel-pose/upright_three_pt.h>
#include <ento-pose/rel-pose/upright_planar_three_pt.h>

#include <ento-pose/abs-pose/gold_standard.h>
#include <ento-pose/rel-pose/gold_standard.h>

#include <ento-pose/problem-types/homography_problem.h>
#include <ento-pose/problem-types/absolute_pose_problem.h>
#include <ento-pose/problem-types/relative_pose_problem.h>

namespace EntoPose
{

#ifdef NATIVE
template<typename Scalar>
struct BenchmarkResult {
    std::string name_;
    ProblemOptions<Scalar> options_;
    int instances_ = 0;
    int solutions_ = 0;
    int valid_solutions_ = 0;
    int found_gt_pose_ = 0;
    int runtime_ns_ = 0;
};
#endif

// Wrappers for the Benchmarking code

template <typename Scalar>
struct SolverP3P {
  using scalar_type = Scalar;
  static constexpr size_t MaxSolns = 4;
  static constexpr size_t MinSampleSize = 3;

  static inline int solve(AbsolutePoseProblem<Scalar, SolverP3P<Scalar>, 0>& instance,
                          std::vector<CameraPose<Scalar>>* solutions)
  {
    return p3p(instance.x_point_, instance.X_point_, solutions);
  }

  static inline int solve(AbsolutePoseProblem<Scalar, SolverP3P<Scalar>, 3>& instance,
                          EntoUtil::EntoArray<CameraPose<Scalar>, MaxSolns>* solutions)
  {
    return p3p(instance.x_point_, instance.X_point_, solutions);
  }
  
  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x,
                          EntoContainer<Vec3<Scalar>, N>& X,
                          std::vector<CameraPose<Scalar>>* solutions)
  {
    return p3p(x, X, solutions);
  }

  // Add missing EntoArray overload for template version
  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x,
                          EntoContainer<Vec3<Scalar>, N>& X,
                          EntoUtil::EntoArray<CameraPose<Scalar>, MaxSolns>* solutions)
  {
    return p3p(x, X, solutions);
  }

  typedef CalibratedAbsolutePoseValidator<Scalar> validator;
  static std::string name() { return "p3p"; }
};

template <typename Scalar>
struct SolverUP2P {
  using scalar_type = Scalar;
  static constexpr size_t MaxSolns = 4;
  static constexpr size_t MinSampleSize = 2;

  static inline int solve(const AbsolutePoseProblem<Scalar, SolverUP2P<Scalar>, 0>& instance,
                          std::vector<CameraPose<Scalar>>* solutions)
  {
    return up2p(instance.x_point_, instance.X_point_, solutions);
  }

  static inline int solve(AbsolutePoseProblem<Scalar, SolverUP2P<Scalar>, 2>& instance,
                          EntoUtil::EntoArray<CameraPose<Scalar>, MaxSolns>* solutions)
  {
    return up2p(instance.x_point_, instance.X_point_, solutions);
  }

  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x,
                          EntoContainer<Vec3<Scalar>, N>& X,
                          std::vector<CameraPose<Scalar>>* solutions)
  {
    return up2p(x, X, solutions);
  }

  // Add missing EntoArray overload for template version
  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x,
                          EntoContainer<Vec3<Scalar>, N>& X,
                          EntoUtil::EntoArray<CameraPose<Scalar>, MaxSolns>* solutions)
  {
    return up2p(x, X, solutions);
  }

  typedef CalibratedAbsolutePoseValidator<Scalar> validator;
  static std::string name() { return "up2p"; }
};

template <typename Scalar>
struct SolverRelUpright3pt {
  using scalar_type = Scalar;
  static constexpr size_t MaxSolns = 4;
  static constexpr size_t MinSampleSize = 3;

  static inline int solve(const RelativePoseProblem<Scalar, SolverRelUpright3pt<Scalar>, 0>& instance,
                          std::vector<CameraPose<Scalar>>* solutions)
  {
    return relpose_upright_3pt<Scalar, 0>(instance.x1_, instance.x2_, solutions);
  }

  static inline int solve(RelativePoseProblem<Scalar, SolverRelUpright3pt<Scalar>, 3>& instance,
                          EntoUtil::EntoArray<CameraPose<Scalar>, MaxSolns>* solutions)
  {
    return relpose_upright_3pt<Scalar, 3>(instance.x1_, instance.x2_, solutions);
  }

  // Add missing EntoArray overload for template version
  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x,
                          EntoContainer<Vec3<Scalar>, N>& X,
                          EntoUtil::EntoArray<CameraPose<Scalar>, MaxSolns>* solutions)
  {
    return relpose_upright_3pt<Scalar, N>(x, X, solutions);
  }

  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x,
                          EntoContainer<Vec3<Scalar>, N>& X,
                          std::vector<CameraPose<Scalar>>* solutions)
  {
    return relpose_upright_3pt<Scalar, N>(x, X, solutions);
  }

  typedef CalibratedAbsolutePoseValidator<Scalar> validator;
  typedef CameraPose<Scalar> Solution;
  static std::string name() { return "RelUpright3pt"; }
};


template <typename Scalar>
struct SolverRel8pt
{
  using scalar_type = Scalar;
  static constexpr size_t MaxSolns = 4;
  static constexpr size_t MinSampleSize = 8;    // Original minimal sample size
  // static constexpr size_t MinSampleSize = 12;    // Test overdetermined system
  // static constexpr size_t MinSampleSize = 16;    // Test highly overdetermined system

  static inline int solve(const RelativePoseProblem<Scalar, SolverRel8pt<Scalar>, 0>& instance,
                          std::vector<CameraPose<Scalar>> *solutions) {
    EntoArray<CameraPose<Scalar>, 4> solutions_;
    int sols = relpose_8pt<Scalar, 0>(instance.x1_, instance.x2_, &solutions_);
    for (size_t i = 0; i < sols; i++)
    {
      solutions->emplace_back(solutions_[i]);
    }
    return sols;
  }

  static inline int solve(RelativePoseProblem<Scalar, SolverRel8pt<Scalar>, 8>& instance,
                          EntoArray<CameraPose<Scalar>, MaxSolns> *solutions) {
    //EntoArray<CameraPose<Scalar>, 4> solutions_;
    int sols = relpose_8pt<Scalar, 8>(instance.x1_, instance.x2_, solutions);
    return sols;
  }

  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x1,
                          EntoContainer<Vec3<Scalar>, N>& x2,
                          std::vector<CameraPose<Scalar>>* solutions)
  {
    EntoArray<CameraPose<Scalar>, 4> solutions_;
    size_t sols = relpose_8pt<Scalar, N>(x1, x2, &solutions_);
    for (size_t i = 0; i < sols; i++)
    {
      solutions->emplace_back(solutions_[i]);
    }
    return sols;

  }
  
  // Add missing EntoArray overload for template version
  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x1,
                          EntoContainer<Vec3<Scalar>, N>& x2,
                          EntoUtil::EntoArray<CameraPose<Scalar>, MaxSolns>* solutions)
  {
    return relpose_8pt<Scalar, N>(x1, x2, solutions);
  }

  typedef CalibratedRelativePoseValidator<Scalar> validator;
  typedef CameraPose<Scalar> Solution;
  static std::string name() { return "Rel8pt"; }
};

template <typename Scalar>
struct SolverRel5pt
{
  using scalar_type = Scalar;
  static constexpr size_t MaxSolns = 40;
  static constexpr size_t MinSampleSize = 5;

  static inline int solve(const RelativePoseProblem<Scalar, SolverRel5pt, 0>& instance,
                          std::vector<CameraPose<Scalar>> *solutions)
  {
    EntoArray<CameraPose<Scalar>, 40> solutions_;
    int sols = relpose_5pt<Scalar>(instance.x1_, instance.x2_, &solutions_);
    for (size_t i = 0; i < sols; i++)
    {
      solutions->emplace_back(solutions_[i]);
    }
    return sols;
  }

  static inline int solve(RelativePoseProblem<Scalar, SolverRel5pt, 5>& instance,
                          EntoArray<CameraPose<Scalar>, MaxSolns> *solutions)
  {
    int sols = relpose_5pt<Scalar>(instance.x1_, instance.x2_, solutions);
    return sols;
  }

  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x1,
                          EntoContainer<Vec3<Scalar>, N>& x2,
                          std::vector<CameraPose<Scalar>>* solutions)
  {
    EntoArray<CameraPose<Scalar>, 40> solutions_;
    size_t sols = relpose_5pt<Scalar>(x1, x2, &solutions_);
    for (size_t i = 0; i < sols; i++)
    {
      solutions->emplace_back(solutions_[i]);
    }
    return sols;
  }
  
  // Add missing EntoArray overload for template version
  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x1,
                          EntoContainer<Vec3<Scalar>, N>& x2,
                          EntoUtil::EntoArray<CameraPose<Scalar>, MaxSolns>* solutions)
  {
    return relpose_5pt<Scalar>(x1, x2, solutions);
  }

  typedef CalibratedRelativePoseValidator<Scalar> validator;
  typedef CameraPose<Scalar> Solution;
  static std::string name() { return "Rel5pt"; }
};

template <typename Scalar>
struct SolverRelUprightPlanar2pt 
{
  using scalar_type = Scalar;
  static constexpr size_t MaxSolns = 4;
  static constexpr size_t MinSampleSize = 2;

  static inline int solve(const RelativePoseProblem<Scalar, SolverRelUprightPlanar2pt<Scalar>, 0>& instance,
                          std::vector<CameraPose<Scalar>> *solutions)
  {
    EntoArray<CameraPose<Scalar>, 4> solutions_;
    int sols = relpose_upright_planar_2pt<Scalar>(instance.x1_, instance.x2_, &solutions_);
    for (int i = 0; i < sols; i++)
    {
      solutions->emplace_back(solutions_[i]);
    }
    return sols;
  }

  static inline int solve(RelativePoseProblem<Scalar, SolverRelUprightPlanar2pt<Scalar>, 2>& instance,
                          EntoArray<CameraPose<Scalar>, MaxSolns> *solutions)
  {
    int sols = relpose_upright_planar_2pt<Scalar, 2>(instance.x1_, instance.x2_, solutions);
    return sols;
  }

  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x1,
                          EntoContainer<Vec3<Scalar>, N>& x2,
                          std::vector<CameraPose<Scalar>>* solutions)
  {
    EntoArray<CameraPose<Scalar>, 4> solutions_;
    size_t sols = relpose_upright_planar_2pt<Scalar, N>(x1, x2, &solutions_);
    for (size_t i = 0; i < sols; i++)
    {
      solutions->emplace_back(solutions_[i]);
    }
    return sols;
  }
  
  // Add missing EntoArray overload for template version
  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x1,
                          EntoContainer<Vec3<Scalar>, N>& x2,
                          EntoUtil::EntoArray<CameraPose<Scalar>, MaxSolns>* solutions)
  {
    return relpose_upright_planar_2pt<Scalar, N>(x1, x2, solutions);
  }
  typedef CalibratedRelativePoseValidator<Scalar> validator;
  typedef CameraPose<Scalar> Solution;
  static std::string name() { return "RelUprightPlanar2pt"; }
};

template <typename Scalar>
struct SolverRelUprightPlanar3pt
{
  using scalar_type = Scalar;
  static constexpr size_t MaxSolns = 2;
  static constexpr size_t MinSampleSize = 3;

  static inline int solve(const RelativePoseProblem<Scalar, SolverRelUprightPlanar3pt<Scalar>, 0>& instance,
                          std::vector<CameraPose<Scalar>> *solutions)
  {
    EntoArray<CameraPose<Scalar>, 2> solutions_;
    int sols = relpose_upright_planar_3pt<Scalar, 0>(instance.x1_, instance.x2_, &solutions_);
    for (int i = 0; i < sols; i++)
    {
      solutions->emplace_back(solutions_[i]);
    }
    return sols;
  }

  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x1,
                          EntoContainer<Vec3<Scalar>, N>& x2,
                          std::vector<CameraPose<Scalar>>* solutions)
  {
    ENTO_DEBUG("[SOLVER] SolverRelUprightPlanar3pt::solve called with %zu points", x1.size());
    
    EntoArray<CameraPose<Scalar>, 2> solutions_;
    size_t sols = relpose_upright_planar_3pt<Scalar, N>(x1, x2, &solutions_);
    
    ENTO_DEBUG("[SOLVER] SolverRelUprightPlanar3pt::solve returned %zu solutions", sols);
    for (size_t i = 0; i < sols; i++)
    {
      solutions->emplace_back(solutions_[i]);
      ENTO_DEBUG("[SOLVER] Solution %zu: t=(%f,%f,%f)", i,
                 solutions_[i].t(0), solutions_[i].t(1), solutions_[i].t(2));
    }
    return sols;
  }
  
  // Add missing EntoArray overload for template version
  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x1,
                          EntoContainer<Vec3<Scalar>, N>& x2,
                          EntoUtil::EntoArray<CameraPose<Scalar>, MaxSolns>* solutions)
  {
    ENTO_DEBUG("[SOLVER] SolverRelUprightPlanar3pt::solve (EntoArray) called with %zu points", x1.size());
    
    int sols = relpose_upright_planar_3pt<Scalar, N>(x1, x2, solutions);
    
    ENTO_DEBUG("[SOLVER] SolverRelUprightPlanar3pt::solve (EntoArray) returned %d solutions", sols);
    for (int i = 0; i < sols; i++)
    {
      ENTO_DEBUG("[SOLVER] Solution %d: t=(%f,%f,%f)", i,
                 (*solutions)[i].t(0), (*solutions)[i].t(1), (*solutions)[i].t(2));
    }
    return sols;
  }

  typedef CalibratedRelativePoseValidator<Scalar> validator;
  typedef CameraPose<Scalar> Solution;
  static std::string name() { return "RelUprightPlanar3pt"; }
};

template <typename Scalar, bool CheiralCheck = false, int Method=0, int SVDMethod=0>
struct SolverHomography4pt
{
  using scalar_type = Scalar;
  static constexpr size_t MaxSolns = 1;
  static constexpr size_t MinSampleSize = 4;

  static inline int solve(const RelativePoseProblem<Scalar, SolverHomography4pt<Scalar>, 0>& instance,
                          std::vector<Matrix3x3<Scalar>> *solutions) {
    Matrix3x3<Scalar> H;
    int sols = homography_4pt<Scalar, CheiralCheck, Method>(instance.x1_, instance.x2_, &H);
    solutions->clear();
    if (sols == 1)
    {
      solutions->push_back(H);
    }
    return sols;
  }

  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x1,
                          EntoContainer<Vec3<Scalar>, N>& x2,
                          std::vector<Matrix3x3<Scalar>>* solutions)
  {
    Matrix3x3<Scalar> H;
    size_t sols = homography_4pt<Scalar, CheiralCheck, Method>(x1, x2, &H);
    solutions->clear();
    if (sols == 1) solutions->push_back(H);
    return sols;
  }

  // Add missing EntoArray overload for template version
  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x1,
                          EntoContainer<Vec3<Scalar>, N>& x2,
                          EntoUtil::EntoArray<Matrix3x3<Scalar>, MaxSolns>* solutions)
  {
    Matrix3x3<Scalar> H;
    
    static_assert((N == 0) || (N == 4));
    
    size_t sols = homography_4pt<Scalar, CheiralCheck, Method, SVDMethod>(x1, x2, &H);
    solutions->clear();
    if (sols == 1) {
      solutions->push_back(H);
    }
    return sols;
  }

  typedef HomographyValidator<Scalar> validator;

  static std::string name()
  {
    if constexpr (CheiralCheck)
    {
      return "Homography4pt(C)";
    }
    else
    {
      return "Homography4pt";
    }
  }
};

template <typename Scalar, bool CheiralCheck = false, int Method=0>
struct SolverHomography4ptDLT {
  using scalar_type = Scalar;
  static constexpr size_t MaxSolns = 1;
  static constexpr size_t MinSampleSize = 4;

  static inline int solve(const RelativePoseProblem<Scalar, SolverHomography4ptDLT<Scalar>, 0>& instance,
                          std::vector<Matrix3x3<Scalar>>* solutions) {
    Matrix3x3<Scalar> H;
    size_t sols = homography_Npt<Scalar, 0, CheiralCheck, 0, 0>(instance.x1_, instance.x2_, &H);
    solutions->clear();
    if (sols == 1)
    {
      solutions->push_back(H);
    }
    return sols;
  }

  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x1,
                          EntoContainer<Vec3<Scalar>, N>& x2,
                          std::vector<Matrix3x3<Scalar>>* solutions)
  {
    Matrix3x3<Scalar> H;
    size_t sols = homography_Npt<Scalar, 0, CheiralCheck, 0, 0>(x1, x2, &H);
    solutions->clear();
    if (sols == 1) solutions->push_back(H);
    return sols;
  }

  // Add missing EntoArray overload for template version
  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x1,
                          EntoContainer<Vec3<Scalar>, N>& x2,
                          EntoUtil::EntoArray<Matrix3x3<Scalar>, MaxSolns>* solutions)
  {
    Matrix3x3<Scalar> H;
    size_t sols = homography_Npt<Scalar, 0, CheiralCheck, 0, 0>(x1, x2, &H);
    solutions->clear();
    if (sols == 1) {
      solutions->push_back(H);
    }
    return sols;
  }

  typedef HomographyValidator<Scalar> validator;
  static std::string name()
  {
    if constexpr (CheiralCheck)
    {
      return "Homography4pt(C)";
    }
    else
    {
      return "Homography4pt";
    }
  }
};

template <typename Scalar, bool CheiralCheck = false, int Method=0>
struct SolverHomographyNptDLT {
  using scalar_type = Scalar;
  static constexpr size_t MaxSolns = 1;
  // Note: NptDLT is variable, but let's say minimum 4 for now
  static constexpr size_t MinSampleSize = 4;

  static inline int solve(const RelativePoseProblem<Scalar, SolverHomographyNptDLT<Scalar>, 0>& instance,
                          std::vector<Matrix3x3<Scalar>>* solutions) {
    Matrix3x3<Scalar> H;
    int sols = homography_Npt<Scalar, 0, CheiralCheck, 0, 0>(instance.x1_, instance.x2_, &H);
    solutions->clear();
    if (sols == 1)
    {
      solutions->push_back(H);
    }
    return sols;
  }

  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x1,
                          EntoContainer<Vec3<Scalar>, N>& x2,
                          std::vector<Matrix3x3<Scalar>>* solutions)
  {
    Matrix3x3<Scalar> H;
    size_t sols = homography_Npt<Scalar, 0, CheiralCheck, 0, 0>(x1, x2, &H);
    solutions->clear();
    if (sols == 1) solutions->push_back(H);
    return sols;
  }

  // Add missing EntoArray overload for template version
  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x1,
                          EntoContainer<Vec3<Scalar>, N>& x2,
                          EntoUtil::EntoArray<Matrix3x3<Scalar>, MaxSolns>* solutions)
  {
    Matrix3x3<Scalar> H;
    size_t sols = homography_Npt<Scalar, 0, CheiralCheck, 0, 0>(x1, x2, &H);
    solutions->clear();
    if (sols == 1) {
      solutions->push_back(H);
    }
    return sols;
  }

  typedef HomographyValidator<Scalar> validator;

  static std::string name()
  {
    if (CheiralCheck)
    {
      return "Homography4pt(C)";
    }
    else
    {
      return "Homography4pt";
    }
  }
};

// Solver for DLT (general absolute pose, non-planar)
template <typename Scalar>
struct SolverDLT {
  using scalar_type = Scalar;
  static constexpr size_t MaxSolns = 1;
  static constexpr size_t MinSampleSize = 6;

  static inline int solve(AbsolutePoseProblem<Scalar, SolverDLT<Scalar>, 0>& instance,
                         std::vector<CameraPose<Scalar>>* solutions)
  {
    return dlt(instance.x_point_, instance.X_point_, solutions);
  }

  static inline int solve(AbsolutePoseProblem<Scalar, SolverDLT<Scalar>, 6>& instance,
                         EntoUtil::EntoArray<CameraPose<Scalar>, MaxSolns>* solutions)
  {
    return dlt(instance.x_point_, instance.X_point_, solutions);
  }

  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x,
                         EntoContainer<Vec3<Scalar>, N>& X,
                         std::vector<CameraPose<Scalar>>* solutions)
  {
    return dlt(x, X, solutions);
  }

  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x,
                         EntoContainer<Vec3<Scalar>, N>& X,
                         EntoUtil::EntoArray<CameraPose<Scalar>, MaxSolns>* solutions)
  {
    return dlt<Scalar, N>(x, X, solutions);
  }

  typedef CalibratedAbsolutePoseValidator<Scalar> validator;
  static std::string name() { return "dlt"; }
};

// Gold Standard Absolute Pose Solver (DLT + Bundle Adjustment)
template <typename Scalar>
struct SolverGoldStandardAbs {
  using scalar_type = Scalar;
  static constexpr size_t MaxSolns = 1;
  static constexpr size_t MinSampleSize = 6;

  static inline int solve(AbsolutePoseProblem<Scalar, SolverGoldStandardAbs<Scalar>, 0>& instance,
                         std::vector<CameraPose<Scalar>>* solutions)
  {
    return gold_standard_abs(instance.x_point_, instance.X_point_, solutions);
  }

  static inline int solve(AbsolutePoseProblem<Scalar, SolverGoldStandardAbs<Scalar>, 6>& instance,
                         EntoUtil::EntoArray<CameraPose<Scalar>, MaxSolns>* solutions)
  {
    return gold_standard_abs(instance.x_point_, instance.X_point_, solutions);
  }

  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x,
                         EntoContainer<Vec3<Scalar>, N>& X,
                         std::vector<CameraPose<Scalar>>* solutions)
  {
    return gold_standard_abs(x, X, solutions);
  }

  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x,
                         EntoContainer<Vec3<Scalar>, N>& X,
                         EntoUtil::EntoArray<CameraPose<Scalar>, MaxSolns>* solutions)
  {
    return gold_standard_abs<Scalar, N>(x, X, solutions);
  }

  typedef CalibratedAbsolutePoseValidator<Scalar> validator;
  static std::string name() { return "GoldStandardAbs"; }
};

// Gold Standard Relative Pose Solver (8pt + Sampson Refinement)
template <typename Scalar>
struct SolverGoldStandardRel {
  using scalar_type = Scalar;
  static constexpr size_t MaxSolns = 1;
  static constexpr size_t MinSampleSize = 8;

  static inline int solve(const RelativePoseProblem<Scalar, SolverGoldStandardRel<Scalar>, 0>& instance,
                          std::vector<CameraPose<Scalar>>* solutions)
  {
    return gold_standard_rel(instance.x1_, instance.x2_, solutions);
  }

  static inline int solve(RelativePoseProblem<Scalar, SolverGoldStandardRel<Scalar>, 8>& instance,
                          EntoUtil::EntoArray<CameraPose<Scalar>, MaxSolns>* solutions)
  {
    return gold_standard_rel(instance.x1_, instance.x2_, solutions);
  }

  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x1,
                          EntoContainer<Vec3<Scalar>, N>& x2,
                          std::vector<CameraPose<Scalar>>* solutions)
  {
    return gold_standard_rel(x1, x2, solutions);
  }

  template <size_t N = 0>
  static inline int solve(EntoContainer<Vec3<Scalar>, N>& x1,
                          EntoContainer<Vec3<Scalar>, N>& x2,
                          EntoUtil::EntoArray<CameraPose<Scalar>, MaxSolns>* solutions)
  {
    return gold_standard_rel<Scalar, N>(x1, x2, solutions);
  }

  typedef CalibratedRelativePoseValidator<Scalar> validator;
  static std::string name() { return "GoldStandardRel"; }
};

} // namespace EntoPose

#endif // DATA_GEN_H
