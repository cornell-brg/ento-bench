#ifndef DATA_GEN_H
#define DATA_GEN_H

#include <ento-pose/pose_util.h>
#include <ento-pose/prob_gen.h>
#include <ento-pose/abs-pose/p3p.h>
#include <ento-pose/abs-pose/up2p.h>
#include <ento-pose/abs-pose/up2p.h>
#include <ento-pose/abs-pose/p4p.h>
#include <ento-pose/rel-pose/eight_pt.h>
#include <ento-pose/rel-pose/five_pt_nister.h>
#include <ento-pose/rel-pose/upright_planar_two_pt.h>
#include <ento-pose/rel-pose/upright_three_pt.h>

namespace EntoPose
{

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

// Wrappers for the Benchmarking code

template <typename Scalar>
struct SolverP3P {
  static inline int solve(const AbsolutePoseProblemInstance<Scalar> &instance,
                          std::vector<CameraPose<Scalar>> *solutions)
  {
    return p3p(instance.x_point_, instance.X_point_, solutions);
  }
  typedef CalibPoseValidator<Scalar> validator;
  static std::string name() { return "p3p"; }
};

template <typename Scalar>
struct SolverUP2P {
  static inline int solve(const AbsolutePoseProblemInstance<Scalar> &instance, std::vector<CameraPose<Scalar>> *solutions)
  {
    return up2p(instance.x_point_, instance.X_point_, solutions);
  }
  typedef CalibPoseValidator<Scalar> validator;
  static std::string name() { return "up2p"; }
};

template <typename Scalar>
struct SolverRelUpright3pt {
  static inline int solve(const RelativePoseProblemInstance<Scalar> &instance, std::vector<CameraPose<Scalar>> *solutions)
  {
    return relpose_upright_3pt(instance.x1_, instance.x2_, solutions);
  }
  typedef CalibPoseValidator<Scalar> validator;
  typedef CameraPose<Scalar> Solution;
  static std::string name() { return "RelUpright3pt"; }
};


template <typename Scalar>
struct SolverRel8pt {
    static inline int solve(const RelativePoseProblemInstance<Scalar> &instance, std::vector<CameraPose<Scalar>> *solutions) {
      EntoArray<CameraPose<Scalar>, 4> solutions_;
      int sols = relpose_8pt<Scalar, 0>(instance.x1_, instance.x2_, &solutions_);
      for (size_t i = 0; i < sols; i++)
      {
        solutions->emplace_back(solutions_[i]);
      }
      return sols;
    }
    typedef CalibPoseValidator<Scalar> validator;
    typedef CameraPose<Scalar> Solution;
    static std::string name() { return "Rel8pt"; }
};

template <typename Scalar>
struct SolverRel5pt {
  static inline int solve(const RelativePoseProblemInstance<Scalar> &instance, std::vector<CameraPose<Scalar>> *solutions)
  {
    EntoArray<CameraPose<Scalar>, 40> solutions_;
    int sols = relpose_5pt<Scalar>(instance.x1_, instance.x2_, &solutions_);
    for (size_t i = 0; i < sols; i++)
    {
      solutions->emplace_back(solutions_[i]);
    }
    return sols;
  }
  typedef CalibPoseValidator<Scalar> validator;
  typedef CameraPose<Scalar> Solution;
  static std::string name() { return "Rel5pt"; }
};

template <typename Scalar>
struct SolverRelUprightPlanar2pt {
  static inline int solve(const RelativePoseProblemInstance<Scalar> &instance, std::vector<CameraPose<Scalar>> *solutions)
  {
    EntoArray<CameraPose<Scalar>, 4> solutions_;
    int sols = relpose_upright_planar_2pt<Scalar>(instance.x1_, instance.x2_, &solutions_);
    for (int i = 0; i < sols; i++)
    {
      solutions->emplace_back(solutions_[i]);
    }
    return sols;
  }
  typedef CalibPoseValidator<Scalar> validator;
  typedef CameraPose<Scalar> Solution;
  static std::string name() { return "RelUprightPlanar2pt"; }
};

template <typename Scalar>
struct SolverRelUprightPlanar3pt {
  static inline int solve(const RelativePoseProblemInstance<Scalar> &instance,
                          std::vector<CameraPose<Scalar>> *solutions)
  {
    EntoArray<CameraPose<Scalar>, 2> solutions_;
    int sols = relpose_upright_planar_3pt<Scalar>(instance.x1_, instance.x2_, &solutions_);
    for (int i = 0; i < sols; i++)
    {
      solutions->emplace_back(solutions_[i]);
    }
    return sols;
  }
  typedef CalibPoseValidator<Scalar> validator;
  typedef CameraPose<Scalar> Solution;
  static std::string name() { return "RelUprightPlanar3pt"; }
};

template <typename Scalar, bool CheiralCheck = false, int Method=0>
struct SolverHomography4pt {
  static inline int solve(const RelativePoseProblemInstance<Scalar> &instance, std::vector<Matrix3x3<Scalar>> *solutions) {
    Matrix3x3<Scalar> H;
    int sols = homography_4pt<Scalar, CheiralCheck, Method>(instance.x1_, instance.x2_, &H);
    solutions->clear();
    if (sols == 1)
    {
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

template <typename Scalar, bool CheiralCheck = false, int Method=0>
struct SolverHomography4ptDLT {
  static inline int solve(const RelativePoseProblemInstance<Scalar> &instance, std::vector<Matrix3x3<Scalar>> *solutions) {
    Matrix3x3<Scalar> H;
    int sols = homography_Npt<Scalar, 0, CheiralCheck, 0, 0>(instance.x1_, instance.x2_, &H);
    solutions->clear();
    if (sols == 1)
    {
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

template <typename Scalar, bool CheiralCheck = false, int Method=0>
struct SolverHomographyNptDLT {
  static inline int solve(const RelativePoseProblemInstance<Scalar> &instance, std::vector<Matrix3x3<Scalar>> *solutions) {
    Matrix3x3<Scalar> H;
    int sols = homography_Npt<Scalar, 0, CheiralCheck, 0, 0>(instance.x1_, instance.x2_, &H);
    solutions->clear();
    if (sols == 1)
    {
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

} // namespace EntoPose

#endif // DATA_GEN_H
