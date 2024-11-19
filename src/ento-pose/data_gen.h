#ifndef DATA_GEN_H
#define DATA_GEN_H

#include <ento-pose/pose_util.h>
#include <ento-pose/prob_gen.h>
#include <ento-pose/abs-pose/p3p.h>
#include <ento-pose/abs-pose/up2p.h>
#include <ento-pose/abs-pose/up2p.h>
#include <ento-pose/abs-pose/p4p.h>

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
        return relpose_8pt(instance.x1_, instance.x2_, solutions);
    }
    typedef CalibPoseValidator<Scalar> validator;
    typedef CameraPose<Scalar> Solution;
    static std::string name() { return "Rel8pt"; }
};

template <typename Scalar>
struct SolverRel5pt {
  static inline int solve(const RelativePoseProblemInstance<Scalar> &instance, std::vector<CameraPose<Scalar>> *solutions)
  {
    return relpose_5pt(instance.x1_, instance.x2_, solutions);
  }
  typedef CalibPoseValidator<Scalar> validator;
  typedef CameraPose<Scalar> Solution;
  static std::string name() { return "Rel5pt"; }
};

template <typename Scalar>
struct SolverRelUprightPlanar2pt {
  static inline int solve(const RelativePoseProblemInstance<Scalar> &instance, std::vector<CameraPose<Scalar>> *solutions)
 {
    return relpose_upright_planar_2pt(instance.x1_, instance.x2_, solutions);
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
    return relpose_upright_planar_3pt(instance.x1_, instance.x2_, solutions);
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

} // namespace EntoPose

#endif // DATA_GEN_H
