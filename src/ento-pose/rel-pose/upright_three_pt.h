#ifndef UPRIGHT_THREE_PT_H
#define UPRIGHT_THREE_PT_H

#include <array>
#include <cmath>
#include <ento-math/core.h>
#include <ento-math/quaternion.h>
#include <ento-pose/pose_util.h>
#include <ento-util/containers.h>

namespace EntoPose {


template <typename Scalar, std::size_t N>
int relpose_upright_3pt(const EntoUtil::EntoContainer<EntoMath::Vec3<Scalar>, N> &x1,
                        const EntoUtil::EntoContainer<EntoMath::Vec3<Scalar>, N> &x2,
                        EntoUtil::EntoContainer<CameraPose<Scalar>, 4> *output) {
    using Vec3 = EntoMath::Vec3<Scalar>;
    using Mat3 = EntoMath::Matrix3x3<Scalar>;
    output->clear();
    //if (x1.size() < 3 || x2.size() < 3) return 0;

    ENTO_DEBUG("relpose_upright_3pt: Input x1 vectors:");
    for (size_t i = 0; i < 3; ++i) {
        ENTO_DEBUG("  x1[%zu] = (%f, %f, %f)", i, x1[i](0), x1[i](1), x1[i](2));
    }
    ENTO_DEBUG("relpose_upright_3pt: Input x2 vectors:");
    for (size_t i = 0; i < 3; ++i) {
        ENTO_DEBUG("  x2[%zu] = (%f, %f, %f)", i, x2[i](0), x2[i](1), x2[i](2));
    }

    // Compute cross products
    Vec3 u1 = x1[0].cross(x1[1]);
    Vec3 v1 = x2[1].cross(x2[0]);
    Vec3 u2 = x1[0].cross(x1[2]);
    Vec3 v2 = x2[2].cross(x2[0]);

    Scalar a[12], b[12], m[12];
    a[0] = u1(0); a[1] = u1(1); a[2] = u1(2);
    a[3] = x2[1](0); a[4] = x2[1](1); a[5] = x2[1](2);
    a[6] = v1(0); a[7] = v1(1); a[8] = v1(2);
    a[9] = x1[1](0); a[10] = x1[1](1); a[11] = x1[1](2);
    b[0] = u2(0); b[1] = u2(1); b[2] = u2(2);
    b[3] = x2[2](0); b[4] = x2[2](1); b[5] = x2[2](2);
    b[6] = v2(0); b[7] = v2(1); b[8] = v2(2);
    b[9] = x1[2](0); b[10] = x1[2](1); b[11] = x1[2](2);
    m[0] = a[1] * a[4];
    m[1] = a[0] * a[3] + a[2] * a[5];
    m[2] = a[2] * a[3] - a[0] * a[5];
    m[5] = a[8] * a[9] - a[6] * a[11];
    m[4] = -a[6] * a[9] - a[8] * a[11];
    m[3] = -a[7] * a[10];
    m[8] = b[2] * b[3] - b[0] * b[5];
    m[7] = b[0] * b[3] + b[2] * b[5];
    m[6] = b[1] * b[4];
    m[11] = b[8] * b[9] - b[6] * b[11];
    m[10] = -b[6] * b[9] - b[8] * b[11];
    m[9] = -b[7] * b[10];

    Mat3 D1, D2;
    D1 << m[0] * m[9] - m[3] * m[6], (m[0] * m[10] + m[1] * m[9] - m[3] * m[7] - m[4] * m[6]) * Scalar(0.5),
        (m[0] * m[11] + m[2] * m[9] - m[3] * m[8] - m[5] * m[6]) * Scalar(0.5),
        (m[0] * m[10] + m[1] * m[9] - m[3] * m[7] - m[4] * m[6]) * Scalar(0.5), m[1] * m[10] - m[4] * m[7],
        (m[1] * m[11] + m[2] * m[10] - m[4] * m[8] - m[5] * m[7]) * Scalar(0.5),
        (m[0] * m[11] + m[2] * m[9] - m[3] * m[8] - m[5] * m[6]) * Scalar(0.5),
        (m[1] * m[11] + m[2] * m[10] - m[4] * m[8] - m[5] * m[7]) * Scalar(0.5), m[2] * m[11] - m[5] * m[8];
    D2.setZero(); D2(0,0) = -Scalar(1); D2(1,1) = Scalar(1); D2(2,2) = Scalar(1);

    Mat3 DX1, DX2;
    DX1.col(0) = D1.col(1).cross(D1.col(2));
    DX1.col(1) = D1.col(2).cross(D1.col(0));
    DX1.col(2) = D1.col(0).cross(D1.col(1));
    DX2.col(0) = D2.col(1).cross(D2.col(2));
    DX2.col(1) = D2.col(2).cross(D2.col(0));
    DX2.col(2) = D2.col(0).cross(D2.col(1));

    Scalar k3 = D2.col(0).dot(DX2.col(0));
    Scalar k2 = (D1.array() * DX2.array()).sum();
    Scalar k1 = (D2.array() * DX1.array()).sum();
    Scalar k0 = D1.col(0).dot(DX1.col(0));

    Scalar k3_inv = Scalar(1) / k3;
    k2 *= k3_inv;
    k1 *= k3_inv;
    k0 *= k3_inv;

    Scalar s;
    bool G = EntoPose::solve_cubic_single_real(k2, k1, k0, s);

    Mat3 C = D1 + s * D2;
    std::array<Vec3, 2> pq = EntoPose::compute_pq(C);

    int n_sols = 0;
    for (int i = 0; i < 2; ++i) {
        Scalar p1 = pq[i](0);
        Scalar p2 = pq[i](1);
        Scalar p3 = pq[i](2);
        bool switch_23 = std::abs(p3) <= std::abs(p2);
        if (switch_23) {
            Scalar w0 = -p1 / p2;
            Scalar w1 = -p3 / p2;
            Scalar ca = Scalar(1) / (w1 * w1 + Scalar(1));
            Scalar cb = Scalar(2) * w0 * w1 * ca;
            Scalar cc = (w0 * w0 - Scalar(1)) * ca;
            Scalar taus[2];
            if (!EntoPose::root2real(cb, cc, taus[0], taus[1])) continue;
            for (Scalar sq : taus) {
                Scalar cq = w0 + w1 * sq;
                Scalar lambda = -(m[0] + m[1] * cq + m[2] * sq) / (m[3] + m[4] * cq + m[5] * sq);
                if (lambda < Scalar(0)) continue;
                Mat3 R = Mat3::Identity();
                R(0, 0) = cq;
                R(0, 2) = sq;
                R(2, 0) = -sq;
                R(2, 2) = cq;
                Vec3 trans = lambda * x2[0] - R * x1[0];
                trans.normalize();
                output->push_back(CameraPose<Scalar>(R, trans));
                ENTO_DEBUG("relpose_upright_3pt: Solution %d: R = [ [%f, %f, %f], [%f, %f, %f], [%f, %f, %f] ], t = (%f, %f, %f)",
                    n_sols,
                    R(0,0), R(0,1), R(0,2),
                    R(1,0), R(1,1), R(1,2),
                    R(2,0), R(2,1), R(2,2),
                    trans(0), trans(1), trans(2));
                ++n_sols;
                if (n_sols >= 4) break;
            }
        } else {
            Scalar w0 = -p1 / p3;
            Scalar w1 = -p2 / p3;
            Scalar ca = Scalar(1) / (w1 * w1 + Scalar(1));
            Scalar cb = Scalar(2) * w0 * w1 * ca;
            Scalar cc = (w0 * w0 - Scalar(1)) * ca;
            Scalar taus[2];
            if (!EntoPose::root2real(cb, cc, taus[0], taus[1])) continue;
            for (Scalar cq : taus) {
                Scalar sq = w0 + w1 * cq;
                Scalar lambda = -(m[0] + m[1] * cq + m[2] * sq) / (m[3] + m[4] * cq + m[5] * sq);
                if (lambda < Scalar(0)) continue;
                Mat3 R = Mat3::Identity();
                R(0, 0) = cq;
                R(0, 2) = sq;
                R(2, 0) = -sq;
                R(2, 2) = cq;
                Vec3 trans = lambda * x2[0] - R * x1[0];
                trans.normalize();
                output->push_back(CameraPose<Scalar>(R, trans));
                ENTO_DEBUG("relpose_upright_3pt: Solution %d: R = [ [%f, %f, %f], [%f, %f, %f], [%f, %f, %f] ], t = (%f, %f, %f)",
                    n_sols,
                    R(0,0), R(0,1), R(0,2),
                    R(1,0), R(1,1), R(1,2),
                    R(2,0), R(2,1), R(2,2),
                    trans(0), trans(1), trans(2));
                ++n_sols;
                if (n_sols >= 4) break;
            }
        }
        if (n_sols > 0 && G) break;
    }
    ENTO_DEBUG("relpose_upright_3pt: Number of solutions found: %d", n_sols);
    return n_sols;
}

// Gravity-aware entry point
template <typename Scalar, std::size_t N>
int relpose_upright_3pt(const EntoUtil::EntoContainer<EntoMath::Vec3<Scalar>, N> &x1,
                        const EntoUtil::EntoContainer<EntoMath::Vec3<Scalar>, N> &x2,
                        const EntoMath::Vec3<Scalar> &g_cam1,
                        const EntoMath::Vec3<Scalar> &g_cam2,
                        EntoUtil::EntoContainer<CameraPose<Scalar>, 4> *output) {
    using Vec3 = EntoMath::Vec3<Scalar>;
    using Mat3 = EntoMath::Matrix3x3<Scalar>;
    // Compute rotation matrices to align g_cam1 and g_cam2 to y-axis
    Mat3 Rc1 = rotation_from_two_vectors<Scalar>(g_cam1, Vec3(0,1,0));
    Mat3 Rc2 = rotation_from_two_vectors<Scalar>(g_cam2, Vec3(0,1,0));
    EntoUtil::EntoContainer<Vec3, N> x1_upright, x2_upright;
    x1_upright.clear(); x2_upright.clear();
    for (size_t i = 0; i < 3; ++i) {
        x1_upright.push_back(Rc1 * x1[i]);
        x2_upright.push_back(Rc2 * x2[i]);
    }
    int n_sols = relpose_upright_3pt<Scalar, N>(x1_upright, x2_upright, output);
    // De-rotate solutions
    for (int i = 0; i < n_sols; ++i) {
        Mat3 R = output->operator[](i).R();
        Vec3 t = output->operator[](i).t;
        t = Rc2.transpose() * t;
        R = Rc2.transpose() * R * Rc1;
        output->operator[](i) = CameraPose<Scalar>(R, t);
    }
    return n_sols;
}

} // namespace EntoPose

#endif // UPRIGHT_THREE_PT_H
