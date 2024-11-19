#ifndef FIVE_PT_UTIL_H
#define FIVE_PT_UTIL_H

#include <Eigen/Dense>
#include <ento-pose/pose_util.h>
#include <ento-math/core.h>

// Constructs the quotients needed for evaluating the sturm sequence.
template <typename Scalar, std::size_t N>
void build_sturm_seq(const Scalar *fvec, Scalar *svec) {

    Scalar f[3 * N];
    Scalar *f1 = f;
    Scalar *f2 = f1 + N + 1;
    Scalar *f3 = f2 + N;

    std::copy(fvec, fvec + (2 * N + 1), f);

    for (int i = 0; i < N - 1; ++i) {
        const Scalar q1 = f1[N - i] * f2[N - 1 - i];
        const Scalar q0 = f1[N - 1 - i] * f2[N - 1 - i] - f1[N - i] * f2[N - 2 - i];

        f3[0] = f1[0] - q0 * f2[0];
        for (int j = 1; j < N - 1 - i; ++j) {
            f3[j] = f1[j] - q1 * f2[j - 1] - q0 * f2[j];
        }
        const Scalar c = -std::abs(f3[N - 2 - i]);
        const Scalar ci = 1.0 / c;
        for (int j = 0; j < N - 1 - i; ++j) {
            f3[j] = f3[j] * ci;
        }

        // juggle pointers (f1,f2,f3) -> (f2,f3,f1)
        Scalar *tmp = f1;
        f1 = f2;
        f2 = f3;
        f3 = tmp;

        svec[3 * i] = q0;
        svec[3 * i + 1] = q1;
        svec[3 * i + 2] = c;
    }

    svec[3 * N - 3] = f1[0];
    svec[3 * N - 2] = f1[1];
    svec[3 * N - 1] = f2[0];
}

// Evaluates polynomial using Horner's method.
// Assumes that f[N] = 1.0
template <typename Scalar, std::size_t N>
inline Scalar polyval(const Scalar *f, Scalar x) {
    Scalar fx = x + f[N - 1];
    for (int i = N - 2; i >= 0; --i) {
        fx = x * fx + f[i];
    }
    return fx;
}

// Daniel Thul is responsible for this template-trickery :)
template <typename Scalar, int D>
inline unsigned int flag_negative(const Scalar *const f) {
    return ((f[D] < 0) << D) | flag_negative<D - 1>(f);
}
template <> inline unsigned int flag_negative<float, 0>(const float *const f) { return f[0] < 0; }
template <> inline unsigned int flag_negative<double, 0>(const double *const f) { return f[0] < 0; }

// Evaluates the sturm sequence and counts the number of sign changes
template <typename Scalar, int N, typename std::enable_if<(N < 32), void>::type * = nullptr>
inline int signchanges(const Scalar *svec, Scalar x) {

    Scalar f[N + 1];
    f[N] = svec[3 * N - 1];
    f[N - 1] = svec[3 * N - 3] + x * svec[3 * N - 2];

    for (int i = N - 2; i >= 0; --i) {
        f[i] = (svec[3 * i] + x * svec[3 * i + 1]) * f[i + 1] + svec[3 * i + 2] * f[i + 2];
    }

    // In testing this turned out to be slightly faster compared to a naive loop
    unsigned int S = flag_negative<N>(f);

    return __builtin_popcount((S ^ (S >> 1)) & ~(0xFFFFFFFF << N));
}

template <typename Scalar, std::size_t N, typename std::enable_if<(N >= 32), void>::type * = nullptr>
inline int signchanges(const Scalar *svec, Scalar x) {

    Scalar f[N + 1];
    f[N] = svec[3 * N - 1];
    f[N - 1] = svec[3 * N - 3] + x * svec[3 * N - 2];

    for (int i = N - 2; i >= 0; --i) {
        f[i] = (svec[3 * i] + x * svec[3 * i + 1]) * f[i + 1] + svec[3 * i + 2] * f[i + 2];
    }

    int count = 0;
    bool neg1 = f[0] < 0;
    for (int i = 0; i < N; ++i) {
        bool neg2 = f[i + 1] < 0;
        if (neg1 ^ neg2) {
            ++count;
        }
        neg1 = neg2;
    }
    return count;
}

// Computes the Cauchy bound on the real roots.
// Experiments with more complicated (expensive) bounds did not seem to have a good trade-off.
template <typename Scalar, std::size_t N>
inline Scalar get_bounds(const Scalar *fvec) {
    Scalar max = 0;
    for (int i = 0; i < N; ++i) {
        max = std::max(max, std::abs(fvec[i]));
    }
    return 1.0 + max;
}

// Applies Ridder's bracketing method until we get close to root, followed by newton iterations
template <typename Scalar, std::size_t N>
void ridders_method_newton(const Scalar *fvec, Scalar a, Scalar b, Scalar *roots, int &n_roots, Scalar tol) {
    Scalar fa = polyval<N>(fvec, a);
    Scalar fb = polyval<N>(fvec, b);

    if (!((fa < 0) ^ (fb < 0)))
        return;

    const Scalar tol_newton = 1e-3;

    for (int iter = 0; iter < 30; ++iter) {
        if (std::abs(a - b) < tol_newton) {
            break;
        }
        const Scalar c = (a + b) * 0.5;
        const Scalar fc = polyval<N>(fvec, c);
        const Scalar s = std::sqrt(fc * fc - fa * fb);
        if (!s)
            break;
        const Scalar d = (fa < fb) ? c + (a - c) * fc / s : c + (c - a) * fc / s;
        const Scalar fd = polyval<N>(fvec, d);

        if (fd >= 0 ? (fc < 0) : (fc > 0)) {
            a = c;
            fa = fc;
            b = d;
            fb = fd;
        } else if (fd >= 0 ? (fa < 0) : (fa > 0)) {
            b = d;
            fb = fd;
        } else {
            a = d;
            fa = fd;
        }
    }

    // We switch to Newton's method once we are close to the root
    Scalar x = (a + b) * 0.5;

    Scalar fx, fpx, dx;
    const Scalar *fpvec = fvec + N + 1;
    for (int iter = 0; iter < 10; ++iter) {
        fx = polyval<N>(fvec, x);
        if (std::abs(fx) < tol) {
            break;
        }
        fpx = static_cast<Scalar>(N) * polyval<N - 1>(fpvec, x);
        dx = fx / fpx;
        x = x - dx;
        if (std::abs(dx) < tol) {
            break;
        }
    }

    roots[n_roots++] = x;
}

template <typename Scalar, std::size_t N>
void isolate_roots(const Scalar *fvec, const Scalar *svec, Scalar a, Scalar b, int sa, int sb, Scalar *roots,
                   int &n_roots, Scalar tol, int depth) {
    if (depth > 300)
        return;

    int n_rts = sa - sb;

    if (n_rts > 1) {
        Scalar c = (a + b) * 0.5;
        int sc = signchanges<N>(svec, c);
        isolate_roots<N>(fvec, svec, a, c, sa, sc, roots, n_roots, tol, depth + 1);
        isolate_roots<N>(fvec, svec, c, b, sc, sb, roots, n_roots, tol, depth + 1);
    } else if (n_rts == 1) {
        ridders_method_newton<N>(fvec, a, b, roots, n_roots, tol);
    }
}

template <typename Scalar, std::size_t N>
inline int bisect_sturm(const Scalar *coeffs, Scalar *roots, Scalar tol = 1e-10) {
    if (coeffs[N] == 0.0)
        return 0; // return bisect_sturm<N-1>(coeffs,roots,tol); // This explodes compile times...

    Scalar fvec[2 * N + 1];
    Scalar svec[3 * N];

    // fvec is the polynomial and its first derivative.
    std::copy(coeffs, coeffs + N + 1, fvec);

    // Normalize w.r.t. leading coeff
    Scalar c_inv = 1.0 / fvec[N];
    for (int i = 0; i < N; ++i)
        fvec[i] *= c_inv;
    fvec[N] = 1.0;

    // Compute the derivative with normalized coefficients
    for (int i = 0; i < N - 1; ++i) {
        fvec[N + 1 + i] = fvec[i + 1] * ((i + 1) / static_cast<Scalar>(N));
    }
    fvec[2 * N] = 1.0;

    // Compute sturm sequences
    build_sturm_seq<N>(fvec, svec);

    // All real roots are in the interval [-r0, r0]
    Scalar r0 = get_bounds<N>(fvec);
    Scalar a = -r0;
    Scalar b = r0;

    int sa = signchanges<N>(svec, a);
    int sb = signchanges<N>(svec, b);

    int n_roots = sa - sb;
    if (n_roots == 0)
        return 0;

    n_roots = 0;
    isolate_roots<N>(fvec, svec, a, b, sa, sb, roots, n_roots, tol, 0);

    return n_roots;
}

template <> inline int bisect_sturm<float, 1>(const float *coeffs, float *roots, float tol) {
    if (coeffs[1] == 0.0) {
        return 0;
    } else {
        roots[0] = -coeffs[0] / coeffs[1];
        return 1;
    }
}
template <> inline int bisect_sturm<float, 0>(const float *coeffs, float *roots, float tol) { return 0; }

template <> inline int bisect_sturm<double, 1>(const double *coeffs, double *roots, double tol) {
    if (coeffs[1] == 0.0) {
        return 0;
    } else {
        roots[0] = -coeffs[0] / coeffs[1];
        return 1;
    }
}
template <> inline int bisect_sturm<double, 0>(const double *coeffs, double *roots, double tol) { return 0; }

#endif
