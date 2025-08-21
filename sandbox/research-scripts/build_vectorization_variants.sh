#!/bin/bash

# Build script for vectorization comparison
# Builds the same benchmark with and without vectorized SAD

set -e

BENCHMARK_TARGET=${1:-"bench_block_of_small"}

echo "=== Building Vectorization Variants ==="
echo "Target: $BENCHMARK_TARGET"
echo

# Build with vectorized SAD
echo "--- Building with vectorized SAD ---"
cd build
make clean > /dev/null 2>&1 || true
cmake .. -DCMAKE_CXX_FLAGS="-DENTO_USE_VECTORIZED_SAD" > /dev/null
make $BENCHMARK_TARGET -j$(nproc)
cp benchmark/perception/$BENCHMARK_TARGET benchmark/perception/${BENCHMARK_TARGET}_vectorized
echo "Built: ${BENCHMARK_TARGET}_vectorized"

# Build without vectorized SAD (pixel-by-pixel)
echo "--- Building with pixel-by-pixel SAD ---"
make clean > /dev/null 2>&1 || true
cmake .. > /dev/null  # No special flags
make $BENCHMARK_TARGET -j$(nproc)
cp benchmark/perception/$BENCHMARK_TARGET benchmark/perception/${BENCHMARK_TARGET}_scalar
echo "Built: ${BENCHMARK_TARGET}_scalar"

cd ..

echo
echo "=== Build Complete ==="
echo "Vectorized version: build/benchmark/perception/${BENCHMARK_TARGET}_vectorized"
echo "Scalar version: build/benchmark/perception/${BENCHMARK_TARGET}_scalar"
echo
echo "Usage:"
echo "  ./build/benchmark/perception/${BENCHMARK_TARGET}_vectorized"
echo "  ./build/benchmark/perception/${BENCHMARK_TARGET}_scalar" 