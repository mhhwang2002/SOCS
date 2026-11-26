#include <cuda_runtime.h>
#include <cstddef>
#include <cstdint>

#ifdef _WIN32
#define RMFS_EXPORT __declspec(dllexport)
#else
#define RMFS_EXPORT __attribute__((visibility("default")))
#endif

// Each box is packed as lower x/y followed by upper x/y.
__global__ void intersect_boxes(const double* boxes, size_t first_count,
                                size_t pair_count, double tolerance, int* hit) {
    for (size_t pair = blockIdx.x * blockDim.x + threadIdx.x;
         pair < pair_count; pair += gridDim.x * blockDim.x) {
        const double* a = boxes + 4 * (pair % first_count);
        const double* b = boxes + 4 * (first_count + pair / first_count);
        if (a[0] <= b[2] + tolerance && b[0] <= a[2] + tolerance &&
            a[1] <= b[3] + tolerance && b[1] <= a[3] + tolerance) {
            atomicExch(hit, 1);
            return;
        }
    }
}

extern "C" RMFS_EXPORT const char* rmfs_cuda_error(int code) {
    return cudaGetErrorString(static_cast<cudaError_t>(code));
}

extern "C" RMFS_EXPORT int rmfs_cuda_check_device() {
    int count = 0;
    const auto status = cudaGetDeviceCount(&count);
    if (status != cudaSuccess) return status;
    return count > 0 ? cudaSuccess : cudaErrorNoDevice;
}

extern "C" RMFS_EXPORT int rmfs_cuda_intersects(
    const double* boxes, size_t first_count, size_t second_count,
    double tolerance, int* hit) {
    if (first_count == 0 || second_count == 0) {
        *hit = 0;
        return cudaSuccess;
    }
    if (first_count > SIZE_MAX / second_count ||
        second_count > SIZE_MAX / (4 * sizeof(double)) ||
        first_count > SIZE_MAX / (4 * sizeof(double)) - second_count) {
        return cudaErrorInvalidValue;
    }
    const size_t bytes = 4 * sizeof(double) * (first_count + second_count);
    const size_t pairs = first_count * second_count;
    double* device_boxes = nullptr;
    int* device_hit = nullptr;
    auto status = cudaMalloc(&device_boxes, bytes);
    if (status == cudaSuccess) status = cudaMalloc(&device_hit, sizeof(int));
    if (status == cudaSuccess) {
        status = cudaMemcpy(device_boxes, boxes, bytes, cudaMemcpyHostToDevice);
    }
    if (status == cudaSuccess) status = cudaMemset(device_hit, 0, sizeof(int));
    if (status == cudaSuccess) {
        constexpr int threads = 256;
        const size_t blocks = (pairs - 1) / threads + 1;
        intersect_boxes<<<blocks < 256 ? blocks : 256, threads>>>(
            device_boxes, first_count, pairs, tolerance, device_hit);
        status = cudaGetLastError();
    }
    if (status == cudaSuccess) {
        status = cudaMemcpy(hit, device_hit, sizeof(int), cudaMemcpyDeviceToHost);
    }
    const auto free_hit = cudaFree(device_hit);
    const auto free_boxes = cudaFree(device_boxes);
    if (status != cudaSuccess) return status;
    return free_hit != cudaSuccess ? free_hit : free_boxes;
}
