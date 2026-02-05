#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct apriltag3_cpp_codeword {
    int id;
    uint64_t code;
} apriltag3_cpp_codeword;

uint64_t apriltag3_cpp_rotate_code90(uint64_t code, int d);
int apriltag3_cpp_popcount_u64(uint64_t v);

int apriltag3_cpp_best_codeword_match(
    uint64_t observed,
    int d,
    const apriltag3_cpp_codeword* codebook,
    int codebook_size,
    int* out_best_id,
    int* out_best_hamming,
    int* out_best_rotation);

#ifdef __cplusplus
}
#endif
