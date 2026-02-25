#include "apriltag3_cpp/decode_core.h"

uint64_t apriltag3_cpp_rotate_code90(uint64_t code, int d) {
    uint64_t rotated = 0;
    for (int y = 0; y < d; ++y) {
        for (int x = 0; x < d; ++x) {
            const int src_bit = y * d + x;
            const int dst_x = d - 1 - y;
            const int dst_y = x;
            const int dst_bit = dst_y * d + dst_x;
            if (((code >> src_bit) & 1ULL) != 0ULL) {
                rotated |= (1ULL << dst_bit);
            }
        }
    }
    return rotated;
}

int apriltag3_cpp_popcount_u64(uint64_t v) {
#if defined(__GNUC__)
    return __builtin_popcountll(v);
#else
    int count = 0;
    while (v != 0) {
        v &= (v - 1);
        ++count;
    }
    return count;
#endif
}

int apriltag3_cpp_best_codeword_match(
    uint64_t observed,
    int d,
    const apriltag3_cpp_codeword* codebook,
    int codebook_size,
    int* out_best_id,
    int* out_best_hamming,
    int* out_best_rotation) {
    int best_id = -1;
    int best_hamming = 1 << 30;
    int best_rotation = 0;

    uint64_t rot_code = observed;
    for (int rot = 0; rot < 4; ++rot) {
        for (int i = 0; i < codebook_size; ++i) {
            const int h = apriltag3_cpp_popcount_u64(rot_code ^ codebook[i].code);
            if (h < best_hamming) {
                best_hamming = h;
                best_id = codebook[i].id;
                best_rotation = rot;
            }
        }
        rot_code = apriltag3_cpp_rotate_code90(rot_code, d);
    }

    if (out_best_id != 0) {
        *out_best_id = best_id;
    }
    if (out_best_hamming != 0) {
        *out_best_hamming = best_hamming;
    }
    if (out_best_rotation != 0) {
        *out_best_rotation = best_rotation;
    }
    return 1;
}
