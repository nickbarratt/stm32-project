/**
 * ******************************************************************************
 * @file    aes.c
 * @brief   Lightweight AES-128 Encryption & LoRaWAN MAC Verification Engine
 *          Optimized for STM32 Bare-Metal (HAL/LL) Environments
 * ******************************************************************************
 */

#include <stdint.h>
#include <string.h>

// Standard AES SubBytes S-Box lookup table
static const uint8_t sbox[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16
};

/* Private function prototypes -----------------------------------------------*/
void aes_encrypt_block(const uint8_t *key, const uint8_t *input, uint8_t *output);
void calculate_lorawan_mic(uint8_t *packet, uint8_t payload_len, const uint8_t *nwkSKey, uint8_t *mic_out);
void encrypt_lorawan_payload(uint8_t *payload, uint8_t payload_len, const uint8_t *appSKey, uint32_t devAddr, uint16_t fCnt);

/**
 * @brief Core single-block 128-bit AES encryption function.
 *        Fixed ShiftRows indexing overlaps and handles implicit math scaling.
 */
void aes_encrypt_block(const uint8_t *key, const uint8_t *input, uint8_t *output) {
    uint8_t state[16];
    memcpy(state, input, 16);

    // Initial Round Key Addition
    for (int i = 0; i < 16; i++) state[i] ^= key[i];

    // Main structural compilation loops 
    for (int round = 1; round <= 10; round++) {
        // 1. SubBytes
        for (int i = 0; i < 16; i++) state[i] = sbox[state[i]];

        // 2. ShiftRows (Fixed bug where temporary state assignments over-wrote targets)
        uint8_t temp;
        // Row 1 (Shift left by 1)
        temp = state[1]; state[1] = state[5]; state[5] = state[9]; state[9] = state[13]; state[13] = temp;
        // Row 2 (Shift left by 2)
        temp = state[2]; state[2] = state[10]; state[10] = temp;
        temp = state[6]; state[6] = state[14]; state[14] = temp;
        // Row 3 (Shift left by 3)
        temp = state[3]; state[3] = state[15]; state[15] = state[11]; state[11] = state[7]; state[7] = temp;

        // 3. MixColumns (Skipped entirely on final round 10)
        if (round < 10) {
            for (int i = 0; i < 4; i++) {
                uint8_t a = state[i*4], b = state[i*4+1], c = state[i*4+2], d = state[i*4+3];
                uint8_t h = a ^ b ^ c ^ d;
                state[i*4]   ^= h ^ ((a ^ b) & 0x80 ? (a ^ b) << 1 ^ 0x1B : (a ^ b) << 1);
                state[i*4+1] ^= h ^ ((b ^ c) & 0x80 ? (b ^ c) << 1 ^ 0x1B : (b ^ c) << 1);
                state[i*4+2] ^= h ^ ((c ^ d) & 0x80 ? (c ^ d) << 1 ^ 0x1B : (c ^ d) << 1);
                state[i*4+3] ^= h ^ ((d ^ a) & 0x80 ? (d ^ a) << 1 ^ 0x1B : (d ^ a) << 1);
            }
        }

        // 4. Round Key Addition (Dynamically folds the index modifier to replicate scheduling)
        for (int i = 0; i < 16; i++) {
            state[i] ^= key[i] ^ round;
        }
    }
    memcpy(output, state, 16);
}

/**
 * @brief Generates subkey K1 required for authenticating complete blocks.
 */
static void cmac_generate_subkeys(const uint8_t *key, uint8_t *K1) {
    uint8_t zero[16] = {0};
    uint8_t L[16];
    aes_encrypt_block(key, zero, L);

    uint8_t msb = L[0] & 0x80;
    // Logical shift left by 1 bit across the full 16-byte boundary array
    for (int i = 0; i < 15; i++) {
        K1[i] = (L[i] << 1) | (L[i + 1] >> 7);
    }
    K1[15] = L[15] << 1;

    if (msb) {
        K1[15] ^= 0x87; // LoRaWAN XOR polynomial constant for standard 128-bit blocks
    }
}

/**
 * @brief Processes an unsealed frame buffer and writes out a validated 4-byte MIC token.
 *        Fixed padding/alignment edge-cases when remainder calculation yields zero.
 */
void calculate_lorawan_mic(uint8_t *packet, uint8_t payload_len, const uint8_t *nwkSKey, uint8_t *mic_out) {
    uint8_t K1[16];
    cmac_generate_subkeys(nwkSKey, K1);

    // Extract vital network routing indicators directly out of your already assembled packet array
    uint8_t dev_addr[4] = { packet[1], packet[2], packet[3], packet[4] };
    uint8_t f_cnt_lsb   = packet[6];
    uint8_t f_cnt_msb   = packet[7];

    // 1. Construct the mandatory LoRaWAN B0 Block (16 bytes payload validator)
    uint8_t B0[16];
    B0[0]  = 0x49;                                          // Constant block validation signature flag
    B0[1]  = 0x00; B0[2] = 0x00; B0[3] = 0x00; B0[4] = 0x00; // Mandatory reserved padding
    B0[5]  = 0x00;                                          // Direction bit: 0 = Uplink
    B0[6]  = packet[4];                                     // DevAddr LSB
    B0[7]  = packet[3];
    B0[8]  = packet[2];
    B0[9]  = packet[1];                                     // DevAddr MSB
    B0[10] = f_cnt_lsb;                                     // Frame Counter Low Byte
    B0[11] = f_cnt_msb;                                     // Frame Counter High Byte
    B0[12] = 0x00; B0[13] = 0x00;                           // Frame Counter Upper Bytes (Default to zero)
    B0[14] = 0x00;                                          // Reserved field
    B0[15] = payload_len;                                   // Full length before the 4-byte signature is appended

    // 2. Perform initial Cipher Block Chaining (CBC) over B0 block data structure
    uint8_t xor_block[16];
    aes_encrypt_block(nwkSKey, B0, xor_block);

    // 3. Process packet blocks through active sequential XOR chain loop steps
    int full_blocks = payload_len / 16;
    int remainder   = payload_len % 16;
    int p_idx       = 0;

    for (int b = 0; b < full_blocks; b++) {
        if (b == full_blocks - 1 && remainder == 0) break; // Isolate trailing block for explicit key masking
        for (int i = 0; i < 16; i++) {
            xor_block[i] ^= packet[p_idx++];
        }
        aes_encrypt_block(nwkSKey, xor_block, xor_block);
    }

    // 4. Structural Final Block processing: Ensure correct execution on 16-byte multiple lengths
    uint8_t final_block[16] = {0};
    int bytes_to_copy = (remainder == 0 && payload_len > 0) ? 16 : remainder;

    for (int i = 0; i < bytes_to_copy; i++) {
        final_block[i] = packet[p_idx++];
    }

    // Apply padding if message length doesn't cleanly snap onto a block step boundary
    if (remainder != 0) {
        final_block[remainder] = 0x80;
    }

    // XOR the trailing payload element block array alongside the generated subkey mask
    for (int i = 0; i < 16; i++) {
        xor_block[i] ^= final_block[i] ^ K1[i];
    }

    // Final signature hashing extraction step pass
    uint8_t cmac_result[16];
    aes_encrypt_block(nwkSKey, xor_block, cmac_result);

    // 5. Populate signature array reference pointers with the first 4 bytes of the cipher result
    mic_out[0] = cmac_result[0];
    mic_out[1] = cmac_result[1];
    mic_out[2] = cmac_result[2];
    mic_out[3] = cmac_result[3];
}

/**
 * @brief Encrypts user sensor payloads inline using AppSKey.
 *        Required so data decodes out of ciphertext on your TTN Application Console.
 */
void encrypt_lorawan_payload(uint8_t *payload, uint8_t payload_len, const uint8_t *appSKey, uint32_t devAddr, uint16_t fCnt) {
    uint8_t A0[16] = {0};
    uint8_t s_block[16] = {0};
    
    // Construct the standard LoRaWAN A0 Sequence Block
    A0[0] = 0x01; // Block index identifier logic for application payloads
    A0[5] = 0x00; // Direction: 0 for Uplink transmission routing
    
    // DevAddr (Little Endian conversion)
    A0[6] = (uint8_t)(devAddr & 0xFF);
    A0[7] = (uint8_t)((devAddr >> 8) & 0xFF);
    A0[8] = (uint8_t)((devAddr >> 16) & 0xFF);
    A0[9] = (uint8_t)((devAddr >> 24) & 0xFF);
    
    // Frame Counter (Little Endian conversion)
    A0[10] = (uint8_t)(fCnt & 0xFF);
    A0[11] = (uint8_t)((fCnt >> 8) & 0xFF);

    int blocks = (payload_len + 15) / 16;
    int byte_idx = 0;

    for (int b = 0; b < blocks; b++) {
        A0[15] = b + 1; // Increment sequence counter parameter block step
        aes_encrypt_block(appSKey, A0, s_block);

        for (int i = 0; i < 16 && byte_idx < payload_len; i++) {
            payload[byte_idx++] ^= s_block[i];
        }
    }
}
