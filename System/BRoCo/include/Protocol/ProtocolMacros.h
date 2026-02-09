#ifndef PROTOCOL_MACROS_H
#define PROTOCOL_MACROS_H


#ifdef __cplusplus
#include <cstdint>
#else
#include <stdint.h>
#endif

static uint16_t __gen_crc16(const uint8_t *data, uint16_t size) {
    uint8_t x;
    uint16_t crc = 0xFFFF;

    while (size--){
        x = crc >> 8 ^ *data++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
    }
    return crc;
}

#define STANDARD_PACKET(NAME, PACKET_DEF) typedef struct NAME { PACKET_DEF } __attribute__((packed)) NAME;
#define RELIABLE_PACKET(NAME, PACKET_DEF) typedef struct NAME { PACKET_DEF uint16_t crc; } __attribute__((packed)) NAME;
#define IDENTIFIABLE_PACKET(NAME, PACKET_DEF) typedef struct NAME { PACKET_DEF uint16_t id; } __attribute__((packed)) NAME;
#define RELIABLE_IDENTIFIABLE_PACKET(NAME, PACKET_DEF) typedef struct NAME { PACKET_DEF uint16_t id; uint16_t crc; } __attribute__((packed)) NAME;
#define MAKE_RELIABLE(PACKET) (PACKET).crc = __gen_crc16((uint8_t*) &(PACKET), sizeof((PACKET)) - 2)
#define IS_RELIABLE(PACKET) (PACKET).crc == __gen_crc16((uint8_t*) &(PACKET), sizeof((PACKET)) - 2)

#endif /* PROTOCOL_MACROS_H */
