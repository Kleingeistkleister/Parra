#include "klipper_protocol.h"
#include "usart.h" // Include the appropriate USART header for your board
#include "crc.h"   // Include the CRC library if available

#define SYNC_BYTE 0x7E

static uint8_t calculate_crc(const uint8_t *data, uint16_t length);

void klipper_send_command(UART_HandleTypeDef *huart, const char *cmd) {
    uint8_t message[64];
    uint8_t length = 0;
    uint8_t sequence = 0; // You should manage the sequence number appropriately
    
    // Encode the message
    length = encode_command(message + 2, cmd);
    
    // Set length and sequence
    message[0] = length + 5; // Total length including header and trailer
    message[1] = sequence | 0x10; // Sequence number with reserved bits
    
    // Calculate CRC
    uint16_t crc = calculate_crc(message, length + 2);
    message[length + 2] = (crc >> 8) & 0xFF;
    message[length + 3] = crc & 0xFF;
    
    // Add sync byte
    message[length + 4] = SYNC_BYTE;
    
    // Send message over UART
    HAL_UART_Transmit(huart, message, length + 5, HAL_MAX_DELAY);
}

int klipper_receive_response(UART_HandleTypeDef *huart, char *buffer, uint16_t size) {
    uint8_t message[64];
    int length = HAL_UART_Receive(huart, message, sizeof(message), HAL_MAX_DELAY);
    
    if (length < 0) {
        return -1; // Error
    }
    
    // Validate the message
    if (message[length - 1] != SYNC_BYTE) {
        return -1; // Invalid sync byte
    }
    
    uint16_t crc = calculate_crc(message, length - 3);
    uint16_t received_crc = (message[length - 3] << 8) | message[length - 2];
    if (crc != received_crc) {
        return -1; // CRC mismatch
    }
    
    // Decode the message
    int decoded_length = decode
