#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP
#include "Matrix.hpp"
#include "BluetoothSerial.h"


struct BL_stream
{
private:
    uint8_t **data = NULL;
    char *data_type = NULL;
    char *names = NULL;
    uint32_t names_length = 0;
    uint8_t *mat_col = NULL;
    uint8_t mat_col_length = 0;
    uint16_t *data_size = NULL;
    uint8_t length = 0;
    uint32_t total_data_size = 0;

public:
    String end_line = "\n";
    String name = "default";
    uint32_t _register = 0; // describes what to include in the stream with binary flags
    uint8_t get_length() { return this->length; };
    uint8_t **get_data() { return this->data; };
    uint16_t *get_data_size() { return this->data_size; };
    uint32_t get_total_data_size() { return this->total_data_size; };
    bool include(const char *name, uint8_t *data, char data_type, uint16_t data_size, bool stream = true);
    bool include(const char *name, Vector &vec, bool stream = true);
    bool include(const char *name, Matrix &mat, bool stream = true);
    bool enable(const char *name);
    bool disable(const char *name);
    String header();
};

class Communication
{
private:
    bool receive_stream_register_already_received = false;
    uint32_t receive_stream_expected_length = 0;

public:
    bool running_send_stream = false;
    String device_name = "ESP32-Bluetooth";
    BL_stream send_stream, receive_stream;
    BluetoothSerial SerialBT;
    Communication();
    ~Communication(){}; // TODO : free data
    int start();
    int send_header(BL_stream *stream);
    int receive();
    int send();
};


#endif // COMMUNICATION_HPP