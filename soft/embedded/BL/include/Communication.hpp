/*
*/



#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP
#include "Matrix.hpp"
#include "BluetoothSerial.h"

//types keys
#define CHAR_KEY 'c'
#define INT_KEY 'i'
#define UNSIGNED_LONG_LONG_KEY 'Q'
#define FLOAT_KEY 'f'
#define DOUBLE_KEY 'd'
#define VECTOR_KEY 'v'
#define MATRIX_KEY 'm'

#define SEND_STREAM_NAME "send_stream:"
#define RECEIVE_STREAM_NAME "receive_stream:"
#define END_LINE_KEY "end_line\n"

struct BL_stream
{
    private:
        uint8_t **data=NULL;
        char *data_type=NULL;
        char *names=NULL;
        uint32_t names_length=0;
        uint8_t *mat_col = NULL;
        uint8_t mat_col_length = 0;
        uint8_t *data_size=NULL;
        uint8_t length=0;
    public:
        String name="default";
        uint32_t _register=0; // describes what to include in the stream with binary flags
        uint8_t get_length(){return this->length;};
        uint8_t **get_data(){return this->data;};
        uint8_t *get_data_size(){return this->data_size;};
        bool include(const char *name, uint8_t *data, char data_type, uint8_t data_size, bool stream=true);
        bool include(const char *name, Vector &vec);
        bool include(const char *name, Matrix &mat);
        bool enable(const char *name);
        bool disable(const char *name);
        String header();
};


class Communication {
    private :
        bool receive_stream_register_already_received=false;
        uint32_t receive_stream_expected_length = 0;
    public:
        BL_stream send_stream, receive_stream;
        BluetoothSerial SerialBT;
        Communication(){send_stream.name = SEND_STREAM_NAME;receive_stream.name = RECEIVE_STREAM_NAME;SerialBT.setTimeout(10);};
        ~Communication(){};// TODO : free data
        int send_header(BL_stream *stream);
        int receive();
        int send();
};

#endif // COMMUNICATION_HPP