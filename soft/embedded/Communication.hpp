#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP
#include "Matrix.hpp"
#include "BluetoothSerial.h"


#define TYPE_CHAR "c"
#define TYPE_INT "i"
#define TYPE_UNSIGNED_LONG_LONG "Q"
#define TYPE_FLOAT "f"
#define TYPE_DOUBLE "d"
#define TYPE_STRING "s"
#define TYPE_VECTOR "v"
#define TYPE_MATRIX "m"

#define DESCRIPTION_KEY "description:"

#define END_LINE "end_line\n"


class Communication {
    public:
        
        uint8_t sending_structure = 0; // describes what to include in the stream with binary flags
        const uint8_t s1v = sizeof(data_type); // size of data_type in a vector
        Vector *X; // state vector
        Matrix *P; // covariance matrix
        Vector *Z; // measurement vector
        Vector *h; // measurement vector
        String *internal_events; // internal events


    public:
        
        BluetoothSerial SerialBT;

        Communication(){};
        ~Communication(){};


        bool send_description = true; // description
        bool send_t = false; // time
        bool send_X = false; // state vector
        bool send_P = false; // covariance matrix
        bool send_Z = false; // measurement vector
        bool send_h = false; // predicted measurement vector
        bool send_internal_events = false; // internal events

        void set_X_ptr(Vector *X);
        void set_P_ptr(Matrix *P);
        void set_Z_ptr(Vector *Z);
        void set_h_ptr(Vector *h);
        void set_internal_events_ptr(String *internal_events);

        String describe();

        bool send(unsigned long long t);
};

#endif // COMMUNICATION_HPP