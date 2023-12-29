#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP
#include "Matrix.hpp"
#include "BluetoothSerial.h"


#define TYPE_CHAR 'c'
#define TYPE_INT 'i'
#define TYPE_LONG_LONG 'L'
#define TYPE_FLOAT 'f'
#define TYPE_DOUBLE 'd'
#define TYPE_STRING 's'
#define TYPE_VECTOR 'v'
#define TYPE_MATRIX 'm'


class Communication {
    private:
        BluetoothSerial SerialBT;
        uint8_t sending_structure = 0; // describes what to include in the stream with binary flags
        const uint8_t s1v = sizeof(data_type); // size of data_type in a vector
        Vector *X; // state vector
        Matrix *P; // covariance matrix
        Vector *Z; // measurement vector
        Vector *h; // measurement vector
        String *internal_events; // internal events


    public:
        Communication();
        ~Communication();

        bool send_description = true; // description
        bool send_t = false; // time
        bool send_X = false; // state vector
        bool send_P = false; // covariance matrix
        bool send_Z = false; // measurement vector
        bool send_h = false; // measurement vector
        bool send_internal_events = false; // internal events

        void set_X_ptr(Vector *X);
        void set_P_ptr(Matrix *P);
        void set_Z_ptr(Vector *Z);
        void set_h_ptr(Vector *h);
        void set_internal_events_ptr(String *internal_events);

        String describe()
        {
            String description = "!";
            //every data is described by a name, a type and a size
            description += "t" + TYPE_LONG_LONG+";";
            description += "X" + TYPE_VECTOR + String(X->size()) + "*"+String(s1v)+";";
            description += "P" + TYPE_MATRIX + String(P->rows()) + "*" + String(P->cols()) + "*"+String(s1v)+";";
            description += "Z" + TYPE_VECTOR + String(Z->size()) + "*"+String(s1v)+";";
            description += "h" + TYPE_MATRIX + String(h->size()) + "*"+String(s1v)+";";
            description += "e" + TYPE_STRING;
            return description;
        }

        bool send(unsigned long long t)
        {
            //verify if the connection is still alive
            if (!SerialBT.connected(0)) return false;

            // check if the ptrs are set correctly
            if (send_X && X == nullptr) return false;
            if (send_P && P == nullptr) return false;
            if (send_Z && Z == nullptr) return false;
            if (send_h && h == nullptr) return false;
            if (send_internal_events && internal_events == nullptr) return false;

            
            //compute the sending_structure
            sending_structure = 0;
            if (send_t) sending_structure |= 1 << 0;
            if (send_X) sending_structure |= 1 << 1;    
            if (send_P) sending_structure |= 1 << 2;
            if (send_Z) sending_structure |= 1 << 3;
            if (send_h) sending_structure |= 1 << 4;
            if (send_internal_events) sending_structure |= 1 << 5;

            //send the data
            if (SerialBT.write((uint8_t*)&sending_structure, sizeof(sending_structure)) != sizeof(sending_structure)) return false;
            if (send_description) {
                String description = describe();
                if (SerialBT.write((uint8_t*)description.c_str(), description.length()) != description.length()) return false;
                send_description = false;
            }
            if (send_t) if (SerialBT.write((uint8_t*)&t, sizeof(t)) != sizeof(t)) return false;
            if (send_X) if (SerialBT.write((uint8_t*)X->data(), X->size()*s1v) != X->size()*s1v) return false;
            if (send_P) if (SerialBT.write((uint8_t*)P->data(), P->rows()*P->cols()*s1v) != P->rows()*P->cols()*s1v) return false;
            if (send_Z) if (SerialBT.write((uint8_t*)Z->data(), Z->size()*s1v) != Z->size()*s1v) return false;
            if (send_h) if (SerialBT.write((uint8_t*)h->data(), h->size()*s1v) != h->size()*s1v) return false;
            if (send_internal_events) if (SerialBT.write((uint8_t*)internal_events->c_str(), internal_events->length()) != internal_events->length()) return false;
            return true;
        }    
};

#endif // COMMUNICATION_HPP