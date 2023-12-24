#include "Communication.hpp"




void Communication::set_X_ptr(Vector *X)
{
    this->X = X;
}

void Communication::set_P_ptr(Matrix *P)
{
    this->P = P;
}

void Communication::set_Z_ptr(Vector *Z)
{
    this->Z = Z;
}

void Communication::set_h_ptr(Vector *h)
{
    this->h = h;
}

void Communication::set_internal_events_ptr(String *internal_events)
{
    this->internal_events = internal_events;
}

String Communication::describe()
{
    String description = DESCRIPTION_KEY;
    //every data is described by a name, a type and a size
    description += "t" + String(TYPE_LONG_LONG) + String(sizeof(unsigned long long)) + ";";
    description += "X" + String(TYPE_VECTOR);
    if (X == nullptr) description += "?;";
    else description += String(X->size)+"*"+String(s1v)+";";
    description += "P" + String(TYPE_MATRIX);
    if (P == nullptr) description += "?;";
    else description += String(P->rows)+"*"+String(P->cols)+"*"+String(s1v)+";";
    description += "Z" + String(TYPE_VECTOR);
    if (Z == nullptr) description += "?;";
    else description += String(Z->size)+"*"+String(s1v)+";";
    description += "h" + String(TYPE_VECTOR);
    if (h == nullptr) description += "?;";
    else description += String(h->size)+"*"+String(s1v)+";";
    description += "e" + String(TYPE_STRING);

    return description;
}

bool Communication::send(unsigned long long t){
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
    if (send_description) {
        String description = describe()+END_LINE;
        if (SerialBT.print(description) != description.length()) return false;
        send_description = false;
    }
    if (SerialBT.write(sending_structure) != 1) return false;
    if (send_t) if (SerialBT.write((uint8_t*)&t, sizeof(t)) != sizeof(t)) return false;
    if (send_X) if (SerialBT.write((uint8_t*)X->data, X->size*s1v) != X->size*s1v) return false;
    if (send_P) if (SerialBT.write((uint8_t*)P->data, P->rows*P->cols*s1v) != P->rows*P->cols*s1v) return false;
    if (send_Z) if (SerialBT.write((uint8_t*)Z->data, Z->size*s1v) != Z->size*s1v) return false;
    if (send_h) if (SerialBT.write((uint8_t*)h->data, h->size*s1v) != h->size*s1v) return false;
    if (send_internal_events) if (SerialBT.write((uint8_t*)internal_events->c_str(), internal_events->length()) != internal_events->length()) return false;
    if (SerialBT.print(END_LINE) != sizeof(END_LINE)-1) return false;
    return true;
}