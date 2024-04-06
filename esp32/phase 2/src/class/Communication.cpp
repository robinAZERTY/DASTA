#include "Communication.hpp"

BL_types BL_stream::types;

String BL_stream::header()
{
    String header = "";
    uint8_t im = 0;
    uint32_t in = 0;
    for (uint8_t i = 0; i < this->length; i++)
    {
        String name = "";
        while (this->names[in] != '\0')
        {
            name += this->names[in];
            in++;
        }
        in++;
        header += name + ":" + String(this->data_type[i]) + ":" + String(this->data_size[i]);
        if (this->data_type[i] == types.MATRIX)
        {
            header += ":" + String(this->mat_col[im]);
            im++;
        }
        if (i != this->length - 1)
            header += ",";
    }
    return header;
}
int Communication::start()
{
    if (!SerialBT.begin(this->device_name))
    {
        Serial.println("An error occurred initializing Bluetooth");
        return -1;
    };
    Serial.println("Bluetooth available at " + String(SerialBT.getBtAddressString()));
    Serial.print("Waiting for connection");

    // wait for connection
    while (!SerialBT.connected(0))
    {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("Connected");

    // send headers
    if (this->send_header(&this->send_stream) < 0)
    {
        Serial.println("An error occurred sending the header of the send stream");
        return -2;
    }
    if (this->send_header(&this->receive_stream) < 0)
    {
        Serial.println("An error occurred sending the header of the receive stream");
        return -3;
    }
    return 1;
}
bool BL_stream::include(const char *name, uint8_t *data, char data_type, uint16_t data_size, bool stream)
{
    this->data = (uint8_t **)realloc(this->data, (this->length + 1) * sizeof(uint8_t *));
    this->data_type = (char *)realloc(this->data_type, (this->length + 1) * sizeof(char));
    this->data_size = (uint16_t *)realloc(this->data_size, (this->length + 1) * sizeof(uint16_t));
    this->names = (char *)realloc(this->names, (this->names_length + strlen(name) + 1) * sizeof(char));

    this->data[this->length] = data;
    this->data_type[this->length] = data_type;
    this->data_size[this->length] = data_size;
    this->total_data_size += data_size;
    // copy the name in the names array
    for (uint8_t i = 0; i < strlen(name); i++)
        this->names[this->names_length + i] = name[i];

    this->names_length += strlen(name) + 1;
    this->names[this->names_length - 1] = '\0';

    if (stream)
        this->_register |= 1 << this->length;
    else
        this->_register &= ~(1 << this->length);

    this->length++;

    if (this->length > 32)
        return false;

    return true;
}

bool BL_stream::include(const char *name, Vector &vec, bool stream)
{
    return this->include(name, (uint8_t *)vec.data, types.VECTOR, vec.size * sizeof(data_type), stream);
}

bool BL_stream::include(const char *name, Matrix &mat, bool stream)
{
    if (!this->include(name, (uint8_t *)mat.data, types.MATRIX, mat.size * sizeof(data_type), stream))
        return false;

    this->mat_col = (uint8_t *)realloc(this->mat_col, (this->mat_col_length + 1) * sizeof(uint8_t));
    this->mat_col[this->mat_col_length] = mat.rows;
    this->mat_col_length++;

    return true;
}

bool BL_stream::enable(const char *name)
{
    uint32_t in = 0;
    for (uint8_t i = 0; i < this->length; i++)
    {
        String name2 = "";
        while (this->names[in] != '\0')
        {
            name2 += this->names[in];
            in++;
        }
        in++;
        if (strcmp(name2.c_str(), name) == 0)
        {
            this->_register |= 1 << i;
        }
    }
    if (in == 0)
        return false; // name not found
    return true;
}

bool BL_stream::disable(const char *name)
{
    uint32_t in = 0;
    for (uint8_t i = 0; i < this->length; i++)
    {
        String name2 = "";
        while (this->names[in] != '\0')
        {
            name2 += this->names[in];
            in++;
        }
        in++;
        if (strcmp(name2.c_str(), name) == 0)
        {
            this->_register &= ~(1 << i);
        }
    }
    if (in == 0)
        return false; // name not found
    return true;
}

const char *BL_stream::get_name(uint8_t i)
{
    uint32_t in = 0;
    for (uint8_t j = 0; j < i; j++)
    {
        while (this->names[in] != '\0')
            in++;
        in++;
    }
    return this->names + in;

}

int Communication::send_header(BL_stream *stream)
{
    // write the description key
    String to_send = stream->name + stream->header() + stream->end_line;
    if (SerialBT.print(to_send) != to_send.length())
        return -1;

    return 1; // success
}

int Communication::send()
{   
    if (!this->running_send_stream)
        return 0;
        
    // write the stream_register to the serial
    uint32_t rr = this->send_stream._register;
    if (SerialBT.write((uint8_t *)&rr, sizeof(rr)) != sizeof(rr))
        return -1;
    // write the data to the serial
    for (uint8_t i = 0; i < this->send_stream.get_length(); i++)
        if (rr & (1 << i))
            if (SerialBT.write(this->send_stream.get_data()[i], this->send_stream.get_data_size()[i]) != this->send_stream.get_data_size()[i])
                return -2;

    // write the end line
    if (SerialBT.print(send_stream.end_line) != sizeof(send_stream.end_line) - 1)
        return -3;

    return 1; // success
}
#include "StreamString.h"

int Communication::receive()
{
    if (!receive_stream_register_already_received)
    {
        if (SerialBT.available() <= sizeof(receive_stream._register))
            return 0;

        // read the stream_register
        if (SerialBT.readBytes((uint8_t *)&receive_stream._register, sizeof(receive_stream._register)) != sizeof(receive_stream._register))
            return -1; // error while reading the stream_register

        receive_stream_expected_length = receive_stream.end_line.length();
        for (uint8_t i = 0; i < receive_stream.get_length(); i++)
            if (receive_stream._register & (1 << i))
                receive_stream_expected_length += receive_stream.get_data_size()[i];

        receive_stream_register_already_received = true;
    }

    if (SerialBT.available() < receive_stream_expected_length)
        return 0; // not enough data to read

    receive_stream_register_already_received = false;

    // read the data
    for (uint8_t i = 0; i < receive_stream.get_length(); i++)
        if (receive_stream._register & (1 << i))
            if (SerialBT.readBytes(receive_stream.get_data()[i], receive_stream.get_data_size()[i]) != receive_stream.get_data_size()[i])
                return -3; // error while reading the data

    // read the end line
    for (uint8_t i = 0; i < receive_stream_expected_length; i++)
        if (SerialBT.read() != receive_stream.end_line[i])
            return -4; // error while reading the end line

    return 1; // success (all data read)
}


