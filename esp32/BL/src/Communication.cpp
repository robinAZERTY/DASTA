#include "Communication.hpp"

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
        if (this->data_type[i] == MATRIX_KEY)
        {
            header += ":" + String(this->mat_col[im]);
            im++;
        }
        if (i != this->length - 1)
            header += ",";
    }
    return header;
}

bool BL_stream::include(const char *name, uint8_t *data, char data_type, uint8_t data_size, bool stream)
{
    this->data = (uint8_t **)realloc(this->data, (this->length + 1) * sizeof(uint8_t *));
    this->data_type = (char *)realloc(this->data_type, (this->length + 1) * sizeof(char));
    this->data_size = (uint8_t *)realloc(this->data_size, (this->length + 1) * sizeof(uint8_t));
    // this->names = (String**) realloc(this->names, (this->length+1)*sizeof(String*));
    this->names = (char *)realloc(this->names, (this->names_length + strlen(name) + 1) * sizeof(char));

    this->data[this->length] = data;
    this->data_type[this->length] = data_type;
    this->data_size[this->length] = data_size;
    // copy the name in the names array
    for (uint8_t i = 0; i < strlen(name); i++)
    {
        this->names[this->names_length + i] = name[i];
    }
    this->names_length += strlen(name) + 1;
    this->names[this->names_length - 1] = '\0';

    if (stream)
        this->_register |= 1 << this->length;

    this->length++;

    if (this->length > 32)
        return false;

    return true;
}

bool BL_stream::include(const char *name, Vector &vec)
{
    return this->include(name, (uint8_t *)vec.data, VECTOR_KEY, vec.size * sizeof(data_type));
}

bool BL_stream::include(const char *name, Matrix &mat)
{
    if (!this->include(name, (uint8_t *)mat.data, MATRIX_KEY, mat.size * sizeof(data_type)))
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

int Communication::send_header(BL_stream *stream)
{
    // write the description key
    String to_send = stream->name + stream->header() + END_LINE_KEY;
    if (SerialBT.print(to_send) != to_send.length())
        return -1;

    return 1; // success
}

int Communication::send()
{
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
    if (SerialBT.print(END_LINE_KEY) != sizeof(END_LINE_KEY) - 1)
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

        receive_stream_expected_length = strlen(END_LINE_KEY);
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
    for (uint8_t i = 0; i < strlen(END_LINE_KEY); i++)
        if (SerialBT.read() != END_LINE_KEY[i])
            return -4; // error while reading the end line

    return 1; // success
}
