#include <windows.h>
#include <iostream>
#include <deque>

using namespace std;

// APP <-> COMA <-> COMB <-> logger <-> COMC <-> COMD
// terminal <- COM4, COM5 -> logger <- COM3, COM6 -> terminal


const DWORD UART_BUFFER_SIZE = 1024;
const DWORD MSG_BUFFER_SIZE = 16;



#pragma pack(push, 1)
struct LogEntryHeader {
    uint64_t time_stamp;
    uint8_t source;
    uint16_t length;
    LogEntryHeader(uint64_t time_stamp, uint8_t source, uint16_t length) {
        this->time_stamp = time_stamp;
        this->source = source;
        this->length = length;
    }
};
#pragma pack(pop)

deque<LogEntryHeader> tx_headers = deque<LogEntryHeader>();

VOID WINAPI FileWrittenCallback(DWORD dwErrorCode, DWORD dwBytesTransferred, LPOVERLAPPED lpOverlapped) {
    if (dwErrorCode != 0) {
        fprintf(stdout, "CompletionRoutine: Unable to write to file! Error: %u, AddrOverlapped: %p\n", dwErrorCode, lpOverlapped);
    }
    else {
        tx_headers.pop_front();
    }
}


//Returns the last Win32 error, in string format. Returns an empty string if there is no error.
std::string GetLastErrorAsString()
{
    //Get the error message, if any.
    DWORD errorMessageID = ::GetLastError();
    if (errorMessageID == 0)
        return std::string(); //No error message has been recorded

    LPSTR messageBuffer = nullptr;
    size_t size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL, errorMessageID, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&messageBuffer, 0, NULL);

    std::string message(messageBuffer, size);

    //Free the buffer.
    LocalFree(messageBuffer);

    return message;
}

struct SerialPortState {
    BYTE msg_buffer[MSG_BUFFER_SIZE];
    DWORD buffer_head = 0;
    DWORD buffer_tail = 0;
    OVERLAPPED rx_overlapped = { 0 };
    OVERLAPPED tx_overlapped = { 0 };
    BOOL rx_read = true;
    HANDLE serial_handle;

private:
    const BYTE STOP_BITS = ONESTOPBIT;
    const BYTE PARITY = NOPARITY;
    const BYTE BYTE_SIZE = 8;

public:
    SerialPortState(wstring port_name, DWORD baudrate, DWORD time_out_millis) {
        serial_handle = CreateFile(port_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, 0);
        rx_overlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
        SetupComm(serial_handle, UART_BUFFER_SIZE, UART_BUFFER_SIZE);

        

        // Do some basic settings
        DCB serialParams = { 0 };
        BOOL status;
        serialParams.DCBlength = sizeof(serialParams);

        status = GetCommState(serial_handle, &serialParams);
        if (!status) {
            fprintf(stdout, "Could not GetCommState! Error %u: %s\n", GetLastError(), GetLastErrorAsString().c_str());
            exit(1);
        }
        serialParams.BaudRate = baudrate;
        serialParams.ByteSize = BYTE_SIZE;
        serialParams.StopBits = STOP_BITS;
        serialParams.Parity = PARITY;
        status = SetCommState(serial_handle, &serialParams);
        if (!status) {
            fprintf(stdout, "Could not SetCommState! Error %u: %s\n", GetLastError(), GetLastErrorAsString().c_str());
            exit(1);
        }

        // Set timeouts
        COMMTIMEOUTS timeout = { 0 };
        timeout.ReadIntervalTimeout = time_out_millis;
        timeout.ReadTotalTimeoutConstant = time_out_millis;
        timeout.ReadTotalTimeoutMultiplier = time_out_millis;
        timeout.WriteTotalTimeoutConstant = time_out_millis;
        timeout.WriteTotalTimeoutMultiplier = time_out_millis;

        SetCommTimeouts(serial_handle, &timeout);

        COMSTAT com_stat;
        DWORD error_flags;

        ClearCommError(serial_handle, &error_flags, &com_stat);
        PurgeComm(serial_handle, PURGE_RXCLEAR | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_TXABORT);
    }
};


int main(int argc, char** argv)
{

    cout << "Hello World!" << endl;


    LPCWSTR port1 = L"COM3";
    LPCWSTR port2 = L"COM5";
    LPCWSTR out_file = L"out.bin";
    DWORD baudrate = 9600;
    int read_size = 1;
    DWORD time_out_millis = 0;
    BYTE seperator = '\n';

    HANDLE log_file = CreateFile(out_file, FILE_APPEND_DATA, NULL, NULL, CREATE_ALWAYS, FILE_FLAG_OVERLAPPED, NULL);
    OVERLAPPED log_write_overlapped = { 0xFFFFFFFF, 0xFFFFFFFF, 0 };
    //HANDLE log_file = CreateFile(out_file, GENERIC_READ | GENERIC_WRITE, NULL, NULL, CREATE_ALWAYS, NULL, NULL);
    if (INVALID_HANDLE_VALUE == log_file) {
        fprintf(stdout, "Could not create file! Error %u: %s\n", GetLastError(), GetLastErrorAsString().c_str());
        return -1;
    }

    // Open serial port
    SerialPortState port_states[] = { SerialPortState(port1, baudrate, time_out_millis),
                                      SerialPortState(port2, baudrate, time_out_millis) };


    const HANDLE read_handles[] = { port_states[0].rx_overlapped.hEvent,
                                    port_states[1].rx_overlapped.hEvent};

    LARGE_INTEGER StartingTime, time_stamp, elapsed_microseconds;
    LARGE_INTEGER Frequency;

    QueryPerformanceFrequency(&Frequency);
    QueryPerformanceCounter(&StartingTime);

    while (true) {

        for (int i = 0; i < 2; i++) {
            SerialPortState* state = port_states + i;
            if (state->rx_read) {
                ReadFile(state->serial_handle, // Serial port handle
                    // The address of the data storage read in,
                    // The data read in will be stored in a memory area with the value of the pointer
                    state->msg_buffer + state->buffer_head,
                    // The number of bytes of data to read
                    read_size,
                    // points to a DWORD value, which returns the number of bytes actually read by the read synchronous reads
                    NULL,
                    // In the overlap operation, this parameter points to an OVERLAPPED structure. When the operation is synchronized, this parameter is NULL.
                    &state->rx_overlapped);
            }
        }

        // Wait for a read on either port, or a timeout
        DWORD trigger;
        trigger = WaitForMultipleObjects(2, read_handles, false, INFINITE);

        QueryPerformanceCounter(&time_stamp);
        elapsed_microseconds.QuadPart = time_stamp.QuadPart - StartingTime.QuadPart;
        //
        // We now have the elapsed number of ticks, along with the
        // number of ticks-per-second. We use these values
        // to convert to the number of elapsed microseconds.
        // To guard against loss-of-precision, we convert
        // to microseconds *before* dividing by ticks-per-second.
        //
        elapsed_microseconds.QuadPart *= 1000000;
        elapsed_microseconds.QuadPart /= Frequency.QuadPart;


        if (trigger != WAIT_OBJECT_0 && trigger != WAIT_OBJECT_0 + 1) {
            fprintf(stdout, "Wait for serial failed! Error %u: %s\n", GetLastError(), GetLastErrorAsString().c_str());
            return -1;
        }

        trigger -= WAIT_OBJECT_0;
        DWORD target = (trigger + 1) % 2;

        bool overlapped_status;
        DWORD bytes_transfered;

        overlapped_status = GetOverlappedResult(
            port_states[trigger].serial_handle,
            &port_states[trigger].rx_overlapped,
            &bytes_transfered,
            false
        );

        SerialPortState* state = port_states + trigger;
        state->rx_read = true;

        if (bytes_transfered > 0) {
            int msg_end_idx = -1;
            {
                //FIND MESSAGE SEPERATION CHAR
                BYTE* rx_buffer = state->msg_buffer + state->buffer_head;
                for (DWORD i = 0; i < bytes_transfered; i++) {
                    if (rx_buffer[i] == seperator) {
                        msg_end_idx = i;
                        break;
                    }
                }
            }
            if (msg_end_idx >= 0) {

                BYTE* tx_buffer = state->msg_buffer + state->buffer_tail;
                DWORD message_size = state->buffer_head - state->buffer_tail + msg_end_idx + 1;

                HANDLE target_handle = port_states[target].serial_handle;

                WriteFile(target_handle, tx_buffer, message_size, NULL, &state->tx_overlapped);
                
                tx_headers.push_back(LogEntryHeader(elapsed_microseconds.QuadPart, trigger, message_size));
                // NOTE: Is &tx_headers.back() safe?
                WriteFileEx(log_file, &tx_headers.back(), sizeof(LogEntryHeader), &log_write_overlapped, (LPOVERLAPPED_COMPLETION_ROUTINE)FileWrittenCallback);
                WriteFile(log_file, tx_buffer, message_size, NULL, &log_write_overlapped);

                /*LogEntryHeader header = LogEntryHeader(elapsed_microseconds.QuadPart, trigger, message_size);
                WriteFile(log_file, &header, sizeof(LogEntryHeader), NULL, NULL);
                WriteFile(log_file, tx_buffer, message_size, NULL, NULL);*/

                state->buffer_tail += message_size;
            }

            state->buffer_head += bytes_transfered;

            if (state->buffer_head + read_size > MSG_BUFFER_SIZE) {
                DWORD cur_message_size = state->buffer_head - state->buffer_tail;
                BYTE* cur_message_buffer = state->msg_buffer + state->buffer_tail;

                if (cur_message_size > state->buffer_tail) {
                    fprintf(stdout, "Buffer overrun!");
                    return -1;
                }

                memcpy(state->msg_buffer, cur_message_buffer, cur_message_size);

                state->buffer_head = cur_message_size;
                state->buffer_tail = 0;
            }
        }
        else if (GetLastError() != ERROR_TIMEOUT) {
            cout << GetLastErrorAsString() << endl;
        }
    }
}
