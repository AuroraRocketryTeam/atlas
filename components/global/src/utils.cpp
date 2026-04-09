#include "utils.h"
#include "driver/usb_serial_jtag.h"

int Utils::readLine(char* buf, int maxLen) {
    int pos = 0;
    while (pos < maxLen - 1) {
        uint8_t c = 0;
        int n = usb_serial_jtag_read_bytes(&c, 1, portMAX_DELAY);
        if (n <= 0) continue;
        if (c == '\r' || c == '\n') {
            if (pos > 0) {
                usb_serial_jtag_write_bytes((const uint8_t*)"\r\n", 2, 20 / portTICK_PERIOD_MS);
                break;
            }
            continue; // skip leading newlines
        }
        usb_serial_jtag_write_bytes(&c, 1, 20 / portTICK_PERIOD_MS); // echo
        buf[pos++] = (char)c;
    }
    buf[pos] = '\0';
    return pos;
}
