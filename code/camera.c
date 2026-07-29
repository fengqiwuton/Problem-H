#include "camera.h"
#include "ball_balance.h"

/* Protocol: $B,<signed_int_mm>#
 * Example: $B,0# (center), $B,50# (+5cm), $B,-32# (-3.2cm)
 * mm value is in 0.1mm -> multiply by 10 to get ball_position_raw
 */

static uint8_t cam_rx_buf[32];
static uint8_t cam_rx_idx = 0;
static uint8_t cam_frame_start = 0;
static uint16_t cam_timeout_ms = 0;

void camera_init(void)
{
    uart_init(UART_3, 115200, 1);
    cam_rx_idx = 0;
    cam_frame_start = 0;
    cam_timeout_ms = 0;
    ball_position_valid = 0;
}

void camera_update_timeout(uint16_t dt_ms)
{
    if (cam_timeout_ms < 65535 - dt_ms)
    {
        cam_timeout_ms += dt_ms;
    }
    if (cam_timeout_ms > CAMERA_TIMEOUT_MS)
    {
        ball_position_valid = 0;
        cam_timeout_ms = CAMERA_TIMEOUT_MS;
    }
}

uint8_t camera_is_valid(void)
{
    return ball_position_valid;
}

/* Parse ball position from protocol: $B,<int># */
static void camera_parse_packet(void)
{
    int i;
    int sign = 1;
    int value = 0;
    int has_value = 0;

    /* Check minimum length: $B,0# is 5 chars */
    if (cam_rx_idx < 5) return;

    /* Check header: $B, */
    if (cam_rx_buf[0] != '$' || cam_rx_buf[1] != 'B' || cam_rx_buf[2] != ',')
        return;

    i = 3;

    /* Check sign */
    if (cam_rx_buf[i] == '-')
    {
        sign = -1;
        i++;
    }
    else if (cam_rx_buf[i] == '+')
    {
        i++;
    }

    /* Parse digits */
    while (i < cam_rx_idx && cam_rx_buf[i] >= '0' && cam_rx_buf[i] <= '9')
    {
        value = value * 10 + (cam_rx_buf[i] - '0');
        has_value = 1;
        i++;
    }

    /* Check end marker */
    if (has_value && i < cam_rx_idx && cam_rx_buf[i] == '#')
    {
        /* Convert mm to 0.1mm units */
        ball_position_raw = sign * value * 10;
        ball_position_valid = 1;
        cam_timeout_ms = 0;
    }
}

void camera_uart_rx(uint8_t data)
{
    if (data == '$')
    {
        cam_frame_start = 1;
        cam_rx_idx = 0;
        cam_rx_buf[cam_rx_idx++] = data;
        return;
    }

    if (!cam_frame_start) return;

    if (cam_rx_idx >= sizeof(cam_rx_buf))
    {
        cam_frame_start = 0;
        cam_rx_idx = 0;
        return;
    }

    cam_rx_buf[cam_rx_idx++] = data;

    if (data == '#')
    {
        camera_parse_packet();
        cam_frame_start = 0;
        cam_rx_idx = 0;
    }
}
