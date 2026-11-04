#include "app_ipc.h"

#include "freertos/queue.h"

#define SENSOR_QUEUE_LEN 1
#define MOTION_QUEUE_LEN 4
#define STATUS_QUEUE_LEN 1

static QueueHandle_t      s_sensor_q;
static QueueHandle_t      s_motion_q;
static QueueHandle_t      s_status_q;
static EventGroupHandle_t s_events;

static void app_ipc_free(void)
{
    if (s_sensor_q) {
        vQueueDelete(s_sensor_q);
        s_sensor_q = NULL;
    }
    if (s_motion_q) {
        vQueueDelete(s_motion_q);
        s_motion_q = NULL;
    }
    if (s_status_q) {
        vQueueDelete(s_status_q);
        s_status_q = NULL;
    }
    if (s_events) {
        vEventGroupDelete(s_events);
        s_events = NULL;
    }
}

esp_err_t app_ipc_init(void)
{
    if (s_sensor_q && s_motion_q && s_status_q && s_events) {
        return ESP_OK;
    }

    s_sensor_q = xQueueCreate(SENSOR_QUEUE_LEN, sizeof(sensor_snapshot_t));
    s_motion_q = xQueueCreate(MOTION_QUEUE_LEN, sizeof(motion_cmd_t));
    s_status_q = xQueueCreate(STATUS_QUEUE_LEN, sizeof(motion_status_t));
    s_events   = xEventGroupCreate();

    if (!s_sensor_q || !s_motion_q || !s_status_q || !s_events) {
        app_ipc_free();
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

bool app_ipc_publish_sensor(const sensor_snapshot_t *snapshot)
{
    if (!s_sensor_q || snapshot == NULL) {
        return false;
    }
    return xQueueOverwrite(s_sensor_q, snapshot) == pdPASS;
}

bool app_ipc_read_sensor(sensor_snapshot_t *snapshot)
{
    if (!s_sensor_q || snapshot == NULL) {
        return false;
    }
    return xQueuePeek(s_sensor_q, snapshot, 0) == pdTRUE;
}

bool app_ipc_publish_status(const motion_status_t *status)
{
    if (!s_status_q || status == NULL) {
        return false;
    }
    return xQueueOverwrite(s_status_q, status) == pdPASS;
}

bool app_ipc_read_status(motion_status_t *status)
{
    if (!s_status_q || status == NULL) {
        return false;
    }
    return xQueuePeek(s_status_q, status, 0) == pdTRUE;
}

bool app_ipc_send_motion(motion_cmd_t cmd, TickType_t wait_ticks)
{
    if (!s_motion_q) {
        return false;
    }
    return xQueueSend(s_motion_q, &cmd, wait_ticks) == pdPASS;
}

bool app_ipc_recv_motion(motion_cmd_t *cmd, TickType_t wait_ticks)
{
    if (!s_motion_q || cmd == NULL) {
        return false;
    }
    return xQueueReceive(s_motion_q, cmd, wait_ticks) == pdTRUE;
}

void app_ipc_set_event(EventBits_t bits)
{
    if (s_events) {
        xEventGroupSetBits(s_events, bits);
    }
}

void app_ipc_clear_event(EventBits_t bits)
{
    if (s_events) {
        xEventGroupClearBits(s_events, bits);
    }
}

EventBits_t app_ipc_wait_event(EventBits_t bits, bool clear_on_exit, TickType_t wait_ticks)
{
    if (!s_events) {
        return 0;
    }

    return xEventGroupWaitBits(s_events, bits,
                               clear_on_exit ? pdTRUE : pdFALSE,
                               pdFALSE, wait_ticks);
}
