#include "maze_store.h"

#include <string.h>

bool maze_blob_valid(const maze_t *saved, const maze_t *expected_shape)
{
    return saved->w == expected_shape->w &&
           saved->h == expected_shape->h &&
           saved->goal_count == expected_shape->goal_count &&
           memcmp(saved->goals, expected_shape->goals,
                  expected_shape->goal_count * sizeof(saved->goals[0])) == 0;
}

#ifndef MAZE_STORE_HOST

#include "nvs.h"

#define STORE_NAMESPACE "maze"
#define STORE_KEY       "map"
#define STORE_MAGIC     0x4D5A3031U   /* "MZ01" */
#define STORE_VERSION   1

typedef struct {
    uint32_t magic;
    uint16_t version;
    maze_t   maze;
} maze_blob_t;

static maze_blob_t s_blob;      /* app task only; kept off its stack */
static maze_t      s_expected;

esp_err_t maze_store_save(const maze_t *m)
{
    nvs_handle_t h;
    esp_err_t    err = nvs_open(STORE_NAMESPACE, NVS_READWRITE, &h);

    if (err != ESP_OK) {
        return err;
    }
    s_blob.magic   = STORE_MAGIC;
    s_blob.version = STORE_VERSION;
    s_blob.maze    = *m;

    err = nvs_set_blob(h, STORE_KEY, &s_blob, sizeof(s_blob));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

static esp_err_t read_blob(void)
{
    nvs_handle_t h;
    size_t       len = sizeof(s_blob);
    esp_err_t    err = nvs_open(STORE_NAMESPACE, NVS_READONLY, &h);

    if (err == ESP_OK) {
        err = nvs_get_blob(h, STORE_KEY, &s_blob, &len);
        nvs_close(h);
    }
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }
    if (err == ESP_ERR_NVS_INVALID_LENGTH || (err == ESP_OK && len != sizeof(s_blob))) {
        return ESP_ERR_INVALID_VERSION;
    }
    return err;
}

esp_err_t maze_store_load(maze_t *m)
{
    esp_err_t err = read_blob();

    if (err != ESP_OK) {
        return err;
    }
    maze_init_configured(&s_expected);
    if (s_blob.magic != STORE_MAGIC || s_blob.version != STORE_VERSION ||
        !maze_blob_valid(&s_blob.maze, &s_expected)) {
        return ESP_ERR_INVALID_VERSION;
    }
    *m = s_blob.maze;
    return ESP_OK;
}

#endif
