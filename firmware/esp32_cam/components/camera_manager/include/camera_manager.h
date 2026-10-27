#ifndef CAMERA_MANAGER_H
#define CAMERA_MANAGER_H

#include "esp_camera.h"
#include "esp_err.h"

esp_err_t camera_manager_init(void);

camera_fb_t *camera_manager_capture(void);

void camera_manager_release(camera_fb_t *frame);

#endif