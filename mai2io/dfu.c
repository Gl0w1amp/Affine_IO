#include "dfu_loader.h"

#include <windows.h>
#include <libusb.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#define STM32_VENDOR_ID   0x0483
#define STM32_PRODUCT_ID  0xDF11
#define STM32_BASE_ADDRESS 0x08000000u
#define STM32_DEFAULT_TRANSFER 2048u

#define DFU_REQUEST_DNLOAD   0x01
#define DFU_REQUEST_UPLOAD   0x02
#define DFU_REQUEST_GETSTATUS 0x03
#define DFU_REQUEST_CLRSTATUS 0x04
#define DFU_REQUEST_GETSTATE 0x05
#define DFU_REQUEST_ABORT    0x06

#define DFU_STATE_appIDLE                 0
#define DFU_STATE_appDETACH               1
#define DFU_STATE_dfuIDLE                 2
#define DFU_STATE_dfuDNLOAD_SYNC          3
#define DFU_STATE_dfuDNBUSY               4
#define DFU_STATE_dfuDNLOAD_IDLE          5
#define DFU_STATE_dfuMANIFEST_SYNC        6
#define DFU_STATE_dfuMANIFEST             7
#define DFU_STATE_dfuMANIFEST_WAIT_RESET  8
#define DFU_STATE_dfuUPLOAD_IDLE          9
#define DFU_STATE_dfuERROR                10

typedef struct
{
    libusb_device_handle *handle;
    uint8_t interface_number;
    uint8_t alt_setting;
    uint16_t transfer_size;
    char alt_name[128];
} stm32_dfu_device_t;

typedef struct
{
    uint8_t status;
    uint8_t state;
    uint32_t poll_timeout_ms;
} dfu_status_t;

static void set_status(char *buffer, size_t buffer_len, const char *fmt, ...)
{
    if (!buffer || buffer_len == 0) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    _vsnprintf_s(buffer, buffer_len, _TRUNCATE, fmt, ap);
    va_end(ap);
}

static void report_progress(dfu_loader_progress_cb cb, void *ctx, int percent)
{
    if (cb) {
        cb(percent, ctx);
    }
}

static bool load_firmware_file(const wchar_t *path,
                               unsigned char **out_data,
                               size_t *out_len,
                               char *status_buf,
                               size_t status_len)
{
    if (!path || !out_data || !out_len) {
        set_status(status_buf, status_len, "错误: 固件路径无效");
        return false;
    }

    FILE *fp = NULL;
    if (_wfopen_s(&fp, path, L"rb") != 0 || !fp) {
        set_status(status_buf, status_len, "错误: 无法打开固件文件");
        return false;
    }

    if (fseek(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        set_status(status_buf, status_len, "错误: 无法读取固件文件大小");
        return false;
    }
    long file_size = ftell(fp);
    if (file_size <= 0) {
        fclose(fp);
        set_status(status_buf, status_len, "错误: 固件文件为空");
        return false;
    }
    rewind(fp);

    unsigned char *data = (unsigned char *)malloc((size_t)file_size);
    if (!data) {
        fclose(fp);
        set_status(status_buf, status_len, "错误: 内存不足，无法加载固件");
        return false;
    }

    size_t read = fread(data, 1, (size_t)file_size, fp);
    fclose(fp);

    if (read != (size_t)file_size) {
        free(data);
        set_status(status_buf, status_len, "错误: 读取固件文件失败");
        return false;
    }

    *out_data = data;
    *out_len = (size_t)file_size;
    return true;
}

static uint16_t parse_transfer_size(const struct libusb_interface_descriptor *alt)
{
    if (!alt || alt->extra_length < 9) {
        return STM32_DEFAULT_TRANSFER;
    }

    const unsigned char *cursor = alt->extra;
    int remaining = alt->extra_length;
    while (remaining >= 3) {
        uint8_t length = cursor[0];
        uint8_t type = cursor[1];
        if (length < 3 || length > remaining) {
            break;
        }
        if (type == 0x21 && length >= 9) {
            uint16_t transfer = cursor[5] | (cursor[6] << 8);
            if (transfer == 0) {
                transfer = STM32_DEFAULT_TRANSFER;
            }
            return transfer;
        }
        cursor += length;
        remaining -= length;
    }
    return STM32_DEFAULT_TRANSFER;
}

static bool select_dfu_interface(libusb_device *device,
                                 libusb_device_handle *handle,
                                 stm32_dfu_device_t *out_device)
{
    struct libusb_config_descriptor *config = NULL;
    int ret = libusb_get_active_config_descriptor(device, &config);
    if (ret != LIBUSB_SUCCESS || !config) {
        return false;
    }

    bool found = false;
    stm32_dfu_device_t local = {0};

    for (int i = 0; i < config->bNumInterfaces && !found; ++i) {
        const struct libusb_interface *iface = &config->interface[i];
        for (int j = 0; j < iface->num_altsetting; ++j) {
            const struct libusb_interface_descriptor *alt = &iface->altsetting[j];
            if (alt->bInterfaceClass != LIBUSB_CLASS_APPLICATION ||
                alt->bInterfaceSubClass != 1 ||
                alt->bInterfaceProtocol != 2) {
                continue;
            }

            char alt_name[sizeof(local.alt_name)] = {0};
            if (alt->iInterface != 0) {
                int name_len = libusb_get_string_descriptor_ascii(handle,
                                                                   alt->iInterface,
                                                                   (unsigned char *)alt_name,
                                                                   (int)sizeof(alt_name));
                if (name_len < 0) {
                    alt_name[0] = '\0';
                }
            }

            bool is_internal_flash = strstr(alt_name, "Internal Flash") != NULL;
            bool is_default_alt = (alt->bAlternateSetting == 0);

            if (!found ||
                is_internal_flash ||
                (is_default_alt && local.alt_setting != 0)) {
                memset(&local, 0, sizeof(local));
                local.handle = handle;
                local.interface_number = alt->bInterfaceNumber;
                local.alt_setting = alt->bAlternateSetting;
                local.transfer_size = parse_transfer_size(alt);
                strncpy_s(local.alt_name, sizeof(local.alt_name),
                          alt_name, _TRUNCATE);
                found = true;
                if (is_internal_flash) {
                    break;
                }
            }
        }
    }

    libusb_free_config_descriptor(config);

    if (found && out_device) {
        *out_device = local;
    }
    return found;
}

static bool open_stm32_dfu_device_once(libusb_context *ctx,
                                       stm32_dfu_device_t *out_device,
                                       char *error_buf,
                                       size_t error_len)
{
    libusb_device **list = NULL;
    ssize_t count = libusb_get_device_list(ctx, &list);
    if (count < 0) {
        if (error_buf && error_len) {
            set_status(error_buf, error_len,
                       "错误: 枚举 USB 设备失败 (%s)",
                       libusb_error_name((int)count));
        }
        return false;
    }

    bool success = false;
    for (ssize_t idx = 0; idx < count; ++idx) {
        libusb_device *dev = list[idx];
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(dev, &desc) != LIBUSB_SUCCESS) {
            continue;
        }
        if (desc.idVendor != STM32_VENDOR_ID || desc.idProduct != STM32_PRODUCT_ID) {
            continue;
        }

        libusb_device_handle *handle = NULL;
        int ret = libusb_open(dev, &handle);
        if (ret != LIBUSB_SUCCESS || !handle) {
            if (error_buf && error_len) {
                if (ret == LIBUSB_ERROR_ACCESS) {
                    set_status(error_buf, error_len,
                               "错误: 无法访问 DFU 设备，请安装 WinUSB/libusbK 驱动 (%s)",
                               libusb_error_name(ret));
                } else {
                    set_status(error_buf, error_len,
                               "错误: 无法打开 DFU 设备 (%s)",
                               libusb_error_name(ret));
                }
            }
            continue;
        }

        stm32_dfu_device_t device = {0};
        if (select_dfu_interface(dev, handle, &device)) {
            if (libusb_kernel_driver_active(handle, device.interface_number) == 1) {
                libusb_detach_kernel_driver(handle, device.interface_number);
            }
            ret = libusb_claim_interface(handle, device.interface_number);
            if (ret == LIBUSB_SUCCESS) {
                ret = libusb_set_interface_alt_setting(handle,
                                                       device.interface_number,
                                                       device.alt_setting);
            }
            if (ret == LIBUSB_SUCCESS) {
                device.handle = handle;
                if (out_device) {
                    *out_device = device;
                }
                success = true;
                break;
            }
            if (error_buf && error_len) {
                set_status(error_buf, error_len,
                           "错误: 无法占用 DFU 接口 (%s)",
                           libusb_error_name(ret));
            }
            libusb_close(handle);
        } else {
            if (error_buf && error_len) {
                set_status(error_buf, error_len,
                           "错误: 未找到匹配的 DFU 接口");
            }
            libusb_close(handle);
        }
    }

    libusb_free_device_list(list, 1);

    return success;
}

static bool wait_for_stm32_dfu_device(libusb_context *ctx,
                                      stm32_dfu_device_t *out_device,
                                      DWORD timeout_ms,
                                      char *status_buf,
                                      size_t status_len)
{
    DWORD elapsed = 0;
    char last_error[160] = {0};

    while (elapsed <= timeout_ms) {
        last_error[0] = '\0';
        if (open_stm32_dfu_device_once(ctx, out_device, last_error, sizeof(last_error))) {
            if (status_buf && status_len) {
                if (out_device->alt_name[0]) {
                    set_status(status_buf, status_len,
                               "正在使用 DFU 接口 %u: %s",
                               out_device->alt_setting,
                               out_device->alt_name);
                } else {
                    set_status(status_buf, status_len,
                               "正在使用 DFU 接口 %u",
                               out_device->alt_setting);
                }
            }
            return true;
        }

        Sleep(100);
        elapsed += 100;
    }

    if (status_buf && status_len) {
        if (last_error[0]) {
            set_status(status_buf, status_len, "%s", last_error);
        } else {
            set_status(status_buf, status_len,
                       "错误: 未找到 DFU 设备 (VID=0x%04X PID=0x%04X)",
                       STM32_VENDOR_ID, STM32_PRODUCT_ID);
        }
    }
    return false;
}

static void release_device(stm32_dfu_device_t *device)
{
    if (!device || !device->handle) {
        return;
    }
    libusb_release_interface(device->handle, device->interface_number);
    libusb_close(device->handle);
    device->handle = NULL;
}

static int dfu_get_status(stm32_dfu_device_t *device, dfu_status_t *out_status)
{
    unsigned char buffer[6] = {0};
    int transferred = libusb_control_transfer(
        device->handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
        DFU_REQUEST_GETSTATUS,
        0,
        device->interface_number,
        buffer,
        sizeof(buffer),
        1000);

    if (transferred < 0) {
        return transferred;
    }
    if (transferred != sizeof(buffer)) {
        return LIBUSB_ERROR_IO;
    }

    if (out_status) {
        out_status->status = buffer[0];
        out_status->poll_timeout_ms = (uint32_t)buffer[1]
                                     | ((uint32_t)buffer[2] << 8)
                                     | ((uint32_t)buffer[3] << 16);
        out_status->state = buffer[4];
    }

    return LIBUSB_SUCCESS;
}

static bool dfu_clear_status(stm32_dfu_device_t *device)
{
    int ret = libusb_control_transfer(
        device->handle,
        LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT,
        DFU_REQUEST_CLRSTATUS,
        0,
        device->interface_number,
        NULL,
        0,
        1000);
    return ret >= 0;
}

static bool dfu_abort(stm32_dfu_device_t *device)
{
    int ret = libusb_control_transfer(
        device->handle,
        LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT,
        DFU_REQUEST_ABORT,
        0,
        device->interface_number,
        NULL,
        0,
        1000);
    return ret >= 0;
}

static bool dfu_wait_ready(stm32_dfu_device_t *device,
                           char *status_buf,
                           size_t status_len,
                           bool allow_manifest)
{
    for (;;) {
        dfu_status_t st = {0};
        int ret = dfu_get_status(device, &st);
        if (ret == LIBUSB_ERROR_NO_DEVICE) {
            return allow_manifest;
        }
        if (ret != LIBUSB_SUCCESS) {
            set_status(status_buf, status_len,
                       "错误: 读取 DFU 状态失败 (%s)",
                       libusb_error_name(ret));
            return false;
        }

        if (st.status != 0) {
            dfu_clear_status(device);
            set_status(status_buf, status_len,
                       "错误: DFU 状态 0x%02X (state=0x%02X)",
                       st.status, st.state);
            return false;
        }

        switch (st.state) {
        case DFU_STATE_dfuIDLE:
        case DFU_STATE_dfuDNLOAD_IDLE:
            return true;
        case DFU_STATE_dfuMANIFEST_SYNC:
        case DFU_STATE_dfuMANIFEST:
        case DFU_STATE_dfuMANIFEST_WAIT_RESET:
            return allow_manifest;
        case DFU_STATE_dfuERROR:
            dfu_clear_status(device);
            set_status(status_buf, status_len,
                       "错误: DFU 进入错误状态");
            return false;
        default:
            break;
        }

        DWORD wait_ms = (st.poll_timeout_ms > 0) ? st.poll_timeout_ms : 5;
        Sleep(wait_ms);
    }
}

static bool dfu_download_block(stm32_dfu_device_t *device,
                               uint16_t block_number,
                               const uint8_t *data,
                               uint16_t length,
                               char *status_buf,
                               size_t status_len,
                               bool allow_manifest)
{
    int ret = libusb_control_transfer(
        device->handle,
        LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT,
        DFU_REQUEST_DNLOAD,
        block_number,
        device->interface_number,
        (unsigned char *)data,
        length,
        1000);

    if (ret < 0) {
        set_status(status_buf, status_len,
                   "错误: 发送 DFU 数据失败 (%s)",
                   libusb_error_name(ret));
        return false;
    }
    if (ret != length) {
        set_status(status_buf, status_len,
                   "错误: DFU 数据发送不完整");
        return false;
    }

    return dfu_wait_ready(device, status_buf, status_len, allow_manifest);
}

static bool dfu_set_address_pointer(stm32_dfu_device_t *device,
                                    uint32_t address,
                                    char *status_buf,
                                    size_t status_len)
{
    uint8_t payload[5];
    payload[0] = 0x21; // Set Address Pointer
    payload[1] = (uint8_t)(address & 0xFF);
    payload[2] = (uint8_t)((address >> 8) & 0xFF);
    payload[3] = (uint8_t)((address >> 16) & 0xFF);
    payload[4] = (uint8_t)((address >> 24) & 0xFF);

    return dfu_download_block(device, 0, payload, sizeof(payload),
                              status_buf, status_len, false);
}

static bool dfu_mass_erase(stm32_dfu_device_t *device,
                           char *status_buf,
                           size_t status_len)
{
    uint8_t payload[3] = {0x41, 0xFF, 0xFF};
    return dfu_download_block(device, 0, payload, sizeof(payload),
                              status_buf, status_len, false);
}

static bool dfu_prepare_idle(stm32_dfu_device_t *device,
                             char *status_buf,
                             size_t status_len)
{
    if (!dfu_wait_ready(device, status_buf, status_len, true)) {
        dfu_abort(device);
        Sleep(5);
        return dfu_wait_ready(device, status_buf, status_len, true);
    }
    return true;
}

bool dfu_loader_flash(const wchar_t *firmware_path,
                      dfu_loader_progress_cb progress_cb,
                      void *progress_ctx,
                      char *status_buf,
                      size_t status_buf_len)
{
    unsigned char *firmware_data = NULL;
    size_t firmware_len = 0;
    libusb_context *ctx = NULL;
    stm32_dfu_device_t device = {0};
    bool success = false;

    report_progress(progress_cb, progress_ctx, 0);
    set_status(status_buf, status_buf_len, "正在加载固件文件...");

    if (!load_firmware_file(firmware_path, &firmware_data, &firmware_len,
                            status_buf, status_buf_len)) {
        goto cleanup;
    }

    set_status(status_buf, status_buf_len, "正在初始化 USB...");
    int ret = libusb_init(&ctx);
    if (ret != LIBUSB_SUCCESS) {
        set_status(status_buf, status_buf_len,
                   "错误: 初始化 libusb 失败 (%s)", libusb_error_name(ret));
        goto cleanup;
    }

    set_status(status_buf, status_buf_len, "等待 DFU 设备出现...");
    if (!wait_for_stm32_dfu_device(ctx, &device, 10000, status_buf, status_buf_len)) {
        goto cleanup;
    }

    set_status(status_buf, status_buf_len, "正在准备设备...");
    if (!dfu_prepare_idle(&device, status_buf, status_buf_len)) {
        goto cleanup;
    }

    set_status(status_buf, status_buf_len, "正在擦除闪存...");
    report_progress(progress_cb, progress_ctx, 3);
    if (!dfu_mass_erase(&device, status_buf, status_buf_len)) {
        goto cleanup;
    }

    if (!dfu_prepare_idle(&device, status_buf, status_buf_len)) {
        goto cleanup;
    }

    set_status(status_buf, status_buf_len, "设置写入地址 0x%08X", STM32_BASE_ADDRESS);
    if (!dfu_set_address_pointer(&device, STM32_BASE_ADDRESS, status_buf, status_buf_len)) {
        goto cleanup;
    }

    report_progress(progress_cb, progress_ctx, 6);
    set_status(status_buf, status_buf_len, "正在写入固件...");

    uint16_t block_number = 2;
    size_t written = 0;
    while (written < firmware_len) {
        size_t remaining = firmware_len - written;
        uint16_t chunk = (uint16_t)((remaining > device.transfer_size)
                                    ? device.transfer_size
                                    : remaining);
        if (!dfu_download_block(&device,
                                block_number,
                                firmware_data + written,
                                chunk,
                                status_buf,
                                status_buf_len,
                                false)) {
            goto cleanup;
        }

        written += chunk;
        ++block_number;

        int percent = 6 + (int)((written * 90ULL) / firmware_len);
        if (percent > 95) {
            percent = 95;
        }
        report_progress(progress_cb, progress_ctx, percent);
    }

    if (!dfu_download_block(&device, 0, NULL, 0,
                            status_buf, status_buf_len, true)) {
        goto cleanup;
    }

    report_progress(progress_cb, progress_ctx, 99);
    set_status(status_buf, status_buf_len, "固件写入完成，设备正在重启");
    success = true;

cleanup:
    if (success) {
        report_progress(progress_cb, progress_ctx, 100);
    }

    release_device(&device);

    if (ctx) {
        libusb_exit(ctx);
    }

    if (firmware_data) {
        free(firmware_data);
    }

    if (!success && status_buf && status_buf_len && status_buf[0] == '\0') {
        set_status(status_buf, status_buf_len, "未知错误: DFU 更新失败");
    }

    return success;
}
