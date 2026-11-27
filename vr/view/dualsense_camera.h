#pragma once

#include <windows.h>
#include <hidsdi.h>
#include <xinput.h>
#include <array>
#include <vector>

#pragma comment(lib, "hid.lib")

namespace vrc::flycam::dualsense {

// USB and short Bluetooth reports start their four sticks after the report ID.
// A full Bluetooth report adds one byte before the same stick fields.
struct PadSample {
    SHORT leftX = 0, leftDown = 0, rightX = 0, rightDown = 0;
    WORD dpad = 0;
};

inline bool ParseReport(const BYTE* report, DWORD size, PadSample& sample) {
    if (size < 5) return false;
    const DWORD offset = report[0] == 0x31 && size >= 6 ? 2 : report[0] == 0x01 ? 1 : 0;
    if (!offset) return false;
    const auto axis = [](BYTE value) { return static_cast<SHORT>((int(value) - 128) * 256); };
    sample = {axis(report[offset]), axis(report[offset + 1]), axis(report[offset + 2]), axis(report[offset + 3])};
    // The short Bluetooth report puts the hat immediately after the sticks.
    const DWORD hatOffset = report[0] == 0x01 && (size == 10 || size == 78) ? 5 : offset + 7;
    if (hatOffset < size) {
        constexpr WORD up = XINPUT_GAMEPAD_DPAD_UP, down = XINPUT_GAMEPAD_DPAD_DOWN;
        constexpr WORD left = XINPUT_GAMEPAD_DPAD_LEFT, right = XINPUT_GAMEPAD_DPAD_RIGHT;
        constexpr WORD directions[] = {up, up | right, right, down | right, down, down | left, left, up | left};
        const BYTE hat = report[hatOffset] & 0x0f;
        if (hat < 8) sample.dpad = directions[hat];
    }
    return true;
}

// Owned by the flycam's pose lock. No device I/O is done in the XInput hook.
class Reader {
public:
    Reader() = default;
    Reader(const Reader&) = delete;
    Reader& operator=(const Reader&) = delete;
    ~Reader() { Close(); }

    void Close() {
        if (pending_) {
            CancelIoEx(device_, &operation_);
            DWORD transferred = 0;
            GetOverlappedResult(device_, &operation_, &transferred, TRUE);
            pending_ = false;
        }
        if (device_ != INVALID_HANDLE_VALUE) CloseHandle(device_);
        if (event_) CloseHandle(event_);
        device_ = INVALID_HANDLE_VALUE;
        event_ = nullptr;
        sampleAt_ = 0;
    }

    bool Read(ULONGLONG now, PadSample& sample) {
        if (device_ == INVALID_HANDLE_VALUE) {
            if (now < retryAt_) return false;
            retryAt_ = now + 1500;
            if (!OpenController()) return false;
        }
        for (int i = 0; i < 16; ++i) {
            DWORD transferred = 0;
            if (pending_) {
                if (!GetOverlappedResult(device_, &operation_, &transferred, FALSE)) {
                    if (GetLastError() == ERROR_IO_INCOMPLETE) break;
                    pending_ = false;
                    Close();
                    return false;
                }
                pending_ = false;
            } else {
                ResetEvent(event_);
                operation_ = {};
                operation_.hEvent = event_;
                if (!ReadFile(device_, report_.data(), reportSize_, &transferred, &operation_)) {
                    if (GetLastError() == ERROR_IO_PENDING) {
                        pending_ = true;
                        break;
                    }
                    Close();
                    return false;
                }
            }
            if (ParseReport(report_.data(), transferred, sample_)) sampleAt_ = now;
        }
        if (!sampleAt_ || now - sampleAt_ > 250) return false;
        sample = sample_;
        return true;
    }

private:
    bool OpenController() {
        UINT count = 0;
        if (GetRawInputDeviceList(nullptr, &count, sizeof(RAWINPUTDEVICELIST)) != 0 || !count) return false;
        std::vector<RAWINPUTDEVICELIST> devices(count);
        const UINT found = GetRawInputDeviceList(devices.data(), &count, sizeof(RAWINPUTDEVICELIST));
        if (found == UINT(-1)) return false;
        for (UINT i = 0; i < found; ++i) {
            const auto& raw = devices[i];
            if (raw.dwType != RIM_TYPEHID) continue;
            RID_DEVICE_INFO info{};
            info.cbSize = sizeof info;
            UINT bytes = sizeof info;
            if (GetRawInputDeviceInfoW(raw.hDevice, RIDI_DEVICEINFO, &info, &bytes) == UINT(-1)) continue;
            if (info.hid.dwVendorId != 0x054c || info.hid.dwProductId != 0x0ce6
                || info.hid.usUsagePage != 1 || info.hid.usUsage != 5) continue;
            UINT chars = 0;
            if (GetRawInputDeviceInfoW(raw.hDevice, RIDI_DEVICENAME, nullptr, &chars) == UINT(-1) || !chars) continue;
            std::vector<wchar_t> path(chars + 1, L'\0');
            if (GetRawInputDeviceInfoW(raw.hDevice, RIDI_DEVICENAME, path.data(), &chars) == UINT(-1)) continue;
            const HANDLE handle = CreateFileW(path.data(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,
                                              nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
            if (handle == INVALID_HANDLE_VALUE) continue;
            PHIDP_PREPARSED_DATA data = nullptr;
            HIDP_CAPS caps{};
            bool valid = HidD_GetPreparsedData(handle, &data) != FALSE;
            if (valid) {
                valid = HidP_GetCaps(data, &caps) == HIDP_STATUS_SUCCESS;
                HidD_FreePreparsedData(data);
            }
            if (valid && caps.InputReportByteLength >= 5 && caps.InputReportByteLength <= report_.size()) {
                event_ = CreateEventW(nullptr, TRUE, FALSE, nullptr);
                if (event_) {
                    device_ = handle;
                    reportSize_ = caps.InputReportByteLength;
                    return true;
                }
            }
            CloseHandle(handle);
        }
        return false;
    }

    HANDLE device_ = INVALID_HANDLE_VALUE;
    HANDLE event_ = nullptr;
    OVERLAPPED operation_{};
    std::array<BYTE, 128> report_{};
    DWORD reportSize_ = 0;
    bool pending_ = false;
    ULONGLONG retryAt_ = 0, sampleAt_ = 0;
    PadSample sample_;
};

}  // namespace vrc::flycam::dualsense
