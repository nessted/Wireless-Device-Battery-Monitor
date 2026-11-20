#pragma once
// Pure (I/O-free) parts of the Logitech Centurion protocol used by the G321 LIGHTSPEED headset
// (046D:0B39) and other devices in the same family (e.g. G522 LIGHTSPEED).
//
// Centurion uses vendor-defined HID reports on usage_page 0xFF00 or 0xFF0B rather than the
// standard HID++ 1.0/2.0 (0x10/0x11) messages. The transport layer is CPLP (Centurion Packet
// Layer Protocol): each HID report carries a CPLP frame with a report ID, transport header,
// payload length, and payload bytes, zero-padded to the report size.
//
// Battery is obtained through HID++ 2.0–style feature discovery on top of the CPLP transport:
//   1. Query the Root feature (0x0000) with the desired feature ID to get its local index.
//   2. Call the battery feature at that index (function 0 or 1) to read SoC / charging status.
//
// Kept independent of Win32 so it can be unit-tested without hardware.
// References: Logitech G HUB captures, Solaar Centurion/HID++ 2.0 spec drafts,
// portyporty/g435-battery-percentage (MIT) for the sister G435 protocol.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace Centurion {

// ---- CPLP frame constants ----------------------------------------------------------------

// Known report IDs for Centurion vendor reports.
// The dongle may expose multiple report IDs; the one on usage_page 0xFF00 is typically used.
constexpr std::uint8_t kReportIdShort = 0x11;  // 20-byte payload (HID++ 2.0 long equivalent)
constexpr std::uint8_t kReportIdLong  = 0x12;  // 64-byte payload (extended)

// CPLP frame size (excluding the report ID byte that is prepended for HID writes).
constexpr std::size_t kFrameSizeShort = 20;
constexpr std::size_t kFrameSizeLong  = 64;

// ---- HID++ 2.0 over CPLP -----------------------------------------------------------------

// A HID++ 2.0 message as carried inside CPLP frames. The structure is the same as standard
// HID++ 2.0 but travels on a different report ID / transport.
struct Hidpp20Msg {
    std::uint8_t reportId = kReportIdShort;
    std::uint8_t deviceIndex = 0xFF;  // 0xFF = direct device (not through a receiver sub-index)
    std::uint8_t featureIndex = 0;
    std::uint8_t funcSwId = 0;        // (function << 4) | software_id
    std::uint8_t params[16] = {};
    std::size_t  paramLen = 0;
};

constexpr std::uint8_t kSwId = 0x0D;  // our software ID tag

// Build the byte vector to write to the HID device (report ID + payload, zero-padded).
inline std::vector<std::uint8_t> PackMessage(const Hidpp20Msg& m) {
    // For the short report: 1 byte report ID + 20 bytes payload.
    std::size_t frameSize = (m.reportId == kReportIdLong) ? kFrameSizeLong : kFrameSizeShort;
    std::vector<std::uint8_t> buf(1 + frameSize, 0);
    buf[0] = m.reportId;
    buf[1] = m.deviceIndex;
    buf[2] = m.featureIndex;
    buf[3] = m.funcSwId;
    for (std::size_t i = 0; i < m.paramLen && i < sizeof(m.params) && (4 + i) < buf.size(); i++)
        buf[4 + i] = m.params[i];
    return buf;
}

// Build a feature-discovery request: Root feature (index 0x00), function 0 (getFeatureIndex),
// params = [featureId_hi, featureId_lo].
inline std::vector<std::uint8_t> PackRootQuery(std::uint16_t featureId, std::uint8_t deviceIndex = 0xFF) {
    Hidpp20Msg m;
    m.deviceIndex = deviceIndex;
    m.featureIndex = 0x00;  // IRoot
    m.funcSwId = (0 << 4) | kSwId;  // function 0 = getFeatureIndex
    m.params[0] = (std::uint8_t)(featureId >> 8);
    m.params[1] = (std::uint8_t)(featureId & 0xFF);
    m.paramLen = 2;
    return PackMessage(m);
}

// Build a battery status request given a feature index (function 0 = get_status for Unified
// Battery 0x1004, or function 0 = GetBatteryLevelStatus for 0x1000).
inline std::vector<std::uint8_t> PackBatteryQuery(std::uint8_t batteryFeatureIndex,
                                                   std::uint8_t function = 0,
                                                   std::uint8_t deviceIndex = 0xFF) {
    Hidpp20Msg m;
    m.deviceIndex = deviceIndex;
    m.featureIndex = batteryFeatureIndex;
    m.funcSwId = (function << 4) | kSwId;
    m.paramLen = 0;
    return PackMessage(m);
}

// ---- Reply parsing -----------------------------------------------------------------------

// Check if a received buffer is a HID++ 2.0 reply matching our expectations.
// `data` is the raw HID read buffer (may or may not have a leading report ID depending on the
// OS stripping behaviour; we try both offsets).
struct ParsedReply {
    bool valid = false;
    bool isError = false;
    std::uint8_t deviceIndex = 0;
    std::uint8_t featureIndex = 0;
    std::uint8_t funcSwId = 0;
    std::uint8_t errorCode = 0;
    std::uint8_t params[16] = {};
    std::size_t  paramLen = 0;
};

inline ParsedReply ParseReply(const std::uint8_t* data, std::size_t size) {
    ParsedReply r;
    if (size < 4) return r;

    // The read buffer from HidDev::Read may start with the report ID (0x11) or directly with
    // the device index if the OS strips report ID 0. Try to detect:
    int off = 0;
    if (size >= 5 && (data[0] == kReportIdShort || data[0] == kReportIdLong)) {
        off = 1;  // report ID present
    }
    if ((std::size_t)(off + 4) > size) return r;

    r.deviceIndex  = data[off + 0];
    r.featureIndex = data[off + 1];
    r.funcSwId     = data[off + 2];

    // Error reply: featureIndex == 0xFF, params[0] = original featureIndex, params[1] = funcSwId,
    // params[2] = error code.
    if (r.featureIndex == 0xFF && (std::size_t)(off + 6) <= size) {
        r.isError = true;
        r.valid = true;
        r.errorCode = data[off + 5];
        // Store the original feature index and funcSwId that caused the error.
        r.params[0] = data[off + 3];  // original featureIndex
        r.params[1] = data[off + 4];  // original funcSwId
        r.paramLen = 2;
        return r;
    }

    r.valid = true;
    std::size_t avail = size - (off + 3);
    r.paramLen = avail > 16 ? 16 : avail;
    std::memcpy(r.params, data + off + 3, r.paramLen);
    return r;
}

// ---- Battery response parsing ------------------------------------------------------------

// Well-known battery feature IDs (HID++ 2.0).
constexpr std::uint16_t kFeatureUnifiedBattery = 0x1004;
constexpr std::uint16_t kFeatureBatteryStatus  = 0x1000;
constexpr std::uint16_t kFeatureBatteryVoltage = 0x1001;
constexpr std::uint16_t kFeatureAdcMeasurement = 0x1F20;

struct BatteryInfo {
    bool valid = false;
    int  percent = -1;        // 0–100 or -1 if unknown
    bool charging = false;
    bool full = false;
    std::wstring levelText;   // "Full", "Good", "Low", "Critical" for coarse-level devices
};

// Parse a Unified Battery (0x1004) get_status (function 1) response.
// Params: [soc%][level_bitmap][charging_status][...]
inline BatteryInfo ParseUnifiedBattery(const std::uint8_t* params, std::size_t len) {
    BatteryInfo b;
    if (len < 3) return b;
    int soc = params[0];
    int level = params[1];
    int st = params[2];
    b.valid = true;
    if (soc > 0 && soc <= 100) {
        b.percent = soc;
    } else if (level) {
        b.percent = (level & 8) ? 100 : (level & 4) ? 60 : (level & 2) ? 20 : 5;
        b.levelText = (level & 8) ? L"Full" : (level & 4) ? L"Good" : (level & 2) ? L"Low" : L"Critical";
    }
    b.charging = st == 1 || st == 2 || st == 3;
    b.full = st == 3;  // charging complete
    return b;
}

// Parse a Battery Status (0x1000) GetBatteryLevelStatus (function 0) response.
// Params: [percent][next_percent][status][...]
inline BatteryInfo ParseBatteryStatus(const std::uint8_t* params, std::size_t len) {
    BatteryInfo b;
    if (len < 3) return b;
    b.valid = true;
    int st = params[2];
    b.charging = st >= 1 && st <= 4;
    b.full = st == 3;
    b.percent = params[0] <= 100 ? (int)params[0] : -1;
    if (b.percent == 0 && st == 3) b.percent = 100;
    return b;
}

// ---- Debug helpers -----------------------------------------------------------------------

inline std::string HexDump(const std::uint8_t* data, std::size_t len) {
    std::string s;
    char buf[4];
    for (std::size_t i = 0; i < len; i++) {
        if (i) s += ' ';
        snprintf(buf, sizeof(buf), "%02X", data[i]);
        s += buf;
    }
    return s;
}

inline std::string HexDump(const std::vector<std::uint8_t>& v) {
    return HexDump(v.data(), v.size());
}

}  // namespace Centurion
