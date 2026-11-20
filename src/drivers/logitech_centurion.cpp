// Logitech G321 LIGHTSPEED headset (046D:0B39) on its 2.4 GHz dongle.
//
// The G321 uses the "Centurion" protocol family (the same as the G522 LIGHTSPEED): HID++ 2.0
// feature discovery and battery queries are carried over vendor-defined HID reports on
// usage_page 0xFF00 (usage 0x0212) or 0xFF0B (usage 0x0101), NOT the standard HID++ 0x10/0x11
// short/long collections. The Consumer Control collection (0x000C:0x0001) is for audio and
// must be ignored.
//
// Battery query sequence:
//   1. Open the vendor-defined HID collection (FF00:0212 preferred, FF0B:0101 as fallback).
//   2. Root feature query (feature index 0x00, function 0): look up Unified Battery (0x1004)
//      or Battery Status (0x1000) to get the local feature index.
//   3. Call the battery feature (function 0 or 1) to read SoC% and charging status.
//
// If the headset is turned off or out of range the dongle is still enumerated but reads time
// out; the driver reports the device as disconnected.
//
// Architecture follows the same pattern as logitech_g435_0adf.cpp: a standalone PollXxx()
// function registered in drivers.h and called from devices.cpp before the generic HID++
// PollLogitech(), so the HID++ driver does not try to talk HID++ to this dongle.

#include "drivers.h"
#include "logitech_centurion.h"

#include <cstdio>

namespace {

constexpr USHORT kVid = 0x046D;
constexpr USHORT kPid = 0x0B39;

// Debug logging: writes to OutputDebugStringA when a debugger is attached, and to stderr if a
// console is allocated (e.g. `--dump` or `--list`).  Completely compiled out in release builds
// unless CENTURION_DEBUG is defined; on debug builds it is always active.
#if !defined(NDEBUG) || defined(CENTURION_DEBUG)
#define CENT_DBG 1
#else
#define CENT_DBG 0
#endif

#if CENT_DBG
void CentLog(const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    OutputDebugStringA(buf);
    // Also try stderr for `--list` / `--dump` console runs.
    HANDLE h = GetStdHandle(STD_ERROR_HANDLE);
    if (h && h != INVALID_HANDLE_VALUE) {
        DWORD w;
        WriteFile(h, buf, (DWORD)strlen(buf), &w, nullptr);
    }
}
#else
inline void CentLog(const char*, ...) {}
#endif

// Sends a HID++ 2.0 message over the Centurion vendor collection and waits for a matching reply.
// Returns 1 on success (params in `reply`), 0 on HID++ error (errorCode set), -1 on timeout.
int CenturionTransact(HidDev& d, std::uint8_t featureIndex, std::uint8_t funcSwId,
                       const std::uint8_t* params, std::size_t paramLen,
                       Centurion::ParsedReply& reply, int timeoutMs = 800) {
    // Build outgoing message.
    Centurion::Hidpp20Msg m;
    m.deviceIndex = 0xFF;
    m.featureIndex = featureIndex;
    m.funcSwId = funcSwId;
    m.paramLen = paramLen > sizeof(m.params) ? sizeof(m.params) : paramLen;
    if (paramLen) memcpy(m.params, params, m.paramLen);
    auto pkt = Centurion::PackMessage(m);

    CentLog("[centurion] TX: %s\n", Centurion::HexDump(pkt).c_str());

    d.Flush();
    if (!d.Write(pkt)) {
        CentLog("[centurion] Write failed\n");
        return -1;
    }

    ULONGLONG end = GetTickCount64() + timeoutMs;
    std::vector<BYTE> buf;
    while ((LONGLONG)end - (LONGLONG)GetTickCount64() > 0) {
        int left = (int)((LONGLONG)end - (LONGLONG)GetTickCount64());
        if (left <= 0) break;
        int n = d.Read(buf, left < 50 ? 50 : left);
        if (n < 4) continue;

        CentLog("[centurion] RX (%d bytes): %s\n", n, Centurion::HexDump(buf.data(), (size_t)n).c_str());

        reply = Centurion::ParseReply(buf.data(), (std::size_t)n);
        if (!reply.valid) continue;

        // Match: same feature index and function (ignoring swId nibble), or an error referencing our request.
        if (reply.isError) {
            CentLog("[centurion] Error reply: code=0x%02X for feat=0x%02X func=0x%02X\n",
                    reply.errorCode, reply.params[0], reply.params[1]);
            return 0;
        }
        // Check the reply matches our request's feature index.
        if (reply.featureIndex == featureIndex) {
            return 1;
        }
        // Otherwise it is an async notification or a reply to another request; keep reading.
    }
    CentLog("[centurion] Timeout waiting for reply to feat=0x%02X func=0x%02X\n", featureIndex, funcSwId);
    return -1;
}

// Discover a HID++ 2.0 feature through the Root feature (index 0x00).
// Returns the local feature index, or 0 on failure (0 is never a valid battery feature index).
std::uint8_t DiscoverFeature(HidDev& d, std::uint16_t featureId) {
    std::uint8_t params[2] = { (std::uint8_t)(featureId >> 8), (std::uint8_t)(featureId & 0xFF) };
    Centurion::ParsedReply reply;
    int r = CenturionTransact(d, 0x00, (0 << 4) | Centurion::kSwId, params, 2, reply, 1000);
    if (r == 1 && reply.paramLen >= 1 && reply.params[0] != 0) {
        CentLog("[centurion] Feature 0x%04X -> index 0x%02X\n", featureId, reply.params[0]);
        return reply.params[0];
    }
    CentLog("[centurion] Feature 0x%04X not found (r=%d)\n", featureId, r);
    return 0;
}

// Try to read battery using Unified Battery (0x1004), then Battery Status (0x1000).
bool ReadCenturionBattery(HidDev& d, DeviceStatus& s) {
    // Try Unified Battery first (most modern Logitech devices).
    std::uint8_t idx = DiscoverFeature(d, Centurion::kFeatureUnifiedBattery);
    if (idx) {
        // Unified Battery: function 1 = get_status.
        Centurion::ParsedReply reply;
        int r = CenturionTransact(d, idx, (1 << 4) | Centurion::kSwId, nullptr, 0, reply, 800);
        if (r == 1 && reply.paramLen >= 3) {
            auto b = Centurion::ParseUnifiedBattery(reply.params, reply.paramLen);
            if (b.valid) {
                CentLog("[centurion] Unified Battery: %d%%, charging=%d, full=%d\n",
                        b.percent, b.charging, b.full);
                s.percent = b.percent;
                s.charging = b.charging;
                s.full = b.full;
                s.levelText = b.levelText;
                return true;
            }
        }
        // Some devices answer get_status on function 0 instead.
        r = CenturionTransact(d, idx, (0 << 4) | Centurion::kSwId, nullptr, 0, reply, 800);
        if (r == 1 && reply.paramLen >= 3) {
            auto b = Centurion::ParseUnifiedBattery(reply.params, reply.paramLen);
            if (b.valid) {
                CentLog("[centurion] Unified Battery (fn0): %d%%, charging=%d, full=%d\n",
                        b.percent, b.charging, b.full);
                s.percent = b.percent;
                s.charging = b.charging;
                s.full = b.full;
                s.levelText = b.levelText;
                return true;
            }
        }
    }

    // Fallback: Battery Status (0x1000).
    idx = DiscoverFeature(d, Centurion::kFeatureBatteryStatus);
    if (idx) {
        Centurion::ParsedReply reply;
        int r = CenturionTransact(d, idx, (0 << 4) | Centurion::kSwId, nullptr, 0, reply, 800);
        if (r == 1 && reply.paramLen >= 3) {
            auto b = Centurion::ParseBatteryStatus(reply.params, reply.paramLen);
            if (b.valid) {
                CentLog("[centurion] Battery Status: %d%%, charging=%d, full=%d\n",
                        b.percent, b.charging, b.full);
                s.percent = b.percent;
                s.charging = b.charging;
                s.full = b.full;
                return true;
            }
        }
    }

    // Fallback: Battery Voltage (0x1001).
    idx = DiscoverFeature(d, Centurion::kFeatureBatteryVoltage);
    if (idx) {
        Centurion::ParsedReply reply;
        int r = CenturionTransact(d, idx, (0 << 4) | Centurion::kSwId, nullptr, 0, reply, 800);
        if (r == 1 && reply.paramLen >= 3) {
            int mv = (reply.params[0] << 8) | reply.params[1];
            if (mv >= 2500 && mv <= 4500) {
                CentLog("[centurion] Battery Voltage: %d mV, flags=0x%02X\n", mv, reply.params[2]);
                s.millivolts = mv;
                s.charging = (reply.params[2] & 0x80) != 0;
                // Simple piecewise-linear for a typical Li-ion cell.
                static const int v[] = { 4200, 4150, 4100, 4050, 4000, 3950, 3900, 3850,
                                          3800, 3750, 3700, 3650, 3600, 3550, 3500, 3400 };
                static const int p[] = { 100, 95, 90, 85, 80, 75, 70, 62,
                                          55, 45, 35, 25, 15, 10, 5, 0 };
                if (mv >= v[0]) { s.percent = 100; }
                else {
                    s.percent = 0;
                    for (int i = 1; i < 16; i++) {
                        if (mv >= v[i]) {
                            s.percent = p[i] + ((p[i-1] - p[i]) * (mv - v[i]) + (v[i-1] - v[i]) / 2) / (v[i-1] - v[i]);
                            break;
                        }
                    }
                }
                return true;
            }
        }
    }

    // Fallback: ADC Measurement (0x1F20) used by some headsets.
    idx = DiscoverFeature(d, Centurion::kFeatureAdcMeasurement);
    if (idx) {
        Centurion::ParsedReply reply;
        int r = CenturionTransact(d, idx, (0 << 4) | Centurion::kSwId, nullptr, 0, reply, 800);
        if (r == 1 && reply.paramLen >= 3) {
            int mv = (reply.params[0] << 8) | reply.params[1];
            if (mv >= 2500 && mv <= 4500) {
                CentLog("[centurion] ADC Measurement: %d mV, status=0x%02X\n", mv, reply.params[2]);
                s.millivolts = mv;
                s.charging = reply.params[2] == 0x03;
                // Same default headset curve as the HID++ driver.
                static const int dv[] = { 4100, 3850, 3780, 3730, 3650, 3300 };
                static const int dp[] = { 100, 50, 30, 20, 5, 0 };
                s.percent = Interpolate(mv, dv, dp, 6);
                return true;
            }
        }
    }

    CentLog("[centurion] No battery feature found\n");
    return false;
}

}  // namespace

void PollLogitechG321(PollContext& ctx) {
    if (ctx.IsClaimed(kVid, kPid)) return;

    // Look for the vendor-defined HID collections on the G321 dongle.
    // Prefer FF00:0212 (primary Centurion transport), fall back to FF0B:0101.
    auto colls = ctx.Find(kVid, kPid, [](const HidColl& c) {
        return (c.page() == 0xFF00 && c.usage() == 0x0212) ||
               (c.page() == 0xFF0B && c.usage() == 0x0101);
    });
    if (colls.empty()) {
        // Some firmware versions may expose the Centurion transport with usage 0x0001 on page
        // 0xFF00 (like older LIGHTSPEED dongles) but with longer reports. Try that too, but
        // only for the G321 PID.
        colls = ctx.Find(kVid, kPid, [](const HidColl& c) {
            return c.page() == 0xFF00 && c.usage() == 0x0001 && c.outLen() >= 21;
        });
    }
    if (colls.empty()) return;

    // Sort: prefer FF00:0212 over FF0B:0101 over FF00:0001.
    if (colls.size() > 1) {
        std::sort(colls.begin(), colls.end(), [](const HidColl* a, const HidColl* b) {
            if (a->page() == 0xFF00 && a->usage() == 0x0212) return true;
            if (b->page() == 0xFF00 && b->usage() == 0x0212) return false;
            if (a->page() == 0xFF0B) return true;
            return false;
        });
    }

    CentLog("[centurion] Found %zu vendor collection(s) for G321 (046D:0B39)\n", colls.size());
    for (size_t i = 0; i < colls.size(); i++) {
        CentLog("[centurion]   [%zu] page=0x%04X usage=0x%04X inLen=%d outLen=%d if=%d\n",
                i, colls[i]->page(), colls[i]->usage(), colls[i]->inLen(), colls[i]->outLen(), colls[i]->iface);
    }

    // Try opening the first suitable collection.
    HidDev d;
    bool opened = false;
    for (const HidColl* c : colls) {
        CentLog("[centurion] Trying to open collection page=0x%04X usage=0x%04X\n", c->page(), c->usage());
        if (d.Open(*c)) {
            opened = true;
            CentLog("[centurion] Opened successfully\n");
            break;
        }
        CentLog("[centurion] Open failed, trying next\n");
    }
    if (!opened) return;

    DeviceStatus s;
    s.key = L"g321:" + Hex4(kPid);
    s.name = L"Logitech G321 LIGHTSPEED";
    s.kind = DeviceKind::Headset;
    s.via = L"2.4 GHz receiver";

    if (!ReadCenturionBattery(d, s)) {
        // Dongle present but no battery reply: headset is off or out of range.
        CentLog("[centurion] No battery reply — headset offline\n");
        s.connected = false;
        ctx.Add(s, kVid, kPid);
        return;
    }

    CentLog("[centurion] Battery: %d%%, charging=%d, full=%d\n", s.percent, s.charging, s.full);
    ctx.Add(s, kVid, kPid);
}
