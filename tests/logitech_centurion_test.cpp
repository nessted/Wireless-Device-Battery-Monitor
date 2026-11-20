// Unit tests for the Centurion protocol layer (logitech_centurion.h): message packing,
// reply parsing, and battery response interpretation. No HID I/O, no hardware needed.
#include "../src/drivers/logitech_centurion.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

// ---- PackMessage tests -------------------------------------------------------------------

void TestPackMessageShort() {
    Centurion::Hidpp20Msg m;
    m.reportId = Centurion::kReportIdShort;
    m.deviceIndex = 0xFF;
    m.featureIndex = 0x03;
    m.funcSwId = 0x1D;  // function 1, swId 0x0D
    m.params[0] = 0xAB;
    m.params[1] = 0xCD;
    m.paramLen = 2;

    auto pkt = Centurion::PackMessage(m);
    // report ID (1) + frame (20) = 21 bytes
    assert(pkt.size() == 21);
    assert(pkt[0] == 0x11);  // report ID
    assert(pkt[1] == 0xFF);  // device index
    assert(pkt[2] == 0x03);  // feature index
    assert(pkt[3] == 0x1D);  // funcSwId
    assert(pkt[4] == 0xAB);  // param 0
    assert(pkt[5] == 0xCD);  // param 1
    // rest zero-padded
    for (size_t i = 6; i < pkt.size(); i++) assert(pkt[i] == 0);
    std::puts("  PackMessage (short): ok");
}

void TestPackMessageLong() {
    Centurion::Hidpp20Msg m;
    m.reportId = Centurion::kReportIdLong;
    m.deviceIndex = 0x01;
    m.featureIndex = 0x07;
    m.funcSwId = 0x0D;
    m.paramLen = 0;

    auto pkt = Centurion::PackMessage(m);
    assert(pkt.size() == 65);  // 1 + 64
    assert(pkt[0] == 0x12);
    assert(pkt[1] == 0x01);
    assert(pkt[2] == 0x07);
    assert(pkt[3] == 0x0D);
    std::puts("  PackMessage (long): ok");
}

// ---- PackRootQuery test ------------------------------------------------------------------

void TestPackRootQuery() {
    auto pkt = Centurion::PackRootQuery(0x1004, 0xFF);
    assert(pkt.size() == 21);
    assert(pkt[0] == 0x11);   // report ID
    assert(pkt[1] == 0xFF);   // device index
    assert(pkt[2] == 0x00);   // root feature index
    assert(pkt[3] == (0x00 | Centurion::kSwId));  // function 0 | swId
    assert(pkt[4] == 0x10);   // feature ID hi
    assert(pkt[5] == 0x04);   // feature ID lo
    std::puts("  PackRootQuery (0x1004): ok");
}

// ---- PackBatteryQuery test ---------------------------------------------------------------

void TestPackBatteryQuery() {
    auto pkt = Centurion::PackBatteryQuery(0x05, 1, 0xFF);
    assert(pkt.size() == 21);
    assert(pkt[0] == 0x11);
    assert(pkt[1] == 0xFF);
    assert(pkt[2] == 0x05);    // battery feature index
    assert(pkt[3] == ((1 << 4) | Centurion::kSwId));  // function 1 | swId
    std::puts("  PackBatteryQuery: ok");
}

// ---- ParseReply tests --------------------------------------------------------------------

void TestParseReplyNormal() {
    // Simulate a reply buffer (report ID stripped by OS, so starts with device index).
    std::uint8_t buf[] = { 0xFF, 0x05, 0x1D, 0x4B, 0x02, 0x01, 0x00, 0x00 };
    auto r = Centurion::ParseReply(buf, sizeof(buf));
    assert(r.valid);
    assert(!r.isError);
    assert(r.deviceIndex == 0xFF);
    assert(r.featureIndex == 0x05);
    assert(r.funcSwId == 0x1D);
    assert(r.params[0] == 0x4B);  // 75%
    assert(r.params[1] == 0x02);
    assert(r.params[2] == 0x01);
    std::puts("  ParseReply (normal, no report ID): ok");
}

void TestParseReplyWithReportId() {
    // Reply buffer with report ID present.
    std::uint8_t buf[] = { 0x11, 0xFF, 0x05, 0x1D, 0x4B, 0x02, 0x01, 0x00 };
    auto r = Centurion::ParseReply(buf, sizeof(buf));
    assert(r.valid);
    assert(!r.isError);
    assert(r.deviceIndex == 0xFF);
    assert(r.featureIndex == 0x05);
    assert(r.params[0] == 0x4B);
    std::puts("  ParseReply (with report ID): ok");
}

void TestParseReplyError() {
    // HID++ 2.0 error: featureIndex=0xFF, params = [origFeat, origFunc, errorCode]
    std::uint8_t buf[] = { 0x11, 0xFF, 0xFF, 0x00, 0x05, 0x1D, 0x05, 0x00 };
    auto r = Centurion::ParseReply(buf, sizeof(buf));
    assert(r.valid);
    assert(r.isError);
    assert(r.errorCode == 0x05);
    assert(r.params[0] == 0x05);  // original feature index
    assert(r.params[1] == 0x1D);  // original funcSwId
    std::puts("  ParseReply (error): ok");
}

void TestParseReplyTooShort() {
    std::uint8_t buf[] = { 0x11, 0xFF, 0x05 };
    auto r = Centurion::ParseReply(buf, sizeof(buf));
    // 3 bytes after stripping report ID = only 2 bytes for deviceIndex + featureIndex, missing funcSwId
    // The function should handle this gracefully.
    // With report ID detected (buf[0]==0x11), off=1, so we need off+4 = 5 bytes, but we have 3.
    assert(!r.valid || r.paramLen == 0);
    std::puts("  ParseReply (too short): ok");
}

// ---- Unified Battery parsing tests -------------------------------------------------------

void TestParseUnifiedBatteryNormal() {
    std::uint8_t params[] = { 75, 0x04, 0x00 };  // 75%, level=Good, discharging
    auto b = Centurion::ParseUnifiedBattery(params, 3);
    assert(b.valid);
    assert(b.percent == 75);
    assert(!b.charging);
    assert(!b.full);
    std::puts("  ParseUnifiedBattery (75%, discharging): ok");
}

void TestParseUnifiedBatteryCharging() {
    std::uint8_t params[] = { 50, 0x04, 0x01 };  // 50%, charging
    auto b = Centurion::ParseUnifiedBattery(params, 3);
    assert(b.valid);
    assert(b.percent == 50);
    assert(b.charging);
    assert(!b.full);
    std::puts("  ParseUnifiedBattery (50%, charging): ok");
}

void TestParseUnifiedBatteryFull() {
    std::uint8_t params[] = { 100, 0x08, 0x03 };  // 100%, Full, charge complete
    auto b = Centurion::ParseUnifiedBattery(params, 3);
    assert(b.valid);
    assert(b.percent == 100);
    assert(b.charging);
    assert(b.full);
    std::puts("  ParseUnifiedBattery (100%, full): ok");
}

void TestParseUnifiedBatteryCoarseLevel() {
    std::uint8_t params[] = { 0, 0x02, 0x00 };  // SoC=0 (unknown), level=Low, discharging
    auto b = Centurion::ParseUnifiedBattery(params, 3);
    assert(b.valid);
    assert(b.percent == 20);
    assert(b.levelText == L"Low");
    std::puts("  ParseUnifiedBattery (coarse level Low): ok");
}

void TestParseUnifiedBatteryCritical() {
    std::uint8_t params[] = { 0, 0x01, 0x00 };  // SoC=0, level=Critical
    auto b = Centurion::ParseUnifiedBattery(params, 3);
    assert(b.valid);
    assert(b.percent == 5);
    assert(b.levelText == L"Critical");
    std::puts("  ParseUnifiedBattery (Critical): ok");
}

void TestParseUnifiedBatteryTooShort() {
    std::uint8_t params[] = { 75, 0x04 };
    auto b = Centurion::ParseUnifiedBattery(params, 2);
    assert(!b.valid);
    std::puts("  ParseUnifiedBattery (too short): ok");
}

// ---- Battery Status parsing tests --------------------------------------------------------

void TestParseBatteryStatusNormal() {
    std::uint8_t params[] = { 60, 0, 0x00 };  // 60%, discharging
    auto b = Centurion::ParseBatteryStatus(params, 3);
    assert(b.valid);
    assert(b.percent == 60);
    assert(!b.charging);
    assert(!b.full);
    std::puts("  ParseBatteryStatus (60%, discharging): ok");
}

void TestParseBatteryStatusCharging() {
    std::uint8_t params[] = { 85, 0, 0x02 };  // 85%, charging
    auto b = Centurion::ParseBatteryStatus(params, 3);
    assert(b.valid);
    assert(b.percent == 85);
    assert(b.charging);
    assert(!b.full);
    std::puts("  ParseBatteryStatus (85%, charging): ok");
}

void TestParseBatteryStatusChargeComplete() {
    std::uint8_t params[] = { 0, 0, 0x03 };  // 0% but charge complete -> 100%
    auto b = Centurion::ParseBatteryStatus(params, 3);
    assert(b.valid);
    assert(b.percent == 100);
    assert(b.charging);
    assert(b.full);
    std::puts("  ParseBatteryStatus (charge complete, 0->100): ok");
}

void TestParseBatteryStatusInvalidPercent() {
    std::uint8_t params[] = { 0xFF, 0, 0x00 };  // 255 > 100 -> -1
    auto b = Centurion::ParseBatteryStatus(params, 3);
    assert(b.valid);
    assert(b.percent == -1);
    std::puts("  ParseBatteryStatus (invalid percent): ok");
}

// ---- HexDump test ------------------------------------------------------------------------

void TestHexDump() {
    std::vector<std::uint8_t> data = { 0x11, 0xFF, 0x00, 0x0D };
    auto s = Centurion::HexDump(data);
    assert(s == "11 FF 00 0D");
    std::puts("  HexDump: ok");
}

void TestHexDumpEmpty() {
    auto s = Centurion::HexDump(nullptr, 0);
    assert(s.empty());
    std::puts("  HexDump (empty): ok");
}

// ---- main ---------------------------------------------------------------------------------

int main() {
    std::puts("logitech_centurion_test: running...");

    TestPackMessageShort();
    TestPackMessageLong();
    TestPackRootQuery();
    TestPackBatteryQuery();

    TestParseReplyNormal();
    TestParseReplyWithReportId();
    TestParseReplyError();
    TestParseReplyTooShort();

    TestParseUnifiedBatteryNormal();
    TestParseUnifiedBatteryCharging();
    TestParseUnifiedBatteryFull();
    TestParseUnifiedBatteryCoarseLevel();
    TestParseUnifiedBatteryCritical();
    TestParseUnifiedBatteryTooShort();

    TestParseBatteryStatusNormal();
    TestParseBatteryStatusCharging();
    TestParseBatteryStatusChargeComplete();
    TestParseBatteryStatusInvalidPercent();

    TestHexDump();
    TestHexDumpEmpty();

    std::puts("logitech_centurion_test: ALL PASSED");
    return 0;
}
