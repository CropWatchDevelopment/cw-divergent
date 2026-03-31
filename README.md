TTN/TTI Decoder:

```javascript
function decodeUplink(input) {
  var data = {
    error: null // keep explicit for consistency
  };
  var warnings = [];
  var errors = []; // must remain an array

  // Human-readable error map (used by fPort 10, and referenced for 11)
  var port10Map = {
    0x01: "no sensor detected",
    0x02: "sensor 1 failure",
    0x03: "sensor 2 failure",
    0x04: "sensor validation failed", // sensors disagree
    0x05: "humidity validation failed" // humidity sensors disagree
  };

  // Reset reason map for fPort 9 byte 8
  var resetReasonMap = {
    0x01: "PIN Reset (NRST pin)",
    0x02: "POR/PDR (Power On Reset)",
    0x03: "Software Reset",
    0x04: "Independent Watchdog (IWDG)",
    0x05: "Window Watchdog (WWDG)",
    0x06: "Low Power Reset",
    0x00: "Unknown/None"
  };

  var diagStageMap = {
    0x00000000: "Unknown/None",
    0xB0010001: "Boot complete",
    0xB0010002: "Pre-STOP",
    0xB0010003: "Post-wake",
    0xB0010004: "STOP race skip",
    0xB0010005: "Alarm arm failed",
    0xB0010006: "WFI enter",
    0xB0010007: "WFI return",
    0xB0010008: "Restore start",
    0xB0010009: "Clocks restored",
    0xB001000A: "GPIO restored",
    0xB001000B: "I2C restored",
    0xB001000C: "UART restored",
    0xB001000D: "Restore complete"
  };

  // Calibration: 1413 µS/cm reference / mean 1706.4
  const CAL_EC_SCALE = 1413 / 1706.4; // ≈ 0.828303

  // helper(s)
  function toInt16(raw16) {
    if (raw16 > 32767) raw16 -= 65536; // int16
    return raw16;
  }
  function readU32BE(bytes, offset) {
    var value = 0;
    value += bytes[offset] * 16777216;     // 2^24
    value += bytes[offset + 1] * 65536;    // 2^16
    value += bytes[offset + 2] * 256;      // 2^8
    value += bytes[offset + 3];
    return value >>> 0;
  }
  function toHex32(value) {
    return "0x" + (value >>> 0).toString(16).toUpperCase().padStart(8, "0");
  }
  function diagStageName(stage) {
    var normalized = stage >>> 0;
    if ((normalized & 0xFFFF0000) === 0xB0F10000) {
      return "HardFault";
    }
    return diagStageMap.hasOwnProperty(normalized)
      ? diagStageMap[normalized]
      : "Unknown stage";
  }
  function decodeResetFlags(flags) {
    var names = [];
    var normalized = flags >>> 0;

    if ((normalized & 0x01000000) !== 0) names.push("Firewall reset");
    if ((normalized & 0x02000000) !== 0) names.push("OBL reset");
    if ((normalized & 0x04000000) !== 0) names.push("PIN reset");
    if ((normalized & 0x08000000) !== 0) names.push("POR/PDR reset");
    if ((normalized & 0x10000000) !== 0) names.push("Software reset");
    if ((normalized & 0x20000000) !== 0) names.push("Independent watchdog reset");
    if ((normalized & 0x40000000) !== 0) names.push("Window watchdog reset");
    if ((normalized & 0x80000000) !== 0) names.push("Low-power reset");

    return names;
  }
  function decodePwrCsrFlags(value) {
    var names = [];
    var normalized = value >>> 0;

    if ((normalized & 0x00000001) !== 0) names.push("Wakeup flag");
    if ((normalized & 0x00000002) !== 0) names.push("Standby flag");
    if ((normalized & 0x00000100) !== 0) names.push("WKUP1 enabled");
    if ((normalized & 0x00000200) !== 0) names.push("WKUP2 enabled");
    if ((normalized & 0x00000400) !== 0) names.push("WKUP3 enabled");

    return names;
  }
  function decodeRtcIsrFlags(value) {
    var names = [];
    var normalized = value >>> 0;

    if ((normalized & 0x00000001) !== 0) names.push("Alarm A write flag");
    if ((normalized & 0x00000020) !== 0) names.push("Registers synchronized");
    if ((normalized & 0x00000040) !== 0) names.push("Init mode flag");
    if ((normalized & 0x00000100) !== 0) names.push("Alarm A flag");
    if ((normalized & 0x00000400) !== 0) names.push("Wakeup timer flag");

    return names;
  }
  // T2 / legacy temps (no device offset)
  function toTempC(raw16) {
    if (raw16 > 32767) raw16 -= 65536; // int16
    return raw16 / 100.0;              // raw is centi-degrees
  }
  // T1 temps (device adds +5500; remove it here)
  function toTempC_T1(raw16) {
    if (raw16 > 32767) raw16 -= 65536; // int16
    return (raw16 - 5500) / 100.0;     // compensate +5500 then scale
  }

  try {
    // === Soil sensor packet: ε25(2) + EC(2) + T(2) + VWC(2) ===
    // Accept either fPort 2 (preferred) or fPort 1 with 8-byte payload
    if (input.fPort === 2 || (input.fPort === 1 && input.bytes.length === 8)) {
      if (input.bytes.length < 8) {
        errors.push("Payload too short on fPort 2 - expected 8 bytes");
        return { data, warnings, errors };
      }

      var e25_raw = (input.bytes[0] << 8) | input.bytes[1]; // ε25 * 100 (unused)
      var ec_raw  = (input.bytes[2] << 8) | input.bytes[3]; // µS/cm (already)
      var t_raw   = (input.bytes[4] << 8) | input.bytes[5]; // °C * 100
      var vwc_raw = (input.bytes[6] << 8) | input.bytes[7]; // % * 10

      var temperature = +toTempC(t_raw).toFixed(2);

      // EC: µS/cm -> apply calibration -> mS/cm
      var ec_mS_cm = +((ec_raw * CAL_EC_SCALE) / 1000.0).toFixed(2);

      // Moisture: % with 1 decimal, clamp to 100.0
      var moisture = +(vwc_raw / 10.0).toFixed(1);
      if (moisture > 100.0) moisture = 100.0;
      if (moisture < 0) moisture = 0.0;

      data = {
        temperature: temperature,
        moisture: moisture,
        ec: ec_mS_cm, // mS/cm
        ph: null
      };

      return { data, warnings, errors };
    }

    // === Port 1: normal packet: T1(2) + H1(2) ===
    if (input.fPort === 1) {
      data.error = null;

      if (input.bytes.length < 4) {
        errors.push("Payload too short on fPort 1 - expected 4+ bytes");
        return { data, warnings, errors };
      }

      var t1_raw = (input.bytes[0] << 8) | input.bytes[1];
      var h1_raw = (input.bytes[2] << 8) | input.bytes[3];
      data.temperature_c = toTempC_T1(t1_raw); // T1 uses offset-compensated path
      data.humidity = h1_raw / 100.0;

      if (data.temperature_c < -40 || data.temperature_c > 85) {
        warnings.push("Temperature out of typical range (-40..85°C)");
      }
      if (data.humidity > 100) {
        warnings.push("Humidity > 100%");
      }
      return { data, warnings, errors };
    }

    // === Port 3: pulse counter (uint32) ===
    // Layout: pulse_count(4 bytes, BE)
    if (input.fPort === 3) {
      if (input.bytes.length < 4) {
        errors.push("Payload too short on fPort 3 - expected 4 bytes");
        return { data, warnings, errors };
      }

      var pulseCount = readU32BE(input.bytes, 0);

      data = {
        pulse_count: pulseCount,
        received_at: input.received_at || null
      };

      return { data, warnings, errors };
    }

    // === Port 9: Sensor Serials + Reset Reason (+ optional reset diagnostics) ===
    // Legacy layout (9 bytes total):
    // 0-3: Sensor 1 Serial
    // 4-7: Sensor 2 Serial
    // 8:   Reset Reason Code
    //
    // Extended layout (17 bytes total):
    // 9-12:  Last diagnostic stage (RTC backup register snapshot)
    // 13-16: RCC->CSR reset flags snapshot from the last boot
    //
    // Extended wake-debug layout (29 bytes total):
    // 17-20: PWR->CSR snapshot at boot
    // 21-24: RTC->ISR snapshot at boot
    // 25-28: EXTI->PR snapshot for RTC Alarm line at boot
    if (input.fPort === 9) {
      data.error = null;

      if (input.bytes.length < 9) {
        errors.push("Payload too short on fPort 9 - expected 9, 17, or 29 bytes");
        return { data, warnings, errors };
      }

      var s1 = readU32BE(input.bytes, 0);
      var s2 = readU32BE(input.bytes, 4);
      var resetCode = input.bytes[8] & 0xFF;

      data.sensor1_serial = s1;
      data.sensor2_serial = s2;
      data.reset_reason_code = resetCode;
      data.reset_reason =
        resetReasonMap.hasOwnProperty(resetCode)
          ? resetReasonMap[resetCode]
          : "Unknown reset reason";

      if (input.bytes.length >= 17) {
        var diagStage = readU32BE(input.bytes, 9);
        var resetFlags = readU32BE(input.bytes, 13);

        data.diag_stage_code = diagStage;
        data.diag_stage_hex = toHex32(diagStage);
        data.diag_stage = diagStageName(diagStage);

        if ((diagStage & 0xFFFF0000) === 0xB0F10000) {
          data.hardfault_icsr_low16 = diagStage & 0xFFFF;
        }

        data.reset_flags_code = resetFlags;
        data.reset_flags_hex = toHex32(resetFlags);
        data.reset_flags = decodeResetFlags(resetFlags);
      }

      if (input.bytes.length >= 29) {
        var pwrCsr = readU32BE(input.bytes, 17);
        var rtcIsr = readU32BE(input.bytes, 21);
        var alarmExtiPr = readU32BE(input.bytes, 25);

        data.pwr_csr_code = pwrCsr;
        data.pwr_csr_hex = toHex32(pwrCsr);
        data.pwr_csr_flags = decodePwrCsrFlags(pwrCsr);
        data.pwr_wakeup_flag = (pwrCsr & 0x00000001) !== 0;

        data.rtc_isr_code = rtcIsr;
        data.rtc_isr_hex = toHex32(rtcIsr);
        data.rtc_isr_flags = decodeRtcIsrFlags(rtcIsr);
        data.rtc_alarm_a_flag = (rtcIsr & 0x00000100) !== 0;

        data.rtc_alarm_exti_pr_code = alarmExtiPr;
        data.rtc_alarm_exti_pr_hex = toHex32(alarmExtiPr);
        data.rtc_alarm_exti_pending = (alarmExtiPr & 0x00020000) !== 0;
      }

      return { data, warnings, errors };
    }

    // === Port 10: error-only (first byte is code) ===
    if (input.fPort === 10) {
      if (input.bytes.length < 1) {
        errors.push("Payload too short on fPort 10 - expected ≥1 byte");
        return { data, warnings, errors };
      }
      var code10 = input.bytes[0] & 0xFF;
      data.error = port10Map[code10] || ("unknown error code: " + code10);
      return { data, warnings, errors };
    }

    // === Port 11: sensors disagree — carry both sensors ===
    // Layout: T1(2) + H1(2) + T2(2) + H2(2)  => total 8 bytes
    if (input.fPort === 11) {
      if (input.bytes.length < 8) {
        errors.push("Payload too short on fPort 11 - expected 8 bytes");
        return { data, warnings, errors };
      }

      var t1r = (input.bytes[0] << 8) | input.bytes[1];
      var h1r = (input.bytes[2] << 8) | input.bytes[3];
      var t2r = (input.bytes[4] << 8) | input.bytes[5];
      var h2r = (input.bytes[6] << 8) | input.bytes[7];

      var t1c = toTempC_T1(t1r); // T1 with +5500 compensation
      var t2c = toTempC(t2r);    // T2 unchanged

      data.error = port10Map[0x04]; // "sensor validation failed"
      data.temperature1_c = t1c;
      data.humidity1 = h1r / 100.0;
      data.temperature2_c = t2c;
      data.humidity2 = h2r / 100.0;
      data.temp_delta_c = +(Math.abs(t1c - t2c).toFixed(2));

      if (data.humidity1 > 100 || data.humidity2 > 100) {
        warnings.push("Humidity value exceeds 100%");
      }
      if (t1c < -40 || t1c > 85) warnings.push("Sensor1 temperature out of range");
      if (t2c < -40 || t2c > 85) warnings.push("Sensor2 temperature out of range");

      return { data, warnings, errors };
    }

    // === Default: treat like port 1 ===
    if (input.bytes.length < 4) {
      errors.push("Payload too short - expected at least 4 bytes");
      return { data, warnings, errors };
    }

    var temp_raw = (input.bytes[0] << 8) | input.bytes[1];
    var hum_raw = (input.bytes[2] << 8) | input.bytes[3];
    data.temperature_c = toTempC_T1(temp_raw); // default path mirrors fPort 1 (T1)
    data.humidity = hum_raw / 100.0;

    if (data.temperature_c < -40 || data.temperature_c > 85) {
      warnings.push("Temperature out of typical range (-40..85°C)");
    }
    if (data.humidity > 100) {
      warnings.push("Humidity > 100%");
    }
  } catch (e) {
    errors.push("Error decoding payload: " + (e && e.message ? e.message : e));
  }

  return { data, warnings, errors };
}

// Legacy TTN decoder
function Decoder(bytes, port) {
  var result = decodeUplink({ bytes: bytes, fPort: port });
  return result.data;
}
```