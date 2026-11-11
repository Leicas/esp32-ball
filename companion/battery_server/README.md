# Wi-Fi battery recorder

Dashboard: http://192.168.1.158:8086 ; Docker host 192.168.1.158, /opt/esp32-battery. Compose named volume battery-data persists SQLite across container rebuilds; restart unless-stopped. LAN only, unauthenticated read UI and UDP ingestion: do not expose these ports to the internet. UDP destination port 9876, device sends every 15s. Duplicate device/boot/sequence is ignored. Every restart gets a new boot identifier; server timestamps and stores new readings without rearming the old onboard test. Packets lost during outages are not backfilled.

USB serial N<ssid> TAB <password> newline provisions device NVS; secrets are not in source or packets. I reports connection/IP. Normal haptic operation is preserved. Wi-Fi adds power consumption and timing load, so resulting runtime is for Wi-Fi-enabled firmware.

The voltage is VSYS, not a calibrated VBAT or percentage. USB_GOOD_N reports external power. CHG_N is connected to the charger LED only, not an MCU GPIO. The UI must not infer charge completion or an ETA from the regulated supply rail. Offline does not prove battery depletion. All-data CSV export retains gaps and boot identifiers.

Validation: receiver tested with temporary SQLite (deduplication, separate boot IDs, malformed input); real device packet received. Dedicated-charger restart verified: new boot identifier, six consecutive 15-second readings, USB present, LIGHTSABER gain 1 and zero reported audio drops. Freshness uses server-computed age to avoid client/server clock skew.

## Measured runtime estimator v1
Firmware sends battery_pct and remaining_s every 15 seconds, -1 while USB powered or settling (three samples after unplug). A 25% EMA filters voltage. Calibration is a monotone 21-point table of voltage vs remaining time in the recorded LIGHTSABER boot 2612fc15456e93d3, terminating at the first supply reading below 3300 mV (14.5627 h after that mode began). 0% is this selected usable-runtime reference, not pack protection cutoff; firmware does not change cutoff behavior. Source table: calibration.csv. This is a single-run, similar-load estimate, not coulomb counting or a charging gauge. Native tests verify monotonicity and bounds. Dashboard rounds remaining time to 15 minutes; CSV retains seconds and model ID.
