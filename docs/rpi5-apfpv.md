# Running PixelPilot RPi5 in AP-FPV Mode

AP-FPV (Access Point FPV) mode is for OpenIPC cameras configured to stream video
over standard WiFi rather than wifibroadcast.  The drone acts as a WiFi access
point; the RPi5 connects to it and receives RTP video on UDP port 5600.

## Network interface layout

| Interface | Normal (WFB) | AP-FPV |
|-----------|--------------|--------|
| wlan0 | Internet / home AP | Internet / home AP (unchanged) |
| wlan1 | monitor mode → wfb-ng | station mode → drone AP |

wlan0 is never touched; internet access stays up in both modes.

## Switching modes

Two helper scripts handle all the steps automatically:

### Switch to AP-FPV

```bash
sudo pixelpilot-apfpv [ssid] [password]
# defaults: ssid=OpenIPC  password=12345678
```

What it does:
1. Stops `wifibroadcast.service` (releases wlan1 from wfb-ng)
2. Takes wlan1 out of monitor mode → puts it in station mode
3. Hands wlan1 to NetworkManager
4. Connects to the drone's WiFi AP
5. Sets `rx_mode: APFPV` in `/etc/pixelpilot/pixelpilot.yaml`

### Switch back to WFB

```bash
sudo pixelpilot-wfb
```

What it does:
1. Disconnects wlan1 from the drone AP
2. Removes wlan1 from NetworkManager
3. Puts wlan1 back in monitor mode on the channel from `/etc/wifibroadcast.cfg`
4. Starts `wifibroadcast.service`
5. Restores `rx_mode: WFB` in `/etc/pixelpilot/pixelpilot.yaml`

## Running PixelPilot

After switching to AP-FPV mode, run PixelPilot normally:

```bash
cd /home/mcarr/openipc/PixelPilot_rk/build_rpi
sudo ./pixelpilot
```

The `rx_mode: APFPV` in the config disables the wfb-ng stats thread so there
are no connection errors to a wfb-ng API that isn't running.

## Camera-side setup (OpenIPC AP-FPV)

The OpenIPC camera must be in AP-FPV firmware mode.  In `/etc/majestic.yaml`:

```yaml
video0:
  codec: h265
  size: 1280x720
  fps: 60

udpSender:
  enabled: true
  host: 0.0.0.0    # broadcast, or set to RPi5's IP on the drone subnet
  port: 5600
  codec: h265
```

The drone's AP typically assigns addresses in the `192.168.0.x` range.
After connecting, verify:

```bash
ip addr show wlan1          # find your assigned IP
ping 192.168.0.1            # check drone reachability
sudo tcpdump -i wlan1 udp port 5600 -c 5   # confirm video packets arriving
```

## Changing drone credentials

If your drone uses a different SSID or password, pass them to the switch script:

```bash
sudo pixelpilot-apfpv "MyDroneSSID" "MyPassword"
```

The NetworkManager profile `pixelpilot-apfpv` is updated automatically each time.

## Troubleshooting

**wlan1 won't connect:** Run `nmcli dev wifi list` to confirm the drone SSID is
visible.  If not, the drone may not be in AP-FPV mode or is out of range.

**No video:** Confirm packets are arriving:
```bash
sudo tcpdump -i wlan1 udp port 5600 -c 10
```
Check the drone's `udpSender.host` is set to `0.0.0.0` (broadcast) or the RPi5's
assigned IP.

**wfb-ng won't start after switching back:** Check that wlan1 is in monitor mode:
```bash
iw dev wlan1 info
```
If it shows `type managed`, run `sudo pixelpilot-wfb` again.

**wfb-ng stats errors in log (APFPV mode):** Make sure `/etc/pixelpilot/pixelpilot.yaml`
has `rx_mode: APFPV` — this disables the wfb stats thread.
