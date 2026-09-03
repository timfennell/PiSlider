# PiSlider Setup

Two parts, deliberately separated:

- **[Part 1 — Connect and Shoot](#part-1--connect-and-shoot)** is for anyone handed a
  working rig. No terminal, no SSH. Read this one.
- **[Part 2 — Build a Rig from Scratch](#part-2--build-a-rig-from-scratch)** is for
  setting up a fresh Raspberry Pi. Assumes comfort with the command line.

---

# Part 1 — Connect and Shoot

The PiSlider makes its own WiFi network. You join it with a laptop, phone, or tablet
and drive everything from a browser. No internet, no router, no app to install.

## 1. Power on

Give it about **45 seconds**. The Pi boots, brings up its WiFi, and starts the server.

## 2. Join the PiSlider WiFi

On your laptop or phone, open WiFi settings and connect to:

| | |
|---|---|
| **Network** | `PiSlider` |
| **Password** | ask whoever set up the rig |

Your device will say something like *"no internet connection."* That is expected and
correct — the slider is not a router. Stay connected to it.

## 3. Open the app

In any browser:

```
http://pislider
```

If that doesn't load, use the numeric address, which always works:

```
http://10.42.0.1
```

You should see **PiSlider — Web Command Center**. That's it — you're driving the rig.

> Bookmark `http://10.42.0.1`. It never changes, and it works even when name lookup
> doesn't.

## 4. If it doesn't load

Work down this list in order. Nine times out of ten it's the first two.

**"I can't find the `PiSlider` network."**
Wait the full 45 seconds and rescan. The radio comes up before the server does. If it
still isn't there after two minutes, the Pi likely didn't boot — check power and that
the status LED is lit.

**"I'm connected but the page won't load."**
Check you're actually still on `PiSlider`. Phones and laptops will silently jump back
to a remembered network with real internet. On iPhone, turn off *Settings → WiFi → Auto-Join*
for your home network while shooting. On a Mac, uncheck *Auto-join* for other networks.

**"`http://pislider` says it can't find the site."**
Use `http://10.42.0.1` instead. Some Windows and Android devices don't do the local
name lookup that address depends on.

**"The browser tried to search the web instead of opening the page."**
Type the full address including `http://`. Without it, browsers treat `pislider` as a
search term.

**"The page loads but there's no camera image."**
The camera can fail to initialize while everything else works fine. See
[Camera not detected](#camera-not-detected) in Part 2 — this one needs a terminal.

**Still stuck?**
Power-cycle the Pi and wait the full 45 seconds before rejoining.

## 5. Good to know

- **Two radios.** One broadcasts the `PiSlider` network you connect to. The second is
  reserved for talking to a Sony camera over WiFi Direct, so pairing a camera does not
  kick you off.
- **Multiple people can connect at once** — useful for one person on the camera and
  another watching the frame.
- **It doesn't need internet** and doesn't phone home.

---

# Part 2 — Build a Rig from Scratch

Everything below assumes a fresh Raspberry Pi OS install and SSH access.

## Hardware

- Raspberry Pi 5 (4GB or 8GB)
- microSD card, 32GB+ Class 10 / A2
- IMX477 camera module or compatible
- Stepper controllers on GPIO / UART
- 12V supply rated for the motors plus the Pi
- Optional: NeoPixel strip for status
- **Two WiFi radios.** The Pi 5's built-in radio runs the hotspot; a USB WiFi adapter
  gives you the second interface for the Sony camera link. With only one radio you
  must choose between hosting the hotspot and talking to a camera.

## 1. OS install

Flash **Raspberry Pi OS (64-bit, bookworm)** with Raspberry Pi Imager. In advanced
settings set hostname `pislider`, enable SSH, and configure your home WiFi so you can
reach it for the initial build.

## 2. Dependencies

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python3-venv libcamera-apps nginx \
    avahi-daemon network-manager git
```

## 3. Install PiSlider

```bash
mkdir -p ~/Projects && cd ~/Projects
git clone https://github.com/timfennell/PiSlider.git pislider
cd pislider
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

> The virtualenv **must** be named `.venv`. `run.sh` sources `.venv/bin/activate` by
> absolute path and the service will not start if it's called anything else.

## 4. Create the WiFi hotspot

This step is **not** handled by any script in the repo — you run it once by hand.

Identify your interfaces first:

```bash
nmcli device status
```

Pick the radio to host the access point (below, `wlan0`) and leave the other free for
the camera.

```bash
sudo nmcli device wifi hotspot \
    ifname wlan0 \
    con-name PiSlider \
    ssid PiSlider \
    password 'CHOOSE-A-PASSWORD'

sudo nmcli connection modify PiSlider \
    connection.autoconnect yes \
    connection.autoconnect-priority 10
```

This creates a WPA-PSK access point in NetworkManager's `shared` mode, which puts the
Pi at **10.42.0.1/24** and runs a scoped `dnsmasq` for DHCP and DNS on that interface
automatically. You do not need to configure the standalone `dnsmasq` service, and on a
working rig it is typically inactive.

Verify:

```bash
nmcli -t -f NAME,DEVICE,AUTOCONNECT connection show --active
ip -4 addr show wlan0 | grep inet     # expect 10.42.0.1/24
```

To change the password later without disturbing anything else:

```bash
sudo nmcli connection modify PiSlider wifi-sec.psk 'NEW-PASSWORD'
sudo nmcli connection up PiSlider
```

## 5. Hostname, mDNS and reverse proxy

```bash
sudo bash setup_hostname.sh
```

This script does four things — and note that **creating the hotspot is not one of
them**:

1. Sets the hostname to `pislider`
2. Enables `avahi-daemon` so `pislider.local` resolves on a normal LAN
3. Writes `/etc/dnsmasq.d/pislider.conf` so the bare name `pislider` resolves for
   hotspot clients
4. Installs an nginx reverse proxy so port **80** forwards to the app on **8000**,
   which is what lets people use `http://10.42.0.1` with no port number

## 6. Services

```bash
sudo bash install-service.sh
```

Two units, and the relationship between them is easy to trip over:

- **`pislider.service`** — the app. Runs as root from `run.sh`, which activates `.venv`
  and supervises `app.py` with automatic restart.
- **`pislider-wake.service`** — a lightweight listener that starts the main server when
  a client appears, so the heavy process isn't running on battery when nobody's
  connected. It declares `Conflicts=pislider.service`, and `pislider.service` starts it
  from `ExecStopPost`.

**Restarting is not `systemctl restart`.** Because stopping the app fires the wake unit,
and the wake unit conflicts with the app, a restart gets cancelled mid-flight and you're
left in wake mode:

```bash
sudo systemctl restart pislider     # WRONG — "Job for pislider.service canceled."
sudo systemctl start pislider       # correct
```

Check state with:

```bash
systemctl is-active pislider pislider-wake
```

## 7. Updating

The deployment directory is a normal git checkout, so:

```bash
cd ~/Projects/pislider
git pull
sudo systemctl start pislider
```

`core.fileMode` is set to `false` on the deployment checkout so executable-bit
differences don't show up as spurious modifications.

## Connecting a Sony camera

The camera link uses the **second** radio, so it never disturbs the hotspot. Put the
camera into WiFi Direct mode, then:

```bash
nmcli device wifi connect "DIRECT-XXXX" ifname wlan1
```

The app reports a clear message on startup when that interface is attached to something
that isn't a camera network.

## Troubleshooting

### Camera not detected

```
ERROR:PiSlider:PiCamera2 both configs failed. Full: IndexError. Preview: IndexError
```

The server runs fine without a camera, so this is easy to miss — motors work, the UI
loads, and only the image is missing. Check in this order:

```bash
libcamera-hello --list-cameras     # does the OS see the sensor at all?
```

If that lists nothing, it's below the application layer: reseat the ribbon cable
(easy to get backwards or not fully latched), confirm the camera is enabled in
`sudo raspi-config`, and check `dmesg | grep -i imx` for detection errors.

### Hotspot doesn't come up on boot

```bash
nmcli -t -f NAME,DEVICE,AUTOCONNECT connection show
```

Confirm the `PiSlider` connection shows `yes` for autoconnect and is bound to the right
interface. If a second saved network is winning, raise the hotspot's priority:

```bash
sudo nmcli connection modify PiSlider connection.autoconnect-priority 10
```

### Port 80 doesn't work but 8000 does

nginx isn't running or isn't proxying:

```bash
systemctl is-active nginx
curl -s -o /dev/null -w '%{http_code}\n' http://localhost/
```

Re-run `sudo bash setup_hostname.sh` to reinstall the proxy config.

### Reading the logs

```bash
sudo journalctl -u pislider -n 100 --no-pager
sudo journalctl -u pislider -f            # live
```

## Reference

| | |
|---|---|
| Hotspot SSID | `PiSlider` |
| Hotspot address | `10.42.0.1/24` on the AP interface |
| Hostname | `pislider` / `pislider.local` |
| App port | 8000, proxied to 80 by nginx |
| Install path | `~/Projects/pislider` |
| Virtualenv | `.venv` (name is required) |
| Repository | https://github.com/timfennell/PiSlider.git |
