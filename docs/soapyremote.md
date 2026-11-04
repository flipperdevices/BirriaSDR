# SoapyRemote

SoapyRemote serves a SoapySDR device over the network. The Flipper One runs
`SoapySDRServer` next to the receiver, and an application on a PC opens it as a
`remote` device, as if it were plugged in locally. The PC needs no BirriaSDR
module, only SoapySDR and the SoapyRemote client module.

## On the Flipper

Set up the driver and library as the [Flipper One guide](flipper-one.md)
describes, then install the server:

```sh
sudo apt install soapyremote-server
```

Take the chip and start the server as the same user, with the library's
threads on the fast cores:

```sh
sudo mt7921-sdr-hold
BIRRIA_CPUS=4-7 SoapySDRServer --bind
```

The server listens on port 55132 on every interface and opens the receiver only
when a client connects. Stop it with Ctrl+C. To keep it running after logout,
start it with `setsid -f` and stop it with `pkill -x SoapySDRServer`.

### As a service

Instead of starting the server by hand, it can run as a system service. The
package ships `soapyremote-server.service`, disabled. Add a drop-in that sets
`BIRRIA_CPUS` and takes the chip before the server starts, then enable the
service:

```sh
sudo mkdir -p /etc/systemd/system/soapyremote-server.service.d
sudo tee /etc/systemd/system/soapyremote-server.service.d/birria.conf <<'EOF'
[Service]
Environment=BIRRIA_CPUS=4-7
ExecStartPre=/usr/local/bin/mt7921-sdr-hold
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now soapyremote-server
```

While the service is enabled, the chip is the receiver from every boot on, so
Wi-Fi and Bluetooth are not available on the Flipper. To get them back, disable
the service and release the chip:

```sh
sudo systemctl disable --now soapyremote-server
sudo mt7921-sdr-release
```

## On the PC

Install SoapySDR, its tools and the SoapyRemote client module:

| Distribution | Packages |
| --- | --- |
| Debian, Ubuntu | `soapysdr-tools soapysdr0.8-module-remote` |
| Arch | `soapysdr soapyremote` |

Check that the PC reaches the receiver, with the Flipper's address or name:

```sh
SoapySDRUtil --find="remote=flipperone-45d248"
SoapySDRUtil --probe="remote=flipperone-45d248,remote:driver=birria"
```

The device it lists is the same `FlipperOne SDR` with the same controls as on
the Flipper, which the [SoapySDR module guide](soapy.md) describes.

## Device and stream arguments

Open the receiver with the device arguments
`remote=<flipper>,remote:driver=birria`. These stream arguments change how
samples cross the network:

| Argument | Values | Effect |
| --- | --- | --- |
| `remote:format` | `CS16` (default), `CS8`, `CF32` | The format on the wire. The client converts it to the format the application asks for. `CS8` halves the traffic and carries the module's 8-bit samples |
| `remote:prot` | `udp` (default), `tcp` | `tcp` costs the Flipper about 40 percent less CPU at the same rate and loses nothing on a clean link |
| `remote:mtu` | bytes, default 1500 | The UDP datagram size. Leave it unless the link carries jumbo frames |

In GNU Radio, the Soapy source block takes the device arguments in its device
field and the stream arguments in its stream args field:

```text
device: remote=flipperone-45d248,remote:driver=birria
stream: remote:format=CS8,remote:prot=tcp
```

SDR++ lists the receiver in its SoapySDR source, which Debian's `sdrpp` builds and
Arch's `sdrpp-git` leaves out. The source finds servers on the local network by
itself. It has no stream args field, so it always gets `CS16` over UDP, and
its gain sliders do not show the gain the AGC picks.

## What fits the link

Each complex sample takes 4 bytes in `CS16`, 2 in `CS8` and 8 in `CF32`. Measured
from a PC on gigabit Ethernet, which carries about 940 Mbit/s:

| Rate | `CS16` | `CS8` |
| --- | --- | --- |
| 10 MSps | 320 Mbit/s, clean | 160 Mbit/s, clean |
| 20 MSps | 640 Mbit/s, clean | 320 Mbit/s, clean |
| 40 MSps | 1280 Mbit/s, does not fit: about 24 to 29 MSps arrive, with gaps | 640 Mbit/s, clean |
| 80 MSps | does not fit | does not fit |

When the link is too slow, the client reports gaps as `SOAPY_SDR_OVERFLOW` and
the stream keeps running. The USB network link (NCM) runs at USB 2 speed, about
300 Mbit/s, so over it 10 MSps needs `CS8`. Wi-Fi is not an option: the chip that
would carry it is the receiver.

At 20 MSps `CS16` over UDP, the server takes most of one A72 core and the
Flipper as a whole is about 20 percent busy.

## Troubleshooting

| Symptom | Cause |
| --- | --- |
| `--find` lists nothing | The server is not running, or a firewall blocks port 55132 |
| The device is listed but does not open | The chip is not held, the server runs as a user `mt7921-sdr-hold` did not give access to, or another program has the receiver. One client at a time |
| Regular gaps at a rate that should fit | Switch to `remote:prot=tcp` or `remote:format=CS8`. For UDP, raising `net.core.rmem_max` and `net.core.wmem_max` on both ends, as `/usr/share/doc/soapyremote-server/README.Debian` describes, gives the sockets more room |
