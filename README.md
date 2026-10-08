# gpsdash

Realtime curses GNSS dashboard for the Sierra EM7455 in the Panasonic CF-20. Reads
NMEA straight from the modem (`/dev/ttyUSB2`), starts the engine with `$GPS_START`,
polls AT engine status, and can inject gpsOneXTRA + UTC time over QMI (`x` / `X`).

## Install

```sh
pipx install --system-site-packages ~/src/gpsdash      # system site for gi/libqmi (XTRA)
pipx install --system-site-packages --force ~/src/gpsdash   # after changes
```

`--system-site-packages` is needed only for XTRA injection (PyGObject + `gir1.2-qmi-1.0`
come from Debian). XTRA helpers re-run `python -m gpsdash` under `sudo -n`.

## Use

```sh
gpsdash                      # live, owns /dev/ttyUSB2
gpsdash --stop --serve 10110 # power GNSS down on exit; share NMEA on tcp/10110
gpsdash --replay log.nmea    # replay a capture
```

`--serve` lets gpsd or map apps share the receiver while gpsdash owns the port
(`gpsd -N -n tcp://127.0.0.1:10110`). Don't run gpsd on the tty at the same time.

In the OPS console (`$mod+o`) it lives in tmux window `2:GNSS`, launched by
`~/.config/ops/gpsdash`.
