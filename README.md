# hidclient

My tweaks to [hidclient](http://anselm.hoffmeister.be/computer/hidclient/index.html.en).

Original software is GPL'ed, written by Anselm Martin Hoffmeister.

## Current status

Currently, this works, but it's definitely not a smooth setup process. Quirks:

- Requires `--compat` flag to `bluetoothd`
    - (possibly unneeded if not publishing SDP records)
- Only works once per `bluetoothd` session
    - Need to restart `bluetooth` service on host after disconnecting client
- `bluetoothd` needs to be restarted *after* `hidclient` is started

## Full example usage

This is what works for me running Arch Linux on both machines.

### One-time setup steps

On host machine:

    ## compile this software
    [host hidclient]$ make

    ## Edit /usr/lib/systemd/system/bluetooth.service to change this line:
    ExecStart=/usr/lib/bluetooth/bluetoothd

    ## To:
    ExecStart=/usr/lib/bluetooth/bluetoothd --compat

    ## Determine correct IDs for use below:
    [host hidclient]$ XAUTHORITY=$HOME/.Xauthority sudo ./hidclient -l

### Each time connecting

On the system running hidclient, in the source directory:

    ## The numbers provided to the -e flag(s) will vary for your system
    [host hidclient]$ XAUTHORITY=$HOME/.Xauthority sudo ./hidclient -x -e0 -e17 -e18
      HID keyboard/mouse service registered
      Opened /dev/input/event0 as event device [counter 0]
      Opened /dev/input/event17 as event device [counter 1]
      Opened /dev/input/event18 as event device [counter 2]
      The HID-Client is now ready to accept connections from another machine

On the system running hidclient, in another terminal emulator:

    [host ~]$ sudo systemctl restart bluetooth.service
    [host ~]$ bluetoothctl
      [bluetooth]# power on
      Changing power on succeeded
      [bluetooth]# agent on
      Agent registered
      [bluetooth]# default-agent
      Default agent request successful
      [bluetooth]#

On the client system:

    [client ~]$ bluetoothctl
      [bluetooth]# power on
      Changing power on succeeded
      [bluetooth]# agent on
      Agent registered
      [bluetooth]# default-agent
      Default agent request successful
      [bluetooth]# connect XX:XX:XX:XX:XX:XX
      Attempting to connect to XX:XX:XX:XX:XX:XX
      [CHG] Device XX:XX:XX:XX:XX:XX Connected: yes
      Connection successful
      [host]#
