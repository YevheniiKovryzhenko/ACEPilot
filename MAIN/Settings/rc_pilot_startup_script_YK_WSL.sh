:set fileformat=unix
#!/bin/sh
exec echo temppwd | sudo -u root -S /mnt/d/GitHub/rc_pilot_v2/out/build/MAIN/rc_pilot -s /mnt/d/GitHub/rc_pilot_v2/out/build/MAIN/Settings/YK_settings_X.json "$@"