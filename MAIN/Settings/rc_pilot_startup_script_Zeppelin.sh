:set fileformat=unix
#!/bin/sh
exec echo temppwd | sudo -u root -S /home/debian/rc_pilot/MAIN/rc_pilot -s /home/debian/rc_pilot/MAIN/Settings/YK_settings_X_Zeppelin.json "$@"