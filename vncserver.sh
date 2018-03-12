#! /bin/sh

### BEGIN INIT INFO
# Provides:          vncserver
# Required-Start:    $local_fs $remote_fs
# Required-Stop:
# X-Start-Before:    rmnologin
# Default-Start:     5
# Default-Stop:
# Short-Description: Starts vncserver on system startup
### END INIT INFO

ROOT_DIR="/home/pi/bauxi/software/system_controller"

case "$1" in
    start)
        echo "Starting service: vncserver"
        /sbin/runuser pi -c "vncserver > $ROOT_DIR/vncserver_start.log 2>&1"
    ;;
    stop|reload|restart|force-reload|status)
    ;;
    *)
        echo "Usage: $N {start|stop|restart|force-reload|status}" >&2
        exit 1
    ;;
esac

exit 0


