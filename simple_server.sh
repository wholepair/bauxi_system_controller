#! /bin/sh

### BEGIN INIT INFO
# Provides:          simple_server.py
# Required-Start:    $local_fs $remote_fs
# Required-Stop:
# X-Start-Before:    rmnologin
# Default-Start:     5
# Default-Stop:
# Short-Description: Starts a simple python based HTTP server for file upload/download
### END INIT INFO

ROOT_DIR="/home/pi/bauxi/software/system_controller"

case "$1" in
    start)
        cd $ROOT_DIR
        echo "Starting service: simple_server.py"
        /sbin/runuser pi -c "nohup $ROOT_DIR/simple_server.py > $ROOT_DIR/simple_server.log 2>&1 &"
    ;;
    stop|reload|restart|force-reload|status)
    ;;
    *)
        echo "Usage: $N {start|stop|restart|force-reload|status}" >&2
        exit 1
    ;;
esac

exit 0


