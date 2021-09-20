#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/arwin/Documents/git/TIF160/armstrong/src/rosserial/rosserial_xbee"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/arwin/Documents/git/TIF160/armstrong/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/arwin/Documents/git/TIF160/armstrong/install/lib/python3/dist-packages:/home/arwin/Documents/git/TIF160/armstrong/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/arwin/Documents/git/TIF160/armstrong/build" \
    "/usr/bin/python3" \
    "/home/arwin/Documents/git/TIF160/armstrong/src/rosserial/rosserial_xbee/setup.py" \
     \
    build --build-base "/home/arwin/Documents/git/TIF160/armstrong/build/rosserial/rosserial_xbee" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/arwin/Documents/git/TIF160/armstrong/install" --install-scripts="/home/arwin/Documents/git/TIF160/armstrong/install/bin"
