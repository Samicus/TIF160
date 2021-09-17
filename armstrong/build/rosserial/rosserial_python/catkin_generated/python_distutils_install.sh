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

echo_and_run cd "/home/samnehme/Dev/armstrong/src/rosserial/rosserial_python"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/samnehme/Dev/armstrong/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/samnehme/Dev/armstrong/install/lib/python3/dist-packages:/home/samnehme/Dev/armstrong/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/samnehme/Dev/armstrong/build" \
    "/usr/bin/python3" \
    "/home/samnehme/Dev/armstrong/src/rosserial/rosserial_python/setup.py" \
     \
    build --build-base "/home/samnehme/Dev/armstrong/build/rosserial/rosserial_python" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/samnehme/Dev/armstrong/install" --install-scripts="/home/samnehme/Dev/armstrong/install/bin"
