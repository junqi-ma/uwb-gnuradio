#!/bin/sh
# Version/SONAME regression (2026-11-15 cleanup fix A): the adapter shared
# library must carry SONAME libgnuradio-uwb.so.<LIBVER> and the versioned
# file libgnuradio-uwb.so.<VERSION> must exist next to it.  This pins the
# include order VERSION_MAJOR/API/ABI/PATCH -> include(GrVersion).
#
# Usage: qa_uwb_gnuradio_uwb_version.sh <libgnuradio-uwb.so> <LIBVER> <VERSION>
set -eu
LIB=$1
LIBVER=$2
VERSION=$3
DIR=$(dirname "$LIB")
readelf -d "$LIB" | grep -q "SONAME.*\[libgnuradio-uwb.so.${LIBVER}\]"
test -f "$DIR/libgnuradio-uwb.so.${VERSION}"
echo "uwb_qa_gnuradio_uwb_version: SONAME=libgnuradio-uwb.so.${LIBVER} file=libgnuradio-uwb.so.${VERSION} OK"
