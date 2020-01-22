#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
export LD_LIBRARY_PATH="$DIR/x86_64"
for f in ./x86_64/*; do
    # do some stuff here with "$f"
    # remember to quote it or spaces may misbehave
    MISSING_LIB="$(ldd "$f" | grep -e "not found")"
    if ! test -z "$MISSING_LIB"
    then
	    echo "$f"
	    echo "$MISSING_LIB"
    fi
done
