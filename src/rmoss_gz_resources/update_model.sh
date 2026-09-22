#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="$SCRIPT_DIR/resource/models"

# 让 xmacro4sdf 能解析 model://xxx/...
export IGN_GAZEBO_RESOURCE_PATH="$MODEL_DIR:${IGN_GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_MODEL_PATH="$MODEL_DIR:${GAZEBO_MODEL_PATH:-}"

for dir in "$MODEL_DIR"/*; do
    [ -d "$dir" ] || continue
    [ -f "$dir/model.sdf.xmacro" ] || continue

    echo "parse $dir"

    cd "$dir" || exit 1

    tempfile=$(mktemp temp.XXXXXX)

    if xmacro4sdf model.sdf.xmacro > "$tempfile"; then
        lines_num=$(wc -l < "$tempfile")

        if (( lines_num > 3 )); then
            cat "$tempfile" > model.sdf
            echo "generated: $dir/model.sdf"
        else
            echo "ERROR: generated content too short"
            cat "$tempfile"
        fi
    else
        echo "ERROR: xmacro4sdf failed: $dir"
    fi

    rm -f "$tempfile"
done