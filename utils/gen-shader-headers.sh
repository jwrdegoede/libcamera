#!/bin/sh
set -x

if [ $# -lt 4 ]; then
	echo "Invalid arg count must be >= 5"
	exit 1
fi
src_dir="$1"; shift
build_dir="$1"; shift
build_path=$build_dir/"$1"; shift

cat <<EOF > "$build_path"
/* SPDX-License-Identifier: LGPL-2.1-or-later */
/* This file is auto-generated, do not edit! */
/*
 * Copyright (C) 2025, Linaro Ltd.
 *
 */

#pragma once

EOF

cat <<EOF >> "$build_path"
/*
 * List the names of the shaders at the top of
 * header for readability's sake
 *
EOF

for file in "$@"; do
	echo "file is $file"
	name=$(basename "$build_dir/$file" | tr '.' '_')
	echo " * unsigned char $name;" >> "$build_path"
done

echo "*/" >> "$build_path"

echo "/* Hex encoded shader data */" >> "$build_path"
for file in "$@"; do
	name=$(basename "$build_dir/$file")
	"$src_dir/utils/gen-shader-header.py" "$name" "$build_dir/$file" >> "$build_path"
	echo >> "$build_path"
done
