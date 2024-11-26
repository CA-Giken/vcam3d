#!/bin/bash
# 特定ファイルのパス
specific_file="$(rospack find vcam3d)/config/tfs.yaml"

# デフォルトのファイルのパス
default_file="$(rospack find vcam3d)/config/tfs.yaml"

# ファイルの存在を確認
if [ -f "$specific_file" ]; then
    echo "Loading parameters from $specific_file"
    rosparam load "$specific_file"
else
    echo "Loading default parameters from $default_file"
    rosparam load "$default_file"
fi