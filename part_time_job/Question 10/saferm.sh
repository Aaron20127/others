#!/bin/bash

#### safe remove
#### usage: sh saferm.sh filenames

#1.check dir
junk_dir=${HOME}/.junk

if [ ! -d "${junk_dir}" ]; then
    mkdir ${junk_dir}
    echo "creat junk_dir"
fi

#2.Check for outdated files
for filename in $(ls ${junk_dir})
do
    var_date_string=$(printf "${filename}" | \
                sed 's/^[^-]*-//g' | \
                sed 's/\.[^\.]*$//g' | \
                sed 's/\./:/g')
    var_date_format_string=$(printf "${var_date_string}" | \
                    sed 's/-[^-]*$//g')" "$(printf "${var_date_string}" | \
                    sed 's/[0-9][0-9]-//g')
    declare -i var_date_last_seconds=$(date --date="${var_date_format_string}" +%s)
    declare -i var_date_now_seconds=$(date +%s)
    declare -i diff_seconds=$((${var_date_now_seconds}-${var_date_last_seconds}))

    if [ "${diff_seconds}" -gt "172800" ]; then
        rm -rf ${junk_dir}/${filename}
        echo "delete ${filename}"
    fi
done

#3.safe delete file
for filename in ${@}
do
    # 1.add pth
    file_dir=$(cd `dirname $filename`; pwd)
    filename_new=$(basename ${filename})
    absolute_path="${file_dir}/${filename_new}"

    # 2.tar
    if [ -f "${absolute_path}" ]; then
        date_string=$(date +-%y-%m-%d-%H.%M.%S)
        last_file_name=${filename_new}${date_string}.tgz
        tar -czvf  "${junk_dir}/${last_file_name}" "${absolute_path}" >/dev/null  2>&1 
        rm -rf ${absolute_path}

        echo "${absolute_path}  ===>  "${junk_dir}/${last_file_name}
    fi
done





























