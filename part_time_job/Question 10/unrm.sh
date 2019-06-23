#!/bin/bash

### recover
### usage: sh unrm.sh filenames-19-02-17-22.16.33.tgz 

junk_dir=${HOME}/.junk

for filename in ${@}
do
    file_path=${junk_dir}/${filename}
    if [ -f "${file_path}" ]; then
       
        cd ${junk_dir}
        file_pro=$(tar zxvf ${filename})
        origin_dir="/"$(dirname ${file_pro})
        mv ${file_pro} ${origin_dir}
        dir=$(printf ${file_pro} | sed 's/\/.*$//g')
        rm -rf ${filename}
        rm -rf ${dir}

        # print
        base_name=$(basename ${file_pro})
        echo "$file_path  ===>  ${origin_dir}/${base_name}"
    fi
done

