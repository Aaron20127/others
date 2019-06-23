#!/bin/bash

#### usage: sh myAWK.sh "able"
#### https://opengers.github.io/linux/awk-format-print/ 

echo $1 | awk 'BEGIN {FS=""} { 
    for(i=1;i<=NF;i++) {
        if($1 ~ /[aeiou]/) {
            printf $i;
        }
        else if($2 ~ /[aeiou]/) {
            if(i == NF) {
                printf $1;
            }
            else {
                printf $(i+1);
            }
        }
        else {
            if(i == (NF-1)) {
                printf $1;
            }
            else if(i == NF) {
                printf $2;
            }
            else {
                printf $(i+2);
            }
        }
    }
    printf "ay\n"
}' 
