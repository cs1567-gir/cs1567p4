#!/usr/bin/env bash

while true; do
    roslaunch cs1567p4 minimal.launch &
    sleep 5
    while true; do

        # count number of node processes
        NUM=$(ps -A | grep node | wc -l)
        # stop all node processes if less than 3 are active
        if [ ${NUM} -lt 3 ]
            then
                echo 'stopping base nodes.'
                # kills all node processes
                kill -9 `ps aux | grep node | grep -v grep | awk '{print $2}'`
                break
        fi

    done

done
