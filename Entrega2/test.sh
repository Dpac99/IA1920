#!/usr/bin/env bash

FILE=NDruagomesfreiregame2.py
TOTAL=0
MAX=500

for i in $(eval echo "{1..$MAX}"); do
    NUM=$(python $FILE | grep 'Grade' | grep -o -E '[0-9]+')
    TOTAL=$((TOTAL + NUM))
done

echo "Average grade:" 
echo "scale=2 ; $TOTAL / $MAX" | bc