#!/bin/bash
#!/usr/local/bin/env python3.7
numTries=10

for ((i=0; i < 12; i++)); do

  python3 search.py f $i

done
