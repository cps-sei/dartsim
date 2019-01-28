seq 100 | parallel --eta -n0 ./run.sh --seed={#} $* | grep ^csv | cut -d, -f2- > r2phase.csv

