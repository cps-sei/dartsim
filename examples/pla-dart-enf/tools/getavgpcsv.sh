for i in {1..100}; do grep "new P avg=" exp-s$i.log | cut -d= -f2 >> ~/research/dart-enf-test/a6t.csv; done
