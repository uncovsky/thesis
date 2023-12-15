set key font ", 16"
set tics font ", 16"
set pointsize 1
set autoscale 
set style fill transparent solid 0.3 noborder
plot 'curve.txt' using 1:2 with linespoints lc rgb 'spring-green' pt 5 title "Pareto Curve"
