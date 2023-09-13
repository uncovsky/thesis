set xlabel "Objective 1"
set ylabel "Objective 2"
set output "output.png"
set pointsize 1
set autoscale 
set style fill transparent solid 0.3 noborder
plot 'starting_lower.txt' using 1:2 with linespoints lc rgb 'spring-green' title "Lower Bound", \
     'starting_upper.txt' using 1:2 with linespoints lc rgb 'light-red' title "Upper Bound"
