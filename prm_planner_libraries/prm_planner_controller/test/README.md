# Torque 

Files are stored to /tmp => goto tmp and start gnuplot.

- Plotting pose errors:
<code>plot "path_errors.txt" u 1:2 w l title "x","path_errors.txt" u 1:3 w l title "y","path_errors.txt" u 1:4 w l title "z","path_errors.txt" u 1:5 w l title "r","path_errors.txt" u 1:6 w l title "p","path_errors.txt" u 1:7 w l title "y"</code>

- Plotting velocity errors:
<code>plot "path_errors_dot.txt" u 1:2 w l title "x","path_errors_dot.txt" u 1:3 w l title "y","path_errors_dot.txt" u 1:4 w l title "z","path_errors_dot.txt" u 1:5 w l title "r","path_errors_dot.txt" u 1:6 w l title "p","path_errors_dot.txt" u 1:7 w l title "y"</code>

- Plotting path:
<code>plot "path.txt" u 1:2 w l, "path_gt.txt" u 1:2 w l</code>
