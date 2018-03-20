set xrange[0:320]
set yrange[0:320]
plot "lidar_reading.dat" using 1:2 with points
pause 3
reread
