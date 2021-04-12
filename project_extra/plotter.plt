set terminal pdf
set out "graphs.pdf"

set title "Latency"
set xlabel "Simulation Time (in s)"
set ylabel "Latency (in s)"
plot "metric_latency.txt" using 1:2 with lines title "Latency"

set title "Packet Delivery Ratio"
set xlabel "Simulation Time (in s)"
set ylabel "Packet Delivery Ratio"
plot "metric_pdr.txt" using 1:2 with lines title "PDR"

set title "Throughput without OLSR"
set xlabel "Simulation Time (in s)"
set ylabel "Throughput (in bytes/s)"
plot "metric_throughput.txt" using 1:2 with lines title "Throughput"