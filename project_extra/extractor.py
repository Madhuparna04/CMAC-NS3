def metric_gen():
    filename = "manet_project.tr"
    with open(filename, "r") as f:
        lines = f.readlines()
    no_of_packets_transmitted = 0
    no_of_packets_received = 0

    no_of_icmp_packets = 0

    latencies = []
    throughputs = []
    pd_ratios = []

    tx_time = 0
    rx_time = 0
    i = 1
    for line in lines:
        line = line.split()
        pkt_time = float(line[1])
        if line[0] == "t":
            no_of_packets_transmitted += 1
            tx_time = pkt_time
        else:
            no_of_packets_received += 1
            rx_time = pkt_time

            if len(line) == 52 and line[45] == "ns3::Icmpv4Echo":
                no_of_icmp_packets += 1
                latencies.append((pkt_time,rx_time - tx_time))
        
        pdr = (no_of_packets_received / no_of_packets_transmitted)
        pd_ratios.append((pkt_time, pdr))

        size_of_icmp_packet = 64
        rxBytes = size_of_icmp_packet * no_of_icmp_packets
        # total_time_elapsed = pkt_time - 30
        total_time_elapsed = pkt_time
        throughput = rxBytes / total_time_elapsed
        throughputs.append((pkt_time, throughput))

        i = i + 1

    return [latencies, pd_ratios, throughputs]

def main():
    latencies, pd_ratios, throughputs = metric_gen()

    with open("metric_latency.txt", "w+") as f:
        for latency in latencies:
            f.write(str(latency[0]) + " " + str(latency[1]) + "\n")

    with open("metric_pdr.txt", "w+") as f:
        for pd_ratio in pd_ratios:
            f.write(str(pd_ratio[0]) + " " + str(pd_ratio[1]) + "\n")

    with open("metric_throughput.txt", "w+") as f:
        for throughput in throughputs:
            f.write(str(throughput[0]) + " " + str(throughput[1]) + "\n")

if __name__ == '__main__':
    main()