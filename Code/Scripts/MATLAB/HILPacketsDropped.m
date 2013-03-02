% This measures the dropped packets from an HIL run:
tout*100 - unique(latency_in(:,2))