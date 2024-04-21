for i in $(seq 1 10)
do
    ./Apps/mono_euroc /home/nerea/SLAM/datasets/V101 Apps/EuRoC_TimeStamps/V101.txt > test.txt
    mv trajectory.txt trajectory_V101_$i.txt
done