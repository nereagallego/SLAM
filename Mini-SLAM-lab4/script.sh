dataset='MH01'
task='task2'
for i in $(seq 1 10)
do
    mkdir -p trajectories/trajectory_$dataset\_$task
    ./Apps/mono_euroc /home/nerea/SLAM/datasets/$dataset Apps/EuRoC_TimeStamps/$dataset.txt > test.txt
    mv trajectory.txt trajectories/trajectory_$dataset\_$task/trajectory_$dataset\_$task\_$i.txt
done